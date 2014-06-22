/*
 * visualodometry_.cpp
 *
 *  Created on: 2013-01-16
 *      Author: mathieu
 */

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/util3d.h>
#include "rtabmap/MsgConversion.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"

using namespace rtabmap;

class VisualOdometry
{
public:
	VisualOdometry() :
		odometry_(0),
		frameId_("base_link"),
		odomFrameId_("odom"),
		sync_(0),
		paused_(false)
	{
		ros::NodeHandle nh;

		odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);

		ros::NodeHandle pnh("~");

		int queueSize = 5;
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("queue_size", queueSize, queueSize);

		//parameters
		rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap parametersOdom;
		for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			std::string group = uSplit(iter->first, '/').front();
			if((group.compare("Odom") == 0 ||
				group.compare("SURF") == 0 ||
				group.compare("SIFT") == 0 ||
				group.compare("ORB") == 0 ||
				group.compare("FAST") == 0 ||
				group.compare("FREAK") == 0 ||
				group.compare("BRIEF") == 0)
			   &&
			   group.compare("OdomICP") != 0)
			{
				parametersOdom.insert(*iter);
			}
		}

		for(rtabmap::ParametersMap::iterator iter=parametersOdom.begin(); iter!=parametersOdom.end(); ++iter)
		{
			std::string vStr;
			bool vBool;
			int vInt;
			double vDouble;
			if(pnh.getParam(iter->first, vStr))
			{
				ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
				iter->second = vStr;
			}
			else if(pnh.getParam(iter->first, vBool))
			{
				ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
				iter->second = uBool2Str(vBool);
			}
			else if(pnh.getParam(iter->first, vInt))
			{
				ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
				iter->second = uNumber2Str(vInt);

				if(iter->first.compare(Parameters::kOdomMinInliers()) == 0 && vInt < 8)
				{
					ROS_WARN("Parameter min_inliers must be >= 8, setting to 8...");
					iter->second = uNumber2Str(8);
				}
			}
			else if(pnh.getParam(iter->first, vDouble))
			{
				ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
				iter->second = uNumber2Str(vDouble);
			}
		}

		odometry_ = new rtabmap::OdometryBOW(parametersOdom);

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		image_mono_sub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, "camera_info", 1);

		sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), image_mono_sub_, image_depth_sub_, info_sub_);
		sync_->registerCallback(boost::bind(&VisualOdometry::callback, this, _1, _2, _3));

		resetSrv_ = nh.advertiseService("reset_odom", &VisualOdometry::reset, this);
		pauseSrv_ = nh.advertiseService("pause_odom", &VisualOdometry::pause, this);
		resumeSrv_ = nh.advertiseService("resume_odom", &VisualOdometry::resume, this);
	}

	~VisualOdometry()
	{
		ros::NodeHandle pnh("~");
		ParametersMap parameters = Parameters::getDefaultParameters();
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			pnh.deleteParam(iter->first);
		}

		delete sync_;
		delete odometry_;
	}

	void callback(const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo)
	{
		if(!paused_)
		{
			if(!(image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
			   !(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
				 depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0))
			{
				ROS_ERROR("Input type must be image=mono8,rgb8,bgr8 (mono8 recommended) and image_depth=16UC1");
				return;
			}
			else if(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0)
			{
				static bool warned = false;
				if(!warned)
				{
					ROS_WARN("Input depth type is 32FC1, please use type 16UC1 for depth. The depth images "
							 "will be processed anyway but with a conversion. This warning is only be printed once...");
					warned = true;
				}
			}

			tf::StampedTransform localTransform;
			try
			{
				tfListener_.lookupTransform(frameId_, image->header.frame_id, image->header.stamp, localTransform);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			if(image->data.size() && depth->data.size() && cameraInfo->K[4] != 0)
			{
				float depthConstant = 1.0f/cameraInfo->K[4];
				cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(image);
				cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depth);

				rtabmap::Image data(ptrImage->image,
						ptrDepth->image.type() == CV_32FC1?util3d::cvtDepthFromFloat(ptrDepth->image):ptrDepth->image,
						depthConstant,
						rtabmap::Transform(),
						rtabmap::transformFromTF(localTransform));
				rtabmap::Transform pose = odometry_->process(data);
				if(!pose.isNull())
				{
					//*********************
					// Update odometry
					//*********************
					tf::Transform poseTF;
					rtabmap::transformToTF(pose, poseTF);

					tfBroadcaster_.sendTransform( tf::StampedTransform (poseTF, image->header.stamp, odomFrameId_, frameId_));

					if(odomPub_.getNumSubscribers())
					{
						//next, we'll publish the odometry message over ROS
						nav_msgs::Odometry odom;
						odom.header.stamp = image->header.stamp; // use corresponding time stamp to image
						odom.header.frame_id = odomFrameId_;
						odom.child_frame_id = frameId_;

						//set the position
						odom.pose.pose.position.x = poseTF.getOrigin().x();
						odom.pose.pose.position.y = poseTF.getOrigin().y();
						odom.pose.pose.position.z = poseTF.getOrigin().z();
						tf::quaternionTFToMsg(poseTF.getRotation().normalized(), odom.pose.pose.orientation);

						//publish the message
						odomPub_.publish(odom);
					}
				}
				else
				{
					//ROS_WARN("Odometry lost!");

					//send null pose to notify that odometry is lost
					nav_msgs::Odometry odom;
					odom.header.stamp = image->header.stamp; // use corresponding time stamp to image
					odom.header.frame_id = odomFrameId_;
					odom.child_frame_id = frameId_;

					//publish the message
					odomPub_.publish(odom);
				}
			}

			ROS_INFO("Odom update time(%f s)", (ros::WallTime::now()-time).toSec());
		}
	}

	bool reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		ROS_INFO("visual_odometry: reset odom!");
		odometry_->reset();
		return true;
	}

	bool pause(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		if(paused_)
		{
			ROS_WARN("visual_odometry: Already paused!");
		}
		else
		{
			paused_ = true;
			ROS_INFO("visual_odometry: paused!");
		}
		return true;
	}

	bool resume(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
	{
		if(!paused_)
		{
			ROS_WARN("visual_odometry: Already running!");
		}
		else
		{
			paused_ = false;
			ROS_INFO("visual_odometry: resumed!");
		}
		return true;
	}

private:
	rtabmap::Odometry * odometry_;

	// parameters
	std::string frameId_;
	std::string odomFrameId_;

	ros::Publisher odomPub_;
	ros::ServiceServer resetSrv_;
	ros::ServiceServer pauseSrv_;
	ros::ServiceServer resumeSrv_;
	tf::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	image_transport::SubscriberFilter image_mono_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;

	bool paused_;
};

int main(int argc, char *argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	ros::init(argc, argv, "visual_odometry");

	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
			rtabmap::ParametersMap parametersOdom;
			if(strcmp(argv[i], "--params") == 0)
			{
				// show specific parameters
				for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					if((uSplit(iter->first, '/').front().compare("Odom") == 0 ||
						uSplit(iter->first, '/').front().compare("OdomBin") == 0 ||
						uSplit(iter->first, '/').front().compare("Kp") == 0 ||
						uSplit(iter->first, '/').front().compare("SURF") == 0 ||
						uSplit(iter->first, '/').front().compare("SIFT") == 0)
					   &&
					   uSplit(iter->first, '/').front().compare("OdomICP") != 0)
					{
						parametersOdom.insert(*iter);
					}
				}
			}

			for(rtabmap::ParametersMap::iterator iter=parametersOdom.begin(); iter!=parametersOdom.end(); ++iter)
			{
				std::string str = "Param: " + iter->first + " = \"" + iter->second + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						rtabmap::Parameters::getDescription(iter->first).c_str() <<
						"]" <<
						std::endl;
			}
			ROS_WARN("Node will now exit after showing default odometry parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}
	}

	VisualOdometry vOdom;
	ros::spin();
	return 0;
}
