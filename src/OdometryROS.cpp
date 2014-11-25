/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "OdometryROS.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap {

OdometryROS::OdometryROS(int argc, char * argv[]) :
	odometry_(0),
	frameId_("base_link"),
	odomFrameId_("odom"),
	publishTf_(true),
	paused_(false)
{
	this->processArguments(argc, argv);

	ros::NodeHandle nh;

	odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
	odomLocalMap_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_map", 1);
	odomLastFrame_ = nh.advertise<sensor_msgs::PointCloud2>("odom_last_frame", 1);

	ros::NodeHandle pnh("~");

	Transform initialPose = Transform::getIdentity();
	std::string initialPoseStr;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
	pnh.param("publish_tf", publishTf_, publishTf_);
	pnh.param("initial_pose", initialPoseStr, initialPoseStr); // "x y z roll pitch yaw"
	if(initialPoseStr.size())
	{
		std::vector<std::string> values = uListToVector(uSplit(initialPoseStr, ' '));
		if(values.size() == 6)
		{
			initialPose = Transform(
					atof(values[0].c_str()), atof(values[1].c_str()), atof(values[2].c_str()),
					atof(values[3].c_str()), atof(values[4].c_str()), atof(values[5].c_str()));
		}
		else
		{
			ROS_ERROR("Wrong initial_pose format: %s (should be \"x y z roll pitch yaw\" with angle in radians). "
					  "Identity will be used...", initialPoseStr.c_str());
		}
	}


	//parameters
	rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(uStrContains(group, "Odom") ||
			group.compare("Stereo") ||
			group.compare("SURF") == 0 ||
			group.compare("SIFT") == 0 ||
			group.compare("ORB") == 0 ||
			group.compare("FAST") == 0 ||
			group.compare("FREAK") == 0 ||
			group.compare("BRIEF") == 0 ||
			group.compare("GFTT") == 0 ||
			group.compare("BRISK") == 0)
		{
			parameters_.insert(*iter);
		}
	}

	for(rtabmap::ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
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

	// Backward compatibility
	std::list<std::string> oldParameterNames;
	oldParameterNames.push_back("Odom/Type");
	oldParameterNames.push_back("Odom/MaxWords");
	oldParameterNames.push_back("Odom/WordsRatio");
	oldParameterNames.push_back("Odom/LocalHistory");
	oldParameterNames.push_back("Odom/NearestNeighbor");
	oldParameterNames.push_back("Odom/NNDR");
	for(std::list<std::string>::iterator iter=oldParameterNames.begin(); iter!=oldParameterNames.end(); ++iter)
	{
		std::string vStr;
		if(pnh.getParam(*iter, vStr))
		{
			if(iter->compare("Odom/Type") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/Type -> %s. Please update your launch file accordingly.",
						Parameters::kOdomFeatureType().c_str());
				parameters_.at(Parameters::kOdomFeatureType())= vStr;
			}
			else if(iter->compare("Odom/MaxWords") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/MaxWords -> %s. Please update your launch file accordingly.",
						Parameters::kOdomMaxFeatures().c_str());
				parameters_.at(Parameters::kOdomMaxFeatures())= vStr;
			}
			else if(iter->compare("Odom/LocalHistory") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/LocalHistory -> %s. Please update your launch file accordingly.",
						Parameters::kOdomBowLocalHistorySize().c_str());
				parameters_.at(Parameters::kOdomBowLocalHistorySize())= vStr;
			}
			else if(iter->compare("Odom/NearestNeighbor") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/NearestNeighbor -> %s. Please update your launch file accordingly.",
						Parameters::kOdomBowNNType().c_str());
				parameters_.at(Parameters::kOdomBowNNType())= vStr;
			}
			else if(iter->compare("Odom/WordsRatio") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/WordsRatio -> %s. Please update your launch file accordingly.",
						Parameters::kOdomFeaturesRatio().c_str());
				parameters_.at(Parameters::kOdomFeaturesRatio())= vStr;
			}
			else if(iter->compare("Odom/NNDR") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/NNDR -> %s. Please update your launch file accordingly.",
						Parameters::kOdomBowNNDR().c_str());
				parameters_.at(Parameters::kOdomBowNNDR())= vStr;
			}
		}
	}

	int odomStrategy = 0; // BOW
	Parameters::parse(parameters_, Parameters::kOdomStrategy(), odomStrategy);
	if(odomStrategy == 1)
	{
		ROS_INFO("Using OdometryOpticalFlow");
		odometry_ = new rtabmap::OdometryOpticalFlow(parameters_);
	}
	else
	{
		ROS_INFO("Using OdometryBOW");
		odometry_ = new rtabmap::OdometryBOW(parameters_);
	}
	if(!initialPose.isIdentity())
	{
		odometry_->reset(initialPose);
	}

	resetSrv_ = nh.advertiseService("reset_odom", &OdometryROS::reset, this);
	resetToPoseSrv_ = nh.advertiseService("reset_odom_to_pose", &OdometryROS::resetToPose, this);
	pauseSrv_ = nh.advertiseService("pause_odom", &OdometryROS::pause, this);
	resumeSrv_ = nh.advertiseService("resume_odom", &OdometryROS::resume, this);
}

OdometryROS::~OdometryROS()
{
	ros::NodeHandle pnh("~");
	for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		pnh.deleteParam(iter->first);
	}

	delete odometry_;
}

void OdometryROS::processArguments(int argc, char * argv[])
{
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
					if(iter->first.find("Odom") == 0 ||
						uSplit(iter->first, '/').front().compare("Stereo") == 0 ||
						uSplit(iter->first, '/').front().compare("SURF") == 0 ||
						uSplit(iter->first, '/').front().compare("SIFT") == 0 ||
						uSplit(iter->first, '/').front().compare("ORB") == 0 ||
						uSplit(iter->first, '/').front().compare("FAST") == 0 ||
						uSplit(iter->first, '/').front().compare("FREAK") == 0 ||
						uSplit(iter->first, '/').front().compare("BRIEF") == 0 ||
						uSplit(iter->first, '/').front().compare("GFTT") == 0 ||
						uSplit(iter->first, '/').front().compare("BRISK") == 0)
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
}

Transform OdometryROS::processData(SensorData & data, const std_msgs::Header & header, int & quality)
{
	rtabmap::Transform pose = odometry_->process(data, &quality);
	if(!pose.isNull())
	{
		//*********************
		// Update odometry
		//*********************
		tf::Transform poseTF;
		rtabmap::transformToTF(pose, poseTF);

		if(publishTf_)
		{
			tfBroadcaster_.sendTransform( tf::StampedTransform (poseTF, header.stamp, odomFrameId_, frameId_));
		}

		if(odomPub_.getNumSubscribers())
		{
			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = header.stamp; // use corresponding time stamp to image
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

		if(odomLocalMap_.getNumSubscribers() && dynamic_cast<OdometryBOW*>(odometry_))
		{
			const std::multimap<int, pcl::PointXYZ> & map = ((OdometryBOW*)odometry_)->getLocalMap();
			pcl::PointCloud<pcl::PointXYZ> cloud;
			for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=map.begin(); iter!=map.end(); ++iter)
			{
				cloud.push_back(iter->second);
			}
			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
			cloudMsg.header.frame_id = odomFrameId_;
			odomLocalMap_.publish(cloudMsg);
		}

		if(odomLastFrame_.getNumSubscribers())
		{
			if(dynamic_cast<OdometryBOW*>(odometry_))
			{
				const rtabmap::Signature * s  = ((OdometryBOW*)odometry_)->getMemory()->getLastWorkingSignature();
				if(s)
				{
					const std::multimap<int, pcl::PointXYZ> & words3 = s->getWords3();
					pcl::PointCloud<pcl::PointXYZ> cloud;
					rtabmap::Transform t = data.localTransform();
					for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=words3.begin(); iter!=words3.end(); ++iter)
					{
						// transform to odom frame
						pcl::PointXYZ pt = util3d::transformPoint(iter->second, pose);
						cloud.push_back(pt);
					}

					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_.publish(cloudMsg);
				}
			}
			else
			{
				//Optical flow
				const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud = ((OdometryOpticalFlow*)odometry_)->getLastCorners3D();
				if(cloud->size())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTransformed;
					cloudTransformed = util3d::transformPointCloud<pcl::PointXYZ>(cloud, pose);
					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(*cloudTransformed, cloudMsg);
					cloudMsg.header.stamp = header.stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_.publish(cloudMsg);
				}
			}
		}
	}
	else
	{
		//ROS_WARN("Odometry lost!");

		//send null pose to notify that odometry is lost
		nav_msgs::Odometry odom;
		odom.header.stamp = header.stamp; // use corresponding time stamp to image
		odom.header.frame_id = odomFrameId_;
		odom.child_frame_id = frameId_;

		//publish the message
		odomPub_.publish(odom);
	}

	return pose;
}

bool OdometryROS::isOdometryBOW() const
{
	return dynamic_cast<OdometryBOW*>(odometry_) != 0;
}

bool OdometryROS::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	ROS_INFO("visual_odometry: reset odom!");
	odometry_->reset();
	return true;
}

bool OdometryROS::resetToPose(rtabmap_ros::ResetPose::Request& req, rtabmap_ros::ResetPose::Response&)
{
	Transform pose(req.x, req.y, req.z, req.roll, req.pitch, req.yaw);
	ROS_INFO("visual_odometry: reset odom to pose %s!", pose.prettyPrint().c_str());
	odometry_->reset(pose);
	return true;
}

bool OdometryROS::pause(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
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

bool OdometryROS::resume(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
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

}
