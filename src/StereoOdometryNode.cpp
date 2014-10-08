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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <image_geometry/stereo_camera_model.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include "rtabmap/MsgConversion.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UTimer.h"

using namespace rtabmap;

class StereoOdometry
{
public:
	StereoOdometry() :
		odometry_(0),
		feature2D_(0),
		frameId_("base_link"),
		odomFrameId_("odom"),
		publishTf_(true),
		minDisparity_(0.0),
		maxDisparity_(128.0),
		k_(10),
		winSize_(5),
		sync_(0),
		paused_(false)
	{
		ros::NodeHandle nh;

		odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
		odomLocalMapPub_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_map", 1);
		odomLastFrame_ = nh.advertise<sensor_msgs::PointCloud2>("odom_last_frame", 1);
		odomDepth_ = nh.advertise<sensor_msgs::Image>("odom_depth", 1);

		ros::NodeHandle pnh("~");

		int queueSize = 5;
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("publish_tf", publishTf_, publishTf_);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("min_disparity", minDisparity_, minDisparity_);
		pnh.param("max_disparity", maxDisparity_, maxDisparity_);
		pnh.param("k", k_, k_);
		pnh.param("window_size", winSize_, winSize_);

		//parameters
		rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap parametersOdom;
		for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			std::string group = uSplit(iter->first, '/').front();
			if(group.compare("Odom") == 0 ||
				group.compare("SURF") == 0 ||
				group.compare("SIFT") == 0 ||
				group.compare("ORB") == 0 ||
				group.compare("FAST") == 0 ||
				group.compare("FREAK") == 0 ||
				group.compare("BRIEF") == 0 ||
				group.compare("GFTT") == 0 ||
				group.compare("BRISK") == 0)
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

		//Keypoint detector
		ParametersMap::const_iterator iter;
		Feature2D::Type detectorStrategy = Feature2D::kFeatureUndef;
		if((iter=parametersOdom.find(Parameters::kOdomType())) != parametersOdom.end())
		{
			detectorStrategy = (Feature2D::Type)std::atoi((*iter).second.c_str());
		}
		switch(detectorStrategy)
		{
		case Feature2D::kFeatureSift:
			feature2D_ = new SIFT(parametersOdom);
			break;
		case Feature2D::kFeatureFastBrief:
			feature2D_ = new FAST_BRIEF(parametersOdom);
			break;
		case Feature2D::kFeatureFastFreak:
			feature2D_ = new FAST_FREAK(parametersOdom);
			break;
		case Feature2D::kFeatureOrb:
			feature2D_ = new ORB(parametersOdom);
			break;
		case Feature2D::kFeatureGfttFreak:
			feature2D_ = new GFTT_FREAK(parametersOdom);
			break;
		case Feature2D::kFeatureGfttBrief:
			feature2D_ = new GFTT_BRIEF(parametersOdom);
			break;
		case Feature2D::kFeatureBrisk:
			feature2D_ = new BRISK(parametersOdom);
			break;
		case Feature2D::kFeatureSurf:
		default:
			feature2D_ = new SURF(parametersOdom);
			break;
		}

		ros::NodeHandle left_nh(nh, "left");
		ros::NodeHandle right_nh(nh, "right");
		ros::NodeHandle left_pnh(pnh, "left");
		ros::NodeHandle right_pnh(pnh, "right");
		image_transport::ImageTransport left_it(left_nh);
		image_transport::ImageTransport right_it(right_nh);
		image_transport::TransportHints hintsLeft("raw", ros::TransportHints(), left_pnh);
		image_transport::TransportHints hintsRight("raw", ros::TransportHints(), right_pnh);

		imageRectLeft_.subscribe(left_it, left_nh.resolveName("image_rect"), 1, hintsLeft);
		imageRectRight_.subscribe(right_it, right_nh.resolveName("image_rect"), 1, hintsRight);
		cameraInfoLeft_.subscribe(left_nh, "camera_info", 1);
		cameraInfoRight_.subscribe(right_nh, "camera_info", 1);

		sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize), imageRectLeft_, imageRectRight_, cameraInfoLeft_, cameraInfoRight_);
		sync_->registerCallback(boost::bind(&StereoOdometry::callback, this, _1, _2, _3, _4));

		resetSrv_ = nh.advertiseService("reset_odom", &StereoOdometry::reset, this);
		pauseSrv_ = nh.advertiseService("pause_odom", &StereoOdometry::pause, this);
		resumeSrv_ = nh.advertiseService("resume_odom", &StereoOdometry::resume, this);
	}

	~StereoOdometry()
	{
		ros::NodeHandle pnh("~");
		ParametersMap parameters = Parameters::getDefaultParameters();
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			pnh.deleteParam(iter->first);
		}

		delete sync_;
		delete odometry_;
		delete feature2D_;
	}

	void callback(
			const sensor_msgs::ImageConstPtr& imageRectLeft,
			const sensor_msgs::ImageConstPtr& imageRectRight,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoLeft,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoRight)
	{
		if(!paused_)
		{
			if(!(imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 imageRectLeft->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
				!(imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				  imageRectRight->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
			{
				ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended)");
				return;
			}

			tf::StampedTransform localTransform;
			try
			{
				tfListener_.lookupTransform(frameId_, imageRectLeft->header.frame_id, imageRectLeft->header.stamp, localTransform);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			ros::WallTime time = ros::WallTime::now();

			int quality = -1;
			if(imageRectLeft->data.size() && imageRectRight->data.size())
			{
				float depthFx = cameraInfoLeft->K[0];
				float depthFy = cameraInfoLeft->K[4];
				float depthCx = cameraInfoLeft->K[2];
				float depthCy = cameraInfoLeft->K[5];
				cv_bridge::CvImageConstPtr ptrImageLeft = cv_bridge::toCvShare(imageRectLeft, "mono8");
				cv_bridge::CvImageConstPtr ptrImageRight = cv_bridge::toCvShare(imageRectRight, "mono8");

				//generate depth
				cv::Mat depth = cv::Mat::zeros(ptrImageLeft->image.rows, ptrImageLeft->image.cols, CV_32FC1);

				std::vector<cv::KeyPoint> kptsLeft, kptsRight;
				std::vector<cv::Point2f> cornersLeft, cornersRight;
				cv::Mat descLeft, descRight;
				kptsLeft = feature2D_->generateKeypoints(ptrImageLeft->image);
				if(kptsLeft.size())
				{
					descLeft = feature2D_->generateDescriptors(ptrImageLeft->image, kptsLeft);

					kptsRight = feature2D_->generateKeypoints(ptrImageRight->image);
					if(kptsRight.size())
					{
						descRight = feature2D_->generateDescriptors(ptrImageRight->image, kptsRight);

						if(kptsLeft.size() && kptsRight.size())
						{
							cornersLeft.resize(kptsLeft.size());
							cornersRight.resize(kptsRight.size());
							for(unsigned int i=0; i<kptsLeft.size() || i<kptsRight.size(); ++i)
							{
								if(i<kptsLeft.size())
								{
									cornersLeft[i] = kptsLeft[i].pt;
								}
								if(i<kptsRight.size())
								{
									cornersRight[i] = kptsRight[i].pt;
								}
							}
							UTimer time;
							cv::cornerSubPix( ptrImageLeft->image, cornersLeft, cv::Size( winSize_, winSize_ ), cv::Size( -1, -1 ),
										  cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

							cv::cornerSubPix( ptrImageRight->image, cornersRight, cv::Size( winSize_, winSize_ ), cv::Size( -1, -1 ),
										  cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );
							UDEBUG("time subpix = %fs", time.ticks());
						}
					}
				}


				if(kptsLeft.size() >= odometry_->getMinInliers() && kptsRight.size() >= odometry_->getMinInliers())
				{
					std::vector<std::vector<cv::DMatch> > matches;
					cv::BFMatcher matcher(descLeft.depth()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2);
					int k = std::min((int)kptsLeft.size(), k_);
					k = std::min((int)kptsRight.size(), k);
					matcher.knnMatch(descLeft, descRight, matches, k);
					int added = 0;

					if(matches.size())
					{
						image_geometry::StereoCameraModel model;
						model.fromCameraInfo(*cameraInfoLeft, *cameraInfoRight);

						int addedFirst = 0;
						for(int i=0; i< matches.size(); ++i)
						{
							// add only those on same Y
							for(unsigned int j=0; j<matches[i].size(); ++j)
							{
								float disparity = cornersLeft[matches[i].at(j).queryIdx].x - cornersRight[matches[i].at(j).trainIdx].x;

								if((int)disparity >= minDisparity_ && (int)disparity <= maxDisparity_)
								{
									float d = model.getZ(disparity);
									if(	d>0 &&
										cornersLeft[matches[i].at(j).queryIdx].y >= cornersRight[matches[i].at(j).trainIdx].y - 3.0f &&
										cornersLeft[matches[i].at(j).queryIdx].y <= cornersRight[matches[i].at(j).trainIdx].y + 3.0f)
									{
										kptsLeft[matches[i].at(j).queryIdx].pt = cornersLeft[matches[i].at(j).queryIdx];
										depth.at<float>(int(kptsLeft[matches[i].at(j).queryIdx].pt.y+0.5f), int(kptsLeft[matches[i].at(j).queryIdx].pt.x+0.5f)) = d;
										/*ROS_INFO("Add%d Left(%d, %d) Right(%d, %d) distance %d = %f disp=%f, depth=%f",
												j,
												int(kptsLeft[matches[i].at(j).queryIdx].pt.x+0.5f),
												int(kptsLeft[matches[i].at(j).queryIdx].pt.y+0.5f),
												int(kptsRight[matches[i].at(j).trainIdx].pt.x+0.5f),
												int(kptsRight[matches[i].at(j).trainIdx].pt.y+0.5f),
												i, matches[i].at(j).distance, disparity, d);*/
										if(j == 0)
										{
											++addedFirst;
										}
										++added;
										break;
									}
									else
									{
										/*ROS_INFO("--- Left(%d, %d) Right(%d, %d) distance %d = %f disp=%f depth=%f",
												int(kptsLeft[matches[i].queryIdx].pt.x+0.5f),
												int(kptsLeft[matches[i].queryIdx].pt.y+0.5f),
												int(kptsRight[matches[i].trainIdx].pt.x+0.5f),
												int(kptsRight[matches[i].trainIdx].pt.y+0.5f),
												i, matches[i].distance, disparity, d);*/
									}
								}
							}
						}
						UDEBUG("addedFirst = %d/%d", addedFirst, added);

						//
						UDEBUG("localTransform = %s", rtabmap::transformFromTF(localTransform).prettyPrint().c_str());
						rtabmap::SensorData data(ptrImageLeft->image,
								depth,
								depthFx,
								depthFy,
								depthCx,
								depthCy,
								rtabmap::Transform(),
								rtabmap::transformFromTF(localTransform));
						data.setFeatures(kptsLeft, descLeft);
						quality=0;
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
								tfBroadcaster_.sendTransform( tf::StampedTransform (poseTF, imageRectLeft->header.stamp, odomFrameId_, frameId_));
							}

							if(odomPub_.getNumSubscribers())
							{
								//next, we'll publish the odometry message over ROS
								nav_msgs::Odometry odom;
								odom.header.stamp = imageRectLeft->header.stamp; // use corresponding time stamp to image
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

							if(odomLocalMapPub_.getNumSubscribers())
							{
								const std::multimap<int, pcl::PointXYZ> & map = odometry_->getLocalMap();
								pcl::PointCloud<pcl::PointXYZ> cloud;
								for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=map.begin(); iter!=map.end(); ++iter)
								{
									cloud.push_back(iter->second);
								}
								sensor_msgs::PointCloud2 cloudMsg;
								pcl::toROSMsg(cloud, cloudMsg);
								cloudMsg.header.stamp = imageRectLeft->header.stamp; // use corresponding time stamp to image
								cloudMsg.header.frame_id = odomFrameId_;
								odomLocalMapPub_.publish(cloudMsg);
							}

							if(odomLastFrame_.getNumSubscribers())
							{
								const rtabmap::Signature * s  = odometry_->getMemory()->getLastWorkingSignature();
								if(s)
								{
									const std::multimap<int, pcl::PointXYZ> & words3 = s->getWords3();
									pcl::PointCloud<pcl::PointXYZ> cloud;
									rtabmap::Transform t = rtabmap::transformFromTF(localTransform);
									for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=words3.begin(); iter!=words3.end(); ++iter)
									{
										// transform to odom frame
										pcl::PointXYZ pt = util3d::transformPoint(iter->second, pose);
										cloud.push_back(pt);
									}

									sensor_msgs::PointCloud2 cloudMsg;
									pcl::toROSMsg(cloud, cloudMsg);
									cloudMsg.header.stamp = imageRectLeft->header.stamp; // use corresponding time stamp to image
									cloudMsg.header.frame_id = odomFrameId_;
									odomLastFrame_.publish(cloudMsg);
								}
							}
							if(odomDepth_.getNumSubscribers())
							{
								cv_bridge::CvImage img;
								img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
								img.image = depth;
								sensor_msgs::ImagePtr rosMsg = img.toImageMsg();
								rosMsg->header= imageRectLeft->header;
								odomDepth_.publish(rosMsg);
							}
						}
						else
						{
							//ROS_WARN("Odometry lost!");

							//send null pose to notify that odometry is lost
							nav_msgs::Odometry odom;
							odom.header.stamp = imageRectLeft->header.stamp; // use corresponding time stamp to image
							odom.header.frame_id = odomFrameId_;
							odom.child_frame_id = frameId_;

							//publish the message
							odomPub_.publish(odom);
						}
					}
					ROS_INFO("Odom: quality=%d, update time=%fs, stereo matches: added %d/%d",
						quality, (ros::WallTime::now()-time).toSec(),
						added, (int)matches.size());
				}
				else
				{
					ROS_WARN("Odom: no keypoints extracted!");
				}

			}
			else
			{
				ROS_WARN("Odom: input images empty?!?");
			}
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
	rtabmap::OdometryBOW * odometry_;
	rtabmap::Feature2D * feature2D_;

	// parameters
	std::string frameId_;
	std::string odomFrameId_;
	bool publishTf_;
	int minDisparity_;
	int maxDisparity_;
	int k_;
	int winSize_;

	ros::Publisher odomPub_;
	ros::Publisher odomLocalMapPub_;
	ros::Publisher odomLastFrame_;
	ros::Publisher odomDepth_;
	ros::ServiceServer resetSrv_;
	ros::ServiceServer pauseSrv_;
	ros::ServiceServer resumeSrv_;
	tf::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;

	bool paused_;
};

int main(int argc, char *argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	//pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
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
					if(uSplit(iter->first, '/').front().compare("Odom") == 0 ||
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

	StereoOdometry vOdom;
	ros::spin();
	return 0;
}
