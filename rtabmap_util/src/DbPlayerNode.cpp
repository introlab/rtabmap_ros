/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/SetGoal.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/OdometryEvent.h>
#include <cmath>

#ifndef _WIN32
#include <sys/ioctl.h>
#include <termios.h>
bool spacehit()
{
	bool charAvailable = true;
	bool hit = false;
	while(charAvailable)
	{
		termios term;
		tcgetattr(0, &term);

		termios term2 = term;
		term2.c_lflag &= ~ICANON;
		term2.c_lflag &= ~ECHO;
		term2.c_lflag &= ~ISIG;
		term2.c_cc[VMIN] = 0;
		term2.c_cc[VTIME] = 0;
		tcsetattr(0, TCSANOW, &term2);

		int c = getchar();
		if(c != EOF)
		{
			if(c == ' ')
			{
				hit = true;
			}
		}
		else
		{
			charAvailable = false;
		}

		tcsetattr(0, TCSANOW, &term);
	}

    return hit;
}
#endif

bool paused = false;
bool pauseCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(paused)
	{
		ROS_WARN("Already paused!");
	}
	else
	{
		paused = true;
		ROS_INFO("paused!");
	}
	return true;
}

bool resumeCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(!paused)
	{
		ROS_WARN("Already running!");
	}
	else
	{
		paused = false;
		ROS_INFO("resumed!");
	}
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_player");

	//ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	//ULogger::setEventLevel(ULogger::kWarning);

	bool publishClock = false;
	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--clock") == 0)
		{
			publishClock = true;
		}
	}

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string frameId = "base_link";
	std::string odomFrameId = "odom";
	std::string cameraFrameId = "camera_optical_link";
	std::string scanFrameId = "base_laser_link";
	double rate = 1.0f;
	std::string databasePath = "";
	bool publishTf = true;
	int startId = 0;
	bool useDbStamps = true;

	pnh.param("frame_id", frameId, frameId);
	pnh.param("odom_frame_id", odomFrameId, odomFrameId);
	pnh.param("camera_frame_id", cameraFrameId, cameraFrameId);
	pnh.param("scan_frame_id", scanFrameId, scanFrameId);
	pnh.param("rate", rate, rate); // Ratio of the database stamps
	pnh.param("database", databasePath, databasePath);
	pnh.param("publish_tf", publishTf, publishTf);
	pnh.param("start_id", startId, startId);

	// A general 360 lidar with 0.5 deg increment
	double scanAngleMin, scanAngleMax, scanAngleIncrement, scanRangeMin, scanRangeMax;
	pnh.param<double>("scan_angle_min", scanAngleMin, -M_PI);
	pnh.param<double>("scan_angle_max", scanAngleMax, M_PI);
	pnh.param<double>("scan_angle_increment", scanAngleIncrement, M_PI / 720.0);
	pnh.param<double>("scan_range_min", scanRangeMin, 0.0);
	pnh.param<double>("scan_range_max", scanRangeMax, 60);

	ROS_INFO("frame_id = %s", frameId.c_str());
	ROS_INFO("odom_frame_id = %s", odomFrameId.c_str());
	ROS_INFO("camera_frame_id = %s", cameraFrameId.c_str());
	ROS_INFO("scan_frame_id = %s", scanFrameId.c_str());
	ROS_INFO("rate = %f", rate);
	ROS_INFO("publish_tf = %s", publishTf?"true":"false");
	ROS_INFO("start_id = %d", startId);
	ROS_INFO("Publish clock (--clock): %s", publishClock?"true":"false");

	if(databasePath.empty())
	{
		ROS_ERROR("Parameter \"database\" must be set (path to a RTAB-Map database).");
		return -1;
	}
	databasePath = uReplaceChar(databasePath, '~', UDirectory::homeDir());
	if(databasePath.size() && databasePath.at(0) != '/')
	{
		databasePath = UDirectory::currentDir(true) + databasePath;
	}
	ROS_INFO("database = %s", databasePath.c_str());

	rtabmap::DBReader reader(databasePath, -rate, false, false, false, startId);
	if(!reader.init())
	{
		ROS_ERROR("Cannot open database \"%s\".", databasePath.c_str());
		return -1;
	}

	ros::ServiceServer pauseSrv = pnh.advertiseService("pause", pauseCallback);
	ros::ServiceServer resumeSrv = pnh.advertiseService("resume", resumeCallback);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher imagePub;
	image_transport::Publisher rgbPub;
	image_transport::Publisher depthPub;
	image_transport::Publisher leftPub;
	image_transport::Publisher rightPub;
	ros::Publisher rgbCamInfoPub;
	ros::Publisher depthCamInfoPub;
	ros::Publisher leftCamInfoPub;
	ros::Publisher rightCamInfoPub;
	ros::Publisher odometryPub;
	ros::Publisher scanPub;
	ros::Publisher scanCloudPub;
	ros::Publisher globalPosePub;
	ros::Publisher gpsFixPub;
	ros::Publisher clockPub;
	tf2_ros::TransformBroadcaster tfBroadcaster;

	if(publishClock)
	{
		clockPub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
	}

	UTimer timer;
	rtabmap::CameraInfo cameraInfo;
	rtabmap::SensorData data = reader.takeImage(&cameraInfo);
	rtabmap::OdometryInfo odomInfo;
	odomInfo.reg.covariance = cameraInfo.odomCovariance;
	rtabmap::OdometryEvent odom(data, cameraInfo.odomPose, odomInfo);
	double acquisitionTime = timer.ticks();
	while(ros::ok() && odom.data().id())
	{
		ROS_INFO("Reading sensor data %d...", odom.data().id());

		ros::Time time(odom.data().stamp());

		if(publishClock)
		{
			rosgraph_msgs::Clock msg;
			msg.clock = time;
			clockPub.publish(msg);
		}

		sensor_msgs::CameraInfo camInfoA; //rgb or left
		sensor_msgs::CameraInfo camInfoB; //depth or right

		camInfoA.K.assign(0);
		camInfoA.K[0] = camInfoA.K[4] = camInfoA.K[8] = 1;
		camInfoA.R.assign(0);
		camInfoA.R[0] = camInfoA.R[4] = camInfoA.R[8] = 1;
		camInfoA.P.assign(0);
		camInfoA.P[10] = 1;

		camInfoA.header.frame_id = cameraFrameId;
		camInfoA.header.stamp = time;

		camInfoB = camInfoA;

		int type = -1;
		if(!odom.data().depthRaw().empty() && (odom.data().depthRaw().type() == CV_32FC1 || odom.data().depthRaw().type() == CV_16UC1))
		{
			if(odom.data().cameraModels().size() > 1)
			{
				ROS_WARN("Multi-cameras detected in database but this node cannot send multi-images yet...");
			}
			else
			{
				//depth
				if(odom.data().cameraModels().size())
				{
					camInfoA.D.resize(5,0);

					camInfoA.P[0] = odom.data().cameraModels()[0].fx();
					camInfoA.K[0] = odom.data().cameraModels()[0].fx();
					camInfoA.P[5] = odom.data().cameraModels()[0].fy();
					camInfoA.K[4] = odom.data().cameraModels()[0].fy();
					camInfoA.P[2] = odom.data().cameraModels()[0].cx();
					camInfoA.K[2] = odom.data().cameraModels()[0].cx();
					camInfoA.P[6] = odom.data().cameraModels()[0].cy();
					camInfoA.K[5] = odom.data().cameraModels()[0].cy();

					camInfoB = camInfoA;
				}

				type=0;

				if(rgbPub.getTopic().empty()) rgbPub = it.advertise("rgb/image", 1);
				if(depthPub.getTopic().empty()) depthPub = it.advertise("depth_registered/image", 1);
				if(rgbCamInfoPub.getTopic().empty()) rgbCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);
				if(depthCamInfoPub.getTopic().empty()) depthCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("depth_registered/camera_info", 1);
			}
		}
		else if(!odom.data().rightRaw().empty() && odom.data().rightRaw().type() == CV_8U)
		{
			if(odom.data().stereoCameraModels().size() > 1)
			{
				ROS_WARN("Multi-cameras detected in database but this node cannot send multi-images yet...");
			}
			else
			{
				//stereo
				if(odom.data().stereoCameraModels()[0].isValidForProjection())
				{
					camInfoA.D.resize(8,0);

					camInfoA.P[0] = odom.data().stereoCameraModels()[0].left().fx();
					camInfoA.K[0] = odom.data().stereoCameraModels()[0].left().fx();
					camInfoA.P[5] = odom.data().stereoCameraModels()[0].left().fy();
					camInfoA.K[4] = odom.data().stereoCameraModels()[0].left().fy();
					camInfoA.P[2] = odom.data().stereoCameraModels()[0].left().cx();
					camInfoA.K[2] = odom.data().stereoCameraModels()[0].left().cx();
					camInfoA.P[6] = odom.data().stereoCameraModels()[0].left().cy();
					camInfoA.K[5] = odom.data().stereoCameraModels()[0].left().cy();

					camInfoB = camInfoA;
					camInfoB.P[3] = odom.data().stereoCameraModels()[0].right().Tx(); // Right_Tx = -baseline*fx
				}

				type=1;

				if(leftPub.getTopic().empty()) leftPub = it.advertise("left/image", 1);
				if(rightPub.getTopic().empty()) rightPub = it.advertise("right/image", 1);
				if(leftCamInfoPub.getTopic().empty()) leftCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
				if(rightCamInfoPub.getTopic().empty()) rightCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
			}

		}
		else
		{
			if(imagePub.getTopic().empty()) imagePub = it.advertise("image", 1);
		}

		camInfoA.height = odom.data().imageRaw().rows;
		camInfoA.width = odom.data().imageRaw().cols;
		camInfoB.height = odom.data().depthOrRightRaw().rows;
		camInfoB.width = odom.data().depthOrRightRaw().cols;

		if(!odom.data().laserScanRaw().isEmpty())
		{
			if(scanPub.getTopic().empty() && odom.data().laserScanRaw().is2d())
			{
				scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
				if(odom.data().laserScanRaw().angleIncrement() > 0.0f)
				{
					ROS_INFO("Scan will be published.");
				}
				else
				{
					ROS_INFO("Scan will be published with those parameters:");
					ROS_INFO("  scan_angle_min=%f", scanAngleMin);
					ROS_INFO("  scan_angle_max=%f", scanAngleMax);
					ROS_INFO("  scan_angle_increment=%f", scanAngleIncrement);
					ROS_INFO("  scan_range_min=%f", scanRangeMin);
					ROS_INFO("  scan_range_max=%f", scanRangeMax);
				}
			}
			else if(scanCloudPub.getTopic().empty())
			{
				scanCloudPub = nh.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1);
				ROS_INFO("Scan cloud will be published.");
			}
		}

		if(!odom.data().globalPose().isNull() &&
			odom.data().globalPoseCovariance().cols==6 &&
			odom.data().globalPoseCovariance().rows==6)
		{
			if(globalPosePub.getTopic().empty())
			{
				globalPosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("global_pose", 1);
				ROS_INFO("Global pose will be published.");
			}
		}

		if(odom.data().gps().stamp() > 0.0)
		{
			if(gpsFixPub.getTopic().empty())
			{
				gpsFixPub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
				ROS_INFO("GPS will be published.");
			}
		}

		// publish transforms first
		if(publishTf)
		{
			rtabmap::Transform localTransform;
			if(odom.data().cameraModels().size() == 1)
			{
				localTransform = odom.data().cameraModels()[0].localTransform();
			}
			else if(odom.data().stereoCameraModels().size() == 1)
			{
				localTransform = odom.data().stereoCameraModels()[0].left().localTransform();
			}
			if(!localTransform.isNull())
			{
				geometry_msgs::TransformStamped baseToCamera;
				baseToCamera.child_frame_id = cameraFrameId;
				baseToCamera.header.frame_id = frameId;
				baseToCamera.header.stamp = time;
				rtabmap_conversions::transformToGeometryMsg(localTransform, baseToCamera.transform);
				tfBroadcaster.sendTransform(baseToCamera);
			}

			if(!odom.pose().isNull())
			{
				geometry_msgs::TransformStamped odomToBase;
				odomToBase.child_frame_id = frameId;
				odomToBase.header.frame_id = odomFrameId;
				odomToBase.header.stamp = time;
				rtabmap_conversions::transformToGeometryMsg(odom.pose(), odomToBase.transform);
				tfBroadcaster.sendTransform(odomToBase);
			}

			if(!scanPub.getTopic().empty() || !scanCloudPub.getTopic().empty())
			{
				geometry_msgs::TransformStamped baseToLaserScan;
				baseToLaserScan.child_frame_id = scanFrameId;
				baseToLaserScan.header.frame_id = frameId;
				baseToLaserScan.header.stamp = time;
				rtabmap_conversions::transformToGeometryMsg(odom.data().laserScanCompressed().localTransform(), baseToLaserScan.transform);
				tfBroadcaster.sendTransform(baseToLaserScan);
			}
		}
		if(!odom.pose().isNull())
		{
			if(odometryPub.getTopic().empty()) odometryPub = nh.advertise<nav_msgs::Odometry>("odom", 1);

			if(odometryPub.getNumSubscribers())
			{
				nav_msgs::Odometry odomMsg;
				odomMsg.child_frame_id = frameId;
				odomMsg.header.frame_id = odomFrameId;
				odomMsg.header.stamp = time;
				rtabmap_conversions::transformToPoseMsg(odom.pose(), odomMsg.pose.pose);
				UASSERT(odomMsg.pose.covariance.size() == 36 &&
						odom.covariance().total() == 36 &&
						odom.covariance().type() == CV_64FC1);
				memcpy(odomMsg.pose.covariance.begin(), odom.covariance().data, 36*sizeof(double));
				odometryPub.publish(odomMsg);
			}
		}

		// Publish async topics first (so that they can catched by rtabmap before the image topics)
		if(globalPosePub.getNumSubscribers() > 0 &&
			!odom.data().globalPose().isNull() &&
			odom.data().globalPoseCovariance().cols==6 &&
			odom.data().globalPoseCovariance().rows==6)
		{
			geometry_msgs::PoseWithCovarianceStamped msg;
			rtabmap_conversions::transformToPoseMsg(odom.data().globalPose(), msg.pose.pose);
			memcpy(msg.pose.covariance.data(), odom.data().globalPoseCovariance().data, 36*sizeof(double));
			msg.header.frame_id = frameId;
			msg.header.stamp = time;
			globalPosePub.publish(msg);
		}

		if(odom.data().gps().stamp() > 0.0)
		{
			sensor_msgs::NavSatFix msg;
			msg.longitude = odom.data().gps().longitude();
			msg.latitude = odom.data().gps().latitude();
			msg.altitude = odom.data().gps().altitude();
			msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
			msg.position_covariance.at(0) = msg.position_covariance.at(4) = msg.position_covariance.at(8)= odom.data().gps().error()* odom.data().gps().error();
			msg.header.frame_id = frameId;
			msg.header.stamp.fromSec(odom.data().gps().stamp());
			gpsFixPub.publish(msg);
		}

		if(type >= 0)
		{
			if(rgbCamInfoPub.getNumSubscribers() && type == 0)
			{
				rgbCamInfoPub.publish(camInfoA);
			}
			if(leftCamInfoPub.getNumSubscribers() && type == 1)
			{
				leftCamInfoPub.publish(camInfoA);
			}
			if(depthCamInfoPub.getNumSubscribers() && type == 0)
			{
				depthCamInfoPub.publish(camInfoB);
			}
			if(rightCamInfoPub.getNumSubscribers() && type == 1)
			{
				rightCamInfoPub.publish(camInfoB);
			}
		}

		if(imagePub.getNumSubscribers() || rgbPub.getNumSubscribers() || leftPub.getNumSubscribers())
		{
			cv_bridge::CvImage img;
			if(odom.data().imageRaw().channels() == 1)
			{
				img.encoding = sensor_msgs::image_encodings::MONO8;
			}
			else
			{
				img.encoding = sensor_msgs::image_encodings::BGR8;
			}
			img.image = odom.data().imageRaw();
			sensor_msgs::ImagePtr imageRosMsg = img.toImageMsg();
			imageRosMsg->header.frame_id = cameraFrameId;
			imageRosMsg->header.stamp = time;

			if(imagePub.getNumSubscribers())
			{
				imagePub.publish(imageRosMsg);
			}
			if(rgbPub.getNumSubscribers() && type == 0)
			{
				rgbPub.publish(imageRosMsg);
			}
			if(leftPub.getNumSubscribers() && type == 1)
			{
				leftPub.publish(imageRosMsg);
				leftCamInfoPub.publish(camInfoA);
			}
		}

		if(depthPub.getNumSubscribers() && !odom.data().depthRaw().empty() && type==0)
		{
			cv_bridge::CvImage img;
			if(odom.data().depthRaw().type() == CV_32FC1)
			{
				img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			}
			else
			{
				img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			}
			img.image = odom.data().depthRaw();
			sensor_msgs::ImagePtr imageRosMsg = img.toImageMsg();
			imageRosMsg->header.frame_id = cameraFrameId;
			imageRosMsg->header.stamp = time;

			depthPub.publish(imageRosMsg);
			depthCamInfoPub.publish(camInfoB);
		}

		if(rightPub.getNumSubscribers() && !odom.data().rightRaw().empty() && type==1)
		{
			cv_bridge::CvImage img;
			img.encoding = sensor_msgs::image_encodings::MONO8;
			img.image = odom.data().rightRaw();
			sensor_msgs::ImagePtr imageRosMsg = img.toImageMsg();
			imageRosMsg->header.frame_id = cameraFrameId;
			imageRosMsg->header.stamp = time;

			rightPub.publish(imageRosMsg);
			rightCamInfoPub.publish(camInfoB);
		}

		if(!odom.data().laserScanRaw().isEmpty())
		{
			if(scanPub.getNumSubscribers() && odom.data().laserScanRaw().is2d())
			{
				//inspired from pointcloud_to_laserscan package
				sensor_msgs::LaserScan msg;
				msg.header.frame_id = scanFrameId;
				msg.header.stamp = time;

				msg.angle_min = scanAngleMin;
				msg.angle_max = scanAngleMax;
				msg.angle_increment = scanAngleIncrement;
				msg.time_increment = 0.0;
				msg.scan_time = 0;
				msg.range_min = scanRangeMin;
				msg.range_max = scanRangeMax;
				if(odom.data().laserScanRaw().angleIncrement() > 0.0f)
				{
					msg.angle_min = odom.data().laserScanRaw().angleMin();
					msg.angle_max = odom.data().laserScanRaw().angleMax();
					msg.angle_increment = odom.data().laserScanRaw().angleIncrement();
					msg.range_min = odom.data().laserScanRaw().rangeMin();
					msg.range_max = odom.data().laserScanRaw().rangeMax();
				}

				uint32_t rangesSize = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
				msg.ranges.assign(rangesSize, 0.0);

				const cv::Mat & scan = odom.data().laserScanRaw().data();
				for (int i=0; i<scan.cols; ++i)
				{
					const float * ptr = scan.ptr<float>(0,i);
					double range = hypot(ptr[0], ptr[1]);
					if (range >= msg.range_min && range <=msg.range_max)
					{
						double angle = atan2(ptr[1], ptr[0]);
						if (angle >= msg.angle_min && angle <= msg.angle_max)
						{
							int index = (angle - msg.angle_min) / msg.angle_increment;
							if (index>=0 && index<rangesSize && (range < msg.ranges[index] || msg.ranges[index]==0))
							{
								msg.ranges[index] = range;
							}
						}
					}
				}

				scanPub.publish(msg);
			}
			else if(scanCloudPub.getNumSubscribers())
			{
				sensor_msgs::PointCloud2 msg;
				pcl_conversions::moveFromPCL(*rtabmap::util3d::laserScanToPointCloud2(odom.data().laserScanRaw()), msg);
				msg.header.frame_id = scanFrameId;
				msg.header.stamp = time;
				scanCloudPub.publish(msg);
			}
		}

		if(odom.data().userDataRaw().type() == CV_8SC1 &&
		   odom.data().userDataRaw().cols >= 7 && // including null str ending
		   odom.data().userDataRaw().rows == 1 &&
		   memcmp(odom.data().userDataRaw().data, "GOAL:", 5) == 0)
		{
			//GOAL format detected, remove it from the user data and send it as goal event
			std::string goalStr = (const char *)odom.data().userDataRaw().data;
			if(!goalStr.empty())
			{
				std::list<std::string> strs = uSplit(goalStr, ':');
				if(strs.size() == 2)
				{
					int goalId = atoi(strs.rbegin()->c_str());

					if(goalId > 0)
					{
						ROS_WARN("Goal %d detected, calling rtabmap's set_goal service!", goalId);
						rtabmap_msgs::SetGoal setGoalSrv;
						setGoalSrv.request.node_id = goalId;
						setGoalSrv.request.node_label = "";
						if(!ros::service::call("set_goal", setGoalSrv))
						{
							ROS_ERROR("Can't call \"set_goal\" service");
						}
					}
				}
			}
		}

		ros::spinOnce();

		while(ros::ok())
		{
#ifndef _WIN32
			if (spacehit()) {
				paused = !paused;
				if(paused)
				{
					ROS_INFO("paused!");
				}
				else
				{
					ROS_INFO("resumed!");
				}
			}
#endif

			if(!paused)
			{
				break;
			}

			uSleep(100);
			ros::spinOnce();
		}

		timer.restart();
		cameraInfo = rtabmap::CameraInfo();
		data = reader.takeImage(&cameraInfo);
		odomInfo.reg.covariance = cameraInfo.odomCovariance;
		odom = rtabmap::OdometryEvent(data, cameraInfo.odomPose, odomInfo);
		acquisitionTime = timer.ticks();
	}


	return 0;
}
