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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/DBReader.h>

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


	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string frameId = "base_link";
	std::string odomFrameId = "odom";
	std::string cameraFrameId = "camera_optical_link";
	double rate = 1.0f;
	std::string databasePath = "";
	bool publishTf = true;
	int startId = 0;

	pnh.param("frame_id", frameId, frameId);
	pnh.param("odom_frame_id", odomFrameId, odomFrameId);
	pnh.param("camera_frame_id", cameraFrameId, cameraFrameId);
	pnh.param("rate", rate, rate);
	pnh.param("database", databasePath, databasePath);
	pnh.param("publish_tf", publishTf, publishTf);
	pnh.param("start_id", startId, startId);

	ROS_INFO("frame_id = %s", frameId.c_str());
	ROS_INFO("odom_frame_id = %s", odomFrameId.c_str());
	ROS_INFO("camera_frame_id = %s", cameraFrameId.c_str());
	ROS_INFO("database = %s", databasePath.c_str());
	ROS_INFO("rate = %f", rate);
	ROS_INFO("publish_tf = %s", publishTf?"true":"false");

	rtabmap::DBReader reader(databasePath, rate);

	if(databasePath.empty())
	{
		ROS_ERROR("Parameter \"database\" must be set (path to a RTAB-Map database).");
		return -1;
	}

	if(!reader.init(startId))
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
	tf::TransformBroadcaster tfBroadcaster;

	rtabmap::SensorData data = reader.getNextData();
	while(ros::ok() && data.isValid())
	{
		ROS_INFO("Reading sensor data %d...", data.id());

		ros::Time time = ros::Time::now();

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
		if(!data.depth().empty() && (data.depth().type() == CV_32FC1 || data.depth().type() == CV_16UC1))
		{
			//depth
			camInfoA.D.resize(5,0);

			camInfoA.P[0] = data.fx();
			camInfoA.K[0] = data.fx();
			camInfoA.P[5] = data.fy();
			camInfoA.K[4] = data.fy();
			camInfoA.P[2] = data.cx();
			camInfoA.K[2] = data.cx();
			camInfoA.P[6] = data.cy();
			camInfoA.K[5] = data.cy();

			camInfoB = camInfoA;

			type=0;

			if(rgbPub.getTopic().empty()) rgbPub = it.advertise("rgb/image", 1);
			if(depthPub.getTopic().empty()) depthPub = it.advertise("depth_registered/image", 1);
			if(rgbCamInfoPub.getTopic().empty()) rgbCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("rgb/camera_info", 1);
			if(depthCamInfoPub.getTopic().empty()) depthCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("depth_registered/camera_info", 1);
		}
		else if(!data.rightImage().empty() && data.rightImage().type() == CV_8U)
		{
			//stereo
			camInfoA.D.resize(8,0);

			camInfoA.P[0] = data.fx();
			camInfoA.K[0] = data.fx();
			camInfoA.P[5] = data.fx(); // fx = fy
			camInfoA.K[4] = data.fx(); // fx = fy
			camInfoA.P[2] = data.cx();
			camInfoA.K[2] = data.cx();
			camInfoA.P[6] = data.cy();
			camInfoA.K[5] = data.cy();

			camInfoB = camInfoA;
			camInfoB.P[3] = data.baseline()*-data.fx(); // Right_Tx = -baseline*fx

			type=1;

			if(leftPub.getTopic().empty()) leftPub = it.advertise("left/image", 1);
			if(rightPub.getTopic().empty()) rightPub = it.advertise("right/image", 1);
			if(leftCamInfoPub.getTopic().empty()) leftCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
			if(rightCamInfoPub.getTopic().empty()) rightCamInfoPub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

		}
		else
		{
			if(imagePub.getTopic().empty()) imagePub = it.advertise("image", 1);
		}

		camInfoA.height = data.image().rows;
		camInfoA.width = data.image().cols;
		camInfoB.height = data.depthOrRightImage().rows;
		camInfoB.width = data.depthOrRightImage().cols;

		if(!data.laserScan().empty())
		{
			if(scanPub.getTopic().empty()) scanPub = nh.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1);
		}

		// publish transforms first
		if(publishTf)
		{
			ros::Time tfExpiration = time + ros::Duration(1.0/rate);
			if(!data.localTransform().isNull())
			{
				tf::Transform baseToCamera;
				rtabmap_ros::transformToTF(data.localTransform(), baseToCamera);
				tfBroadcaster.sendTransform( tf::StampedTransform (baseToCamera, tfExpiration, frameId, cameraFrameId));
			}

			if(!data.pose().isNull())
			{
				tf::Transform odomToBase;
				rtabmap_ros::transformToTF(data.pose(), odomToBase);
				tfBroadcaster.sendTransform( tf::StampedTransform (odomToBase, tfExpiration, odomFrameId, frameId));
			}
		}
		if(!data.pose().isNull())
		{
			if(odometryPub.getTopic().empty()) odometryPub = nh.advertise<nav_msgs::Odometry>("odom", 1);

			if(odometryPub.getNumSubscribers())
			{
				nav_msgs::Odometry odom;
				odom.child_frame_id = frameId;
				odom.header.frame_id = odomFrameId;
				odom.header.stamp = time;
				rtabmap_ros::transformToPoseMsg(data.pose(), odom.pose.pose);
				odom.pose.covariance[0] = data.poseVariance();
				odom.pose.covariance[7] = data.poseVariance();
				odom.pose.covariance[14] = data.poseVariance();
				odom.pose.covariance[21] = data.poseVariance();
				odom.pose.covariance[28] = data.poseVariance();
				odom.pose.covariance[35] = data.poseVariance();
				odometryPub.publish(odom);
			}
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
			if(data.image().channels() == 1)
			{
				img.encoding = sensor_msgs::image_encodings::MONO8;
			}
			else
			{
				img.encoding = sensor_msgs::image_encodings::BGR8;
			}
			img.image = data.image();
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

		if(depthPub.getNumSubscribers() && !data.depth().empty() && type==0)
		{
			cv_bridge::CvImage img;
			if(data.depth().type() == CV_32FC1)
			{
				img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			}
			else
			{
				img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
			}
			img.image = data.depth();
			sensor_msgs::ImagePtr imageRosMsg = img.toImageMsg();
			imageRosMsg->header.frame_id = cameraFrameId;
			imageRosMsg->header.stamp = time;

			depthPub.publish(imageRosMsg);
			depthCamInfoPub.publish(camInfoB);
		}

		if(rightPub.getNumSubscribers() && !data.rightImage().empty() && type==1)
		{
			cv_bridge::CvImage img;
			img.encoding = sensor_msgs::image_encodings::MONO8;
			img.image = data.rightImage();
			sensor_msgs::ImagePtr imageRosMsg = img.toImageMsg();
			imageRosMsg->header.frame_id = cameraFrameId;
			imageRosMsg->header.stamp = time;

			rightPub.publish(imageRosMsg);
			rightCamInfoPub.publish(camInfoB);
		}

		if(scanPub.getNumSubscribers() && !data.laserScan().empty())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(data.laserScan());
			sensor_msgs::PointCloud2 msg;
			pcl::toROSMsg(*cloud, msg);
			msg.header.frame_id = frameId;
			msg.header.stamp = time;
			scanPub.publish(msg);
		}

		ros::spinOnce();

		while(ros::ok() && paused)
		{
			uSleep(100);
			ros::spinOnce();
		}

		data = reader.getNextData();
	}


	return 0;
}
