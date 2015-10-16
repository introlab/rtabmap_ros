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

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/OdomInfo.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UFile.h"

using namespace rtabmap;

namespace rtabmap_ros {

OdometryROS::OdometryROS(int argc, char * argv[], bool stereo) :
	odometry_(0),
	frameId_("base_link"),
	odomFrameId_("odom"),
	groundTruthFrameId_(""),
	publishTf_(true),
	waitForTransform_(true),
	waitForTransformDuration_(0.1), // 100 ms
	paused_(false)
{
	ros::NodeHandle nh;

	odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
	odomInfoPub_ = nh.advertise<rtabmap_ros::OdomInfo>("odom_info", 1);
	odomLocalMap_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_map", 1);
	odomLastFrame_ = nh.advertise<sensor_msgs::PointCloud2>("odom_last_frame", 1);

	ros::NodeHandle pnh("~");

	Transform initialPose = Transform::getIdentity();
	std::string initialPoseStr;
	std::string tfPrefix;
	std::string configPath;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
	pnh.param("publish_tf", publishTf_, publishTf_);
	pnh.param("tf_prefix", tfPrefix, tfPrefix);
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("initial_pose", initialPoseStr, initialPoseStr); // "x y z roll pitch yaw"
	pnh.param("ground_truth_frame_id", groundTruthFrameId_, groundTruthFrameId_);
	pnh.param("config_path", configPath, configPath);
	configPath = uReplaceChar(configPath, '~', UDirectory::homeDir());
	if(configPath.size() && configPath.at(0) != '/')
	{
		configPath = UDirectory::currentDir(true) + configPath;
	}

	if(!tfPrefix.empty())
	{
		if(!frameId_.empty())
		{
			frameId_ = tfPrefix + "/" + frameId_;
		}
		if(!odomFrameId_.empty())
		{
			odomFrameId_ = tfPrefix + "/" + odomFrameId_;
		}
		if(!groundTruthFrameId_.empty())
		{
			groundTruthFrameId_ = tfPrefix + "/" + groundTruthFrameId_;
		}
	}

	if(initialPoseStr.size())
	{
		std::vector<std::string> values = uListToVector(uSplit(initialPoseStr, ' '));
		if(values.size() == 6)
		{
			initialPose = Transform(
					uStr2Float(values[0]), uStr2Float(values[1]), uStr2Float(values[2]),
					uStr2Float(values[3]), uStr2Float(values[4]), uStr2Float(values[5]));
		}
		else
		{
			ROS_ERROR("Wrong initial_pose format: %s (should be \"x y z roll pitch yaw\" with angle in radians). "
					  "Identity will be used...", initialPoseStr.c_str());
		}
	}


	//parameters
	parameters_ = this->getDefaultOdometryParameters(stereo);
	if(!configPath.empty())
	{
		if(UFile::exists(configPath.c_str()))
		{
			ROS_INFO("Odometry: Loading parameters from %s", configPath.c_str());
			rtabmap::ParametersMap allParameters;
			Rtabmap::readParameters(configPath.c_str(), allParameters);
			// only update odometry parameters
			for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
			{
				ParametersMap::iterator jter = allParameters.find(iter->first);
				if(jter!=allParameters.end())
				{
					iter->second = jter->second;
				}
			}
		}
		else
		{
			ROS_ERROR("Config file \"%s\" not found!", configPath.c_str());
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
		else if(pnh.getParam(iter->first, vDouble))
		{
			ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			ROS_INFO("Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}

		if(iter->first.compare(Parameters::kOdomMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
		{
			ROS_WARN("Parameter min_inliers must be >= 8, setting to 8...");
			iter->second = uNumber2Str(8);
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
	oldParameterNames.push_back("GFTT/MaxCorners");
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
			else if(iter->compare("Odom/NNDR") == 0)
			{
				ROS_WARN("Parameter name changed: Odom/NNDR -> %s. Please update your launch file accordingly.",
						Parameters::kOdomBowNNDR().c_str());
				parameters_.at(Parameters::kOdomBowNNDR())= vStr;
			}
			else if(iter->compare("GFTT/MaxCorners") == 0)
			{
				ROS_WARN("Parameter GFTT/MaxCorners doesn't exist anymore, use %s. Please update your launch file accordingly.",
						Parameters::kOdomMaxFeatures().c_str());
				parameters_.at(Parameters::kOdomMaxFeatures())= vStr;
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

rtabmap::ParametersMap OdometryROS::getDefaultOdometryParameters(bool stereo)
{
	rtabmap::ParametersMap odomParameters;
	rtabmap::ParametersMap defaultParameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::iterator iter=defaultParameters.begin(); iter!=defaultParameters.end(); ++iter)
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
			if(stereo)
			{
				if(iter->first.compare(Parameters::kOdomMaxDepth()) == 0)
				{
					iter->second = "0"; // infinity
				}
				else if(iter->first.compare(Parameters::kOdomEstimationType()) == 0)
				{
					iter->second = "1"; // 3D->2D (PNP)
				}
			}
			odomParameters.insert(*iter);
		}
	}
	return odomParameters;
}

void OdometryROS::processArguments(int argc, char * argv[], bool stereo)
{
	for(int i=1;i<argc;++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parametersOdom = getDefaultOdometryParameters(stereo);
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

Transform OdometryROS::getTransform(const std::string & fromFrameId, const std::string & toFrameId, const ros::Time & stamp) const
{
	// TF ready?
	Transform transform;
	try
	{
		if(waitForTransform_ && !stamp.isZero() && waitForTransformDuration_ > 0.0)
		{
			//if(!tfBuffer_.canTransform(fromFrameId, toFrameId, stamp, ros::Duration(1)))
			if(!tfListener_.waitForTransform(fromFrameId, toFrameId, stamp, ros::Duration(waitForTransformDuration_)))
			{
				ROS_WARN("odometry: Could not get transform from %s to %s (stamp=%f) after %f seconds (\"wait_for_transform_duration\"=%f)!",
						fromFrameId.c_str(), toFrameId.c_str(), stamp.toSec(), waitForTransformDuration_, waitForTransformDuration_);
				return transform;
			}
		}

		tf::StampedTransform tmp;
		tfListener_.lookupTransform(fromFrameId, toFrameId, stamp, tmp);
		transform = rtabmap_ros::transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		ROS_WARN("%s",ex.what());
	}
	return transform;
}

void OdometryROS::processData(const SensorData & data, const ros::Time & stamp)
{
	if(odometry_->getPose().isNull() &&
	   !groundTruthFrameId_.empty())
	{
		// sync with the first value of the ground truth
		Transform initialPose = getTransform(groundTruthFrameId_, frameId_, stamp);
		if(initialPose.isNull())
		{
			return;
		}

		ROS_INFO("Initializing odometry pose to %s (from \"%s\" -> \"%s\")",
				initialPose.prettyPrint().c_str(),
				groundTruthFrameId_.c_str(),
				frameId_.c_str());
		odometry_->reset(initialPose);
	}

	// process data
	ros::WallTime time = ros::WallTime::now();
	rtabmap::OdometryInfo info;
	rtabmap::Transform pose = odometry_->process(data, &info);
	if(!pose.isNull())
	{
		//*********************
		// Update odometry
		//*********************
		geometry_msgs::TransformStamped poseMsg;
		poseMsg.child_frame_id = frameId_;
		poseMsg.header.frame_id = odomFrameId_;
		poseMsg.header.stamp = stamp;
		rtabmap_ros::transformToGeometryMsg(pose, poseMsg.transform);

		if(publishTf_)
		{
			tfBroadcaster_.sendTransform(poseMsg);
		}

		if(odomPub_.getNumSubscribers())
		{
			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = stamp; // use corresponding time stamp to image
			odom.header.frame_id = odomFrameId_;
			odom.child_frame_id = frameId_;

			//set the position
			odom.pose.pose.position.x = poseMsg.transform.translation.x;
			odom.pose.pose.position.y = poseMsg.transform.translation.y;
			odom.pose.pose.position.z = poseMsg.transform.translation.z;
			odom.pose.pose.orientation = poseMsg.transform.rotation;

			//set covariance
			odom.pose.covariance.at(0) = info.variance;  // xx
			odom.pose.covariance.at(7) = info.variance;  // yy
			odom.pose.covariance.at(14) = info.variance; // zz
			odom.pose.covariance.at(21) = info.variance; // rr
			odom.pose.covariance.at(28) = info.variance; // pp
			odom.pose.covariance.at(35) = info.variance; // yawyaw

			//publish the message
			odomPub_.publish(odom);
		}

		if(odomLocalMap_.getNumSubscribers() && dynamic_cast<OdometryBOW*>(odometry_))
		{
			const std::map<int, pcl::PointXYZ> & map = ((OdometryBOW*)odometry_)->getLocalMap();
			pcl::PointCloud<pcl::PointXYZ> cloud;
			for(std::map<int, pcl::PointXYZ>::const_iterator iter=map.begin(); iter!=map.end(); ++iter)
			{
				cloud.push_back(iter->second);
			}
			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
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
					for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=words3.begin(); iter!=words3.end(); ++iter)
					{
						// transform to odom frame
						pcl::PointXYZ pt = util3d::transformPoint(iter->second, pose);
						cloud.push_back(pt);
					}

					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
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
					cloudTransformed = util3d::transformPointCloud(cloud, pose);
					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(*cloudTransformed, cloudMsg);
					cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
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
		odom.header.stamp = stamp; // use corresponding time stamp to image
		odom.header.frame_id = odomFrameId_;
		odom.child_frame_id = frameId_;

		//publish the message
		odomPub_.publish(odom);
	}

	if(odomInfoPub_.getNumSubscribers())
	{
		rtabmap_ros::OdomInfo infoMsg;
		odomInfoToROS(info, infoMsg);
		infoMsg.header.stamp = stamp; // use corresponding time stamp to image
		infoMsg.header.frame_id = odomFrameId_;
		odomInfoPub_.publish(infoMsg);
	}

	ROS_INFO("Odom: quality=%d, std dev=%fm, update time=%fs", info.inliers, pose.isNull()?0.0f:std::sqrt(info.variance), (ros::WallTime::now()-time).toSec());
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
