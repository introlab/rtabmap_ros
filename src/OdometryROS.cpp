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

#include "rtabmap_ros/OdometryROS.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/OdometryF2M.h>
#include <rtabmap/core/OdometryF2F.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Signature.h>
#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/OdomInfo.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"

#define BAD_COVARIANCE 9999

using namespace rtabmap;

namespace rtabmap_ros {

OdometryROS::OdometryROS(bool stereoParams, bool visParams, bool icpParams) :
	odometry_(0),
	warningThread_(0),
	callbackCalled_(false),
	frameId_("base_link"),
	odomFrameId_("odom"),
	groundTruthFrameId_(""),
	groundTruthBaseFrameId_(""),
	guessFrameId_(""),
	guessMinTranslation_(0.0),
	guessMinRotation_(0.0),
	publishTf_(true),
	waitForTransform_(true),
	waitForTransformDuration_(0.1), // 100 ms
	publishNullWhenLost_(true),
	paused_(false),
	resetCountdown_(0),
	resetCurrentCount_(0),
	stereoParams_(stereoParams),
	visParams_(visParams),
	icpParams_(icpParams),
	guessStamp_(0.0)
{

}

OdometryROS::~OdometryROS()
{
	if(warningThread_)
	{
		callbackCalled();
		warningThread_->join();
		delete warningThread_;
	}
	ros::NodeHandle & pnh = getPrivateNodeHandle();
	if(pnh.ok())
	{
		for(ParametersMap::iterator iter=parameters_.begin(); iter!=parameters_.end(); ++iter)
		{
			pnh.deleteParam(iter->first);
		}
	}

	delete odometry_;
}

void OdometryROS::onInit()
{
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();

	odomPub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
	odomInfoPub_ = nh.advertise<rtabmap_ros::OdomInfo>("odom_info", 1);
	odomLocalMap_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_map", 1);
	odomLocalScanMap_ = nh.advertise<sensor_msgs::PointCloud2>("odom_local_scan_map", 1);
	odomLastFrame_ = nh.advertise<sensor_msgs::PointCloud2>("odom_last_frame", 1);

	Transform initialPose = Transform::getIdentity();
	std::string initialPoseStr;
	std::string configPath;
	pnh.param("frame_id", frameId_, frameId_);
	pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
	pnh.param("publish_tf", publishTf_, publishTf_);
	if(pnh.hasParam("tf_prefix"))
	{
		NODELET_ERROR("tf_prefix parameter has been removed, use directly odom_frame_id and frame_id parameters.");
	}
	pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
	pnh.param("wait_for_transform_duration",  waitForTransformDuration_, waitForTransformDuration_);
	pnh.param("initial_pose", initialPoseStr, initialPoseStr); // "x y z roll pitch yaw"
	pnh.param("ground_truth_frame_id", groundTruthFrameId_, groundTruthFrameId_);
	pnh.param("ground_truth_base_frame_id", groundTruthBaseFrameId_, frameId_);
	pnh.param("config_path", configPath, configPath);
	pnh.param("publish_null_when_lost", publishNullWhenLost_, publishNullWhenLost_);
	if(pnh.hasParam("guess_from_tf"))
	{
		if(!pnh.hasParam("guess_frame_id"))
		{
			NODELET_ERROR("Parameter \"guess_from_tf\" doesn't exist anymore, it is enabled if \"guess_frame_id\" is set.");
		}
		else
		{
			NODELET_WARN("Parameter \"guess_from_tf\" doesn't exist anymore, it is enabled if \"guess_frame_id\" is set.");
		}
	}
	pnh.param("guess_frame_id", guessFrameId_, guessFrameId_); // odometry guess frame
	pnh.param("guess_min_translation", guessMinTranslation_, guessMinTranslation_);
	pnh.param("guess_min_rotation", guessMinRotation_, guessMinRotation_);

	if(publishTf_ && !guessFrameId_.empty() && guessFrameId_.compare(odomFrameId_) == 0)
	{
		NODELET_WARN( "\"publish_tf\" and \"guess_frame_id\" cannot be used "
				"at the same time if \"guess_frame_id\" and \"odom_frame_id\" "
				"are the same frame (value=\"%s\"). \"guess_frame_id\" is disabled.", odomFrameId_.c_str());
		guessFrameId_.clear();
	}
	NODELET_INFO("Odometry: frame_id               = %s", frameId_.c_str());
	NODELET_INFO("Odometry: odom_frame_id          = %s", odomFrameId_.c_str());
	NODELET_INFO("Odometry: publish_tf             = %s", publishTf_?"true":"false");
	NODELET_INFO("Odometry: wait_for_transform     = %s", waitForTransform_?"true":"false");
	NODELET_INFO("Odometry: wait_for_transform_duration  = %f", waitForTransformDuration_);
	NODELET_INFO("Odometry: initial_pose           = %s", initialPose.prettyPrint().c_str());
	NODELET_INFO("Odometry: ground_truth_frame_id  = %s", groundTruthFrameId_.c_str());
	NODELET_INFO("Odometry: ground_truth_base_frame_id = %s", groundTruthBaseFrameId_.c_str());
	NODELET_INFO("Odometry: config_path            = %s", configPath.c_str());
	NODELET_INFO("Odometry: publish_null_when_lost = %s", publishNullWhenLost_?"true":"false");
	NODELET_INFO("Odometry: guess_frame_id         = %s", guessFrameId_.c_str());
	NODELET_INFO("Odometry: guess_min_translation  = %f", guessMinTranslation_);
	NODELET_INFO("Odometry: guess_min_rotation     = %f", guessMinRotation_);

	configPath = uReplaceChar(configPath, '~', UDirectory::homeDir());
	if(configPath.size() && configPath.at(0) != '/')
	{
		configPath = UDirectory::currentDir(true) + configPath;
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
			NODELET_ERROR( "Wrong initial_pose format: %s (should be \"x y z roll pitch yaw\" with angle in radians). "
					  "Identity will be used...", initialPoseStr.c_str());
		}
	}


	//parameters
	parameters_ = Parameters::getDefaultOdometryParameters(stereoParams_, visParams_, icpParams_);
	if(!configPath.empty())
	{
		if(UFile::exists(configPath.c_str()))
		{
			NODELET_INFO( "Odometry: Loading parameters from %s", configPath.c_str());
			rtabmap::ParametersMap allParameters;
			Parameters::readINI(configPath.c_str(), allParameters);
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
			NODELET_ERROR( "Config file \"%s\" not found!", configPath.c_str());
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
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
		else if(pnh.getParam(iter->first, vBool))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
			iter->second = uBool2Str(vBool);
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			NODELET_INFO( "Setting odometry parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}

		if(iter->first.compare(Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
		{
			NODELET_WARN( "Parameter min_inliers must be >= 8, setting to 8...");
			iter->second = uNumber2Str(8);
		}
	}

	std::vector<std::string> argList = getMyArgv();
	char * argv[argList.size()];
	for(unsigned int i=0; i<argList.size(); ++i)
	{
		argv[i] = &argList[i].at(0);
	}

	rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		rtabmap::ParametersMap::iterator jter = parameters_.find(iter->first);
		if(jter!=parameters_.end())
		{
			NODELET_INFO( "Update odometry parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
			jter->second = iter->second;
		}
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=Parameters::getRemovedParameters().begin();
		iter!=Parameters::getRemovedParameters().end();
		++iter)
	{
		std::string vStr;
		if(pnh.getParam(iter->first, vStr))
		{
			if(iter->second.first && parameters_.find(iter->second.second) != parameters_.end())
			{
				// can be migrated
				parameters_.at(iter->second.second)= vStr;
				NODELET_WARN( "Odometry: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					NODELET_ERROR( "Odometry: Parameter \"%s\" doesn't exist anymore!",
							iter->first.c_str());
				}
				else
				{
					NODELET_ERROR( "Odometry: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	Parameters::parse(parameters_, Parameters::kOdomResetCountdown(), resetCountdown_);
	parameters_.at(Parameters::kOdomResetCountdown()) = "0"; // use modified reset countdown here

	this->updateParameters(parameters_);

	odometry_ = Odometry::create(parameters_);
	if(!initialPose.isIdentity())
	{
		odometry_->reset(initialPose);
	}

	resetSrv_ = nh.advertiseService("reset_odom", &OdometryROS::reset, this);
	resetToPoseSrv_ = nh.advertiseService("reset_odom_to_pose", &OdometryROS::resetToPose, this);
	pauseSrv_ = nh.advertiseService("pause_odom", &OdometryROS::pause, this);
	resumeSrv_ = nh.advertiseService("resume_odom", &OdometryROS::resume, this);

	setLogDebugSrv_ = pnh.advertiseService("log_debug", &OdometryROS::setLogDebug, this);
	setLogInfoSrv_ = pnh.advertiseService("log_info", &OdometryROS::setLogInfo, this);
	setLogWarnSrv_ = pnh.advertiseService("log_warning", &OdometryROS::setLogWarn, this);
	setLogErrorSrv_ = pnh.advertiseService("log_error", &OdometryROS::setLogError, this);

	onOdomInit();
}

void OdometryROS::startWarningThread(const std::string & subscribedTopicsMsg, bool approxSync)
{
	warningThread_ = new boost::thread(boost::bind(&OdometryROS::warningLoop, this, subscribedTopicsMsg, approxSync));
	NODELET_INFO("%s", subscribedTopicsMsg.c_str());
}

void OdometryROS::warningLoop(const std::string & subscribedTopicsMsg, bool approxSync)
{
	ros::Duration r(5.0);
	while(!callbackCalled_)
	{
		r.sleep();
		if(!callbackCalled_)
		{
			ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. %s%s",
					getName().c_str(),
					approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
						"topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg.c_str());
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
			std::string errorMsg;
			if(!tfListener_.waitForTransform(fromFrameId, toFrameId, stamp, ros::Duration(waitForTransformDuration_), ros::Duration(0.01), &errorMsg))
			{
				NODELET_WARN( "odometry: Could not get transform from %s to %s (stamp=%f) after %f seconds (\"wait_for_transform_duration\"=%f)! Error=\"%s\"",
						fromFrameId.c_str(), toFrameId.c_str(), stamp.toSec(), waitForTransformDuration_, waitForTransformDuration_, errorMsg.c_str());
				return transform;
			}
		}

		tf::StampedTransform tmp;
		tfListener_.lookupTransform(fromFrameId, toFrameId, stamp, tmp);
		transform = rtabmap_ros::transformFromTF(tmp);
	}
	catch(tf::TransformException & ex)
	{
		NODELET_WARN( "%s",ex.what());
	}
	return transform;
}

void OdometryROS::processData(const SensorData & data, const ros::Time & stamp)
{
	if(odometry_->getPose().isIdentity() &&
	   !groundTruthFrameId_.empty())
	{
		// sync with the first value of the ground truth
		Transform initialPose = getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, stamp);
		if(initialPose.isNull())
		{
			NODELET_WARN("Ground truth frames \"%s\" -> \"%s\" are set but failed to "
					"get them, odometry won't be synchronized with ground truth.",
					groundTruthFrameId_.c_str(), groundTruthBaseFrameId_.c_str());
		}
		else
		{
			NODELET_INFO( "Initializing odometry pose to %s (from \"%s\" -> \"%s\")",
					initialPose.prettyPrint().c_str(),
					groundTruthFrameId_.c_str(),
					groundTruthBaseFrameId_.c_str());
			odometry_->reset(initialPose);
		}
	}

	Transform guessCurrentPose;
	if(!guessFrameId_.empty())
	{
		Transform previousPose = this->getTransform(guessFrameId_, frameId_, guessStamp_>0.0?ros::Time(guessStamp_):stamp);
		guessCurrentPose = this->getTransform(guessFrameId_, frameId_, stamp);
		if(!previousPose.isNull() && !guessCurrentPose.isNull())
		{
			if(guess_.isNull())
			{
				guess_ = previousPose.inverse() * guessCurrentPose;
			}
			else
			{
				guess_ = guess_ * previousPose.inverse() * guessCurrentPose;
			}
			if(guessStamp_>0.0 && (guessMinTranslation_ > 0.0 || guessMinRotation_ > 0.0))
			{
				float x,y,z,roll,pitch,yaw;
				guess_.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				if((guessMinTranslation_ <= 0.0 || uMax3(fabs(x), fabs(y), fabs(z)) < guessMinTranslation_) &&
				   (guessMinRotation_ <= 0.0 || uMax3(fabs(roll), fabs(pitch), fabs(yaw)) < guessMinRotation_))
				{
					// Ignore odometry update, we didn't move enough
					if(publishTf_)
					{
						geometry_msgs::TransformStamped correctionMsg;
						correctionMsg.child_frame_id = guessFrameId_;
						correctionMsg.header.frame_id = odomFrameId_;
						correctionMsg.header.stamp = stamp;
						Transform correction = odometry_->getPose() * guess_ * guessCurrentPose.inverse();
						rtabmap_ros::transformToGeometryMsg(correction, correctionMsg.transform);
						tfBroadcaster_.sendTransform(correctionMsg);
					}
					guessStamp_ = stamp.toSec();
					return;
				}
			}
			guessStamp_ = stamp.toSec();
		}
		else
		{
			NODELET_ERROR("\"guess_from_tf\" is true, but guess cannot be computed between frames \"%s\" -> \"%s\". Aborting odometry update...", guessFrameId_.c_str(), frameId_.c_str());
			return;
		}
	}

	// process data
	ros::WallTime time = ros::WallTime::now();
	rtabmap::OdometryInfo info;
	SensorData dataCpy = data;
	rtabmap::Transform pose = odometry_->process(dataCpy, guess_, &info);
	guess_.setNull();
	if(!pose.isNull())
	{
		resetCurrentCount_ = resetCountdown_;

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
			if(!guessFrameId_.empty())
			{
				//publish correction of actual odometry so we have /odom -> /odom_guess -> /base_link
				geometry_msgs::TransformStamped correctionMsg;
				correctionMsg.child_frame_id = guessFrameId_;
				correctionMsg.header.frame_id = odomFrameId_;
				correctionMsg.header.stamp = stamp;
				Transform correction = pose * guessCurrentPose.inverse();
				rtabmap_ros::transformToGeometryMsg(correction, correctionMsg.transform);
				tfBroadcaster_.sendTransform(correctionMsg);
			}
			else
			{
				tfBroadcaster_.sendTransform(poseMsg);
			}
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
			// libviso2 uses approximately vel variance * 2
			odom.pose.covariance.at(0) = info.reg.covariance.at<double>(0,0)*2;  // xx
			odom.pose.covariance.at(7) = info.reg.covariance.at<double>(1,1)*2;  // yy
			odom.pose.covariance.at(14) = info.reg.covariance.at<double>(2,2)*2; // zz
			odom.pose.covariance.at(21) = info.reg.covariance.at<double>(3,3)*2; // rr
			odom.pose.covariance.at(28) = info.reg.covariance.at<double>(4,4)*2; // pp
			odom.pose.covariance.at(35) = info.reg.covariance.at<double>(5,5)*2; // yawyaw

			//set velocity
			bool setTwist = !odometry_->previousVelocityTransform().isNull();
			if(setTwist)
			{
				float x,y,z,roll,pitch,yaw;
				odometry_->previousVelocityTransform().getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
				odom.twist.twist.linear.x = x;
				odom.twist.twist.linear.y = y;
				odom.twist.twist.linear.z = z;
				odom.twist.twist.angular.x = roll;
				odom.twist.twist.angular.y = pitch;
				odom.twist.twist.angular.z = yaw;
			}

			odom.twist.covariance.at(0) = setTwist?info.reg.covariance.at<double>(0,0):BAD_COVARIANCE;  // xx
			odom.twist.covariance.at(7) = setTwist?info.reg.covariance.at<double>(1,1):BAD_COVARIANCE;  // yy
			odom.twist.covariance.at(14) = setTwist?info.reg.covariance.at<double>(2,2):BAD_COVARIANCE; // zz
			odom.twist.covariance.at(21) = setTwist?info.reg.covariance.at<double>(3,3):BAD_COVARIANCE; // rr
			odom.twist.covariance.at(28) = setTwist?info.reg.covariance.at<double>(4,4):BAD_COVARIANCE; // pp
			odom.twist.covariance.at(35) = setTwist?info.reg.covariance.at<double>(5,5):BAD_COVARIANCE; // yawyaw

			//publish the message
			if(setTwist || publishNullWhenLost_)
			{
				odomPub_.publish(odom);
			}
		}

		// local map / reference frame
		if(odomLocalMap_.getNumSubscribers() && !info.localMap.empty())
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud;
			for(std::map<int, cv::Point3f>::const_iterator iter=info.localMap.begin(); iter!=info.localMap.end(); ++iter)
			{
				bool inlier = info.words.find(iter->first) != info.words.end();
				pcl::PointXYZRGB pt(inlier?0:255, 255, 0);
				pt.x = iter->second.x;
				pt.y = iter->second.y;
				pt.z = iter->second.z;
				cloud.push_back(pt);
			}
			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
			cloudMsg.header.frame_id = odomFrameId_;
			odomLocalMap_.publish(cloudMsg);
		}

		if(odomLastFrame_.getNumSubscribers())
		{
			// check which type of Odometry is using
			if(odometry_->getType() == Odometry::kTypeF2M) // If it's Frame to Map Odometry
			{
				const std::multimap<int, cv::Point3f> & words3  = ((OdometryF2M*)odometry_)->getLastFrame().getWords3();
				if(words3.size())
				{
					pcl::PointCloud<pcl::PointXYZ> cloud;
					for(std::multimap<int, cv::Point3f>::const_iterator iter=words3.begin(); iter!=words3.end(); ++iter)
					{
						// transform to odom frame
						cv::Point3f pt = util3d::transformPoint(iter->second, pose);
						cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
					}

					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_.publish(cloudMsg);
				}
			}
			else if(odometry_->getType() == Odometry::kTypeF2F) // if Using Frame to Frame Odometry
			{
				const Signature & refFrame = ((OdometryF2F*)odometry_)->getRefFrame();

				if(refFrame.getWords3().size())
				{
					pcl::PointCloud<pcl::PointXYZ> cloud;
					for(std::multimap<int, cv::Point3f>::const_iterator iter=refFrame.getWords3().begin(); iter!=refFrame.getWords3().end(); ++iter)
					{
						// transform to odom frame
						cv::Point3f pt = util3d::transformPoint(iter->second, pose);
						cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
					}
					sensor_msgs::PointCloud2 cloudMsg;
					pcl::toROSMsg(cloud, cloudMsg);
					cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
					cloudMsg.header.frame_id = odomFrameId_;
					odomLastFrame_.publish(cloudMsg);
				}
			}
		}

		if(odomLocalScanMap_.getNumSubscribers() && !info.localScanMap.isEmpty())
		{
			sensor_msgs::PointCloud2 cloudMsg;
			if(info.localScanMap.hasNormals())
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr cloud = util3d::laserScanToPointCloudNormal(info.localScanMap, info.localScanMap.localTransform());
				pcl::toROSMsg(*cloud, cloudMsg);
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(info.localScanMap, info.localScanMap.localTransform());
				pcl::toROSMsg(*cloud, cloudMsg);
			}

			cloudMsg.header.stamp = stamp; // use corresponding time stamp to image
			cloudMsg.header.frame_id = odomFrameId_;
			odomLocalScanMap_.publish(cloudMsg);
		}
	}
	else if(publishNullWhenLost_)
	{
		//NODELET_WARN( "Odometry lost!");

		//send null pose to notify that odometry is lost
		nav_msgs::Odometry odom;
		odom.header.stamp = stamp; // use corresponding time stamp to image
		odom.header.frame_id = odomFrameId_;
		odom.child_frame_id = frameId_;
		odom.pose.covariance.at(0) = BAD_COVARIANCE;  // xx
		odom.pose.covariance.at(7) = BAD_COVARIANCE;  // yy
		odom.pose.covariance.at(14) = BAD_COVARIANCE; // zz
		odom.pose.covariance.at(21) = BAD_COVARIANCE; // rr
		odom.pose.covariance.at(28) = BAD_COVARIANCE; // pp
		odom.pose.covariance.at(35) = BAD_COVARIANCE; // yawyaw
		odom.twist.covariance.at(0) = BAD_COVARIANCE;  // xx
		odom.twist.covariance.at(7) = BAD_COVARIANCE;  // yy
		odom.twist.covariance.at(14) = BAD_COVARIANCE; // zz
		odom.twist.covariance.at(21) = BAD_COVARIANCE; // rr
		odom.twist.covariance.at(28) = BAD_COVARIANCE; // pp
		odom.twist.covariance.at(35) = BAD_COVARIANCE; // yawyaw

		//publish the message
		odomPub_.publish(odom);
	}

	if(pose.isNull() && resetCurrentCount_ > 0)
	{
		NODELET_WARN( "Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", resetCurrentCount_);

		--resetCurrentCount_;
		if(resetCurrentCount_ == 0)
		{
			// Check TF to see if sensor fusion is used (e.g., the output of robot_localization)
			Transform tfPose = this->getTransform(odomFrameId_, frameId_, stamp);
			if(tfPose.isNull())
			{
				NODELET_WARN( "Odometry automatically reset to latest computed pose!");
				odometry_->reset(odometry_->getPose());
			}
			else
			{
				NODELET_WARN( "Odometry automatically reset to latest odometry pose available from TF (%s->%s)!",
						odomFrameId_.c_str(), frameId_.c_str());
				odometry_->reset(tfPose);
			}

		}
	}

	if(odomInfoPub_.getNumSubscribers())
	{
		rtabmap_ros::OdomInfo infoMsg;
		odomInfoToROS(info, infoMsg);
		infoMsg.header.stamp = stamp; // use corresponding time stamp to image
		infoMsg.header.frame_id = odomFrameId_;
		odomInfoPub_.publish(infoMsg);
	}

	if(visParams_)
	{
		if(icpParams_)
		{
			NODELET_INFO( "Odom: quality=%d, ratio=%f, std dev=%fm|%frad, update time=%fs", info.reg.inliers, info.reg.icpInliersRatio, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (ros::WallTime::now()-time).toSec());
		}
		else
		{
			NODELET_INFO( "Odom: quality=%d, std dev=%fm|%frad, update time=%fs", info.reg.inliers, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (ros::WallTime::now()-time).toSec());
		}
	}
	else
	{
		NODELET_INFO( "Odom: ratio=%f, std dev=%fm|%frad, update time=%fs", info.reg.icpInliersRatio, pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(0,0)), pose.isNull()?0.0f:std::sqrt(info.reg.covariance.at<double>(5,5)), (ros::WallTime::now()-time).toSec());
	}

}

bool OdometryROS::reset(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: reset odom!");
	odometry_->reset();
	guess_.setNull();
	guessStamp_ = 0.0;
	resetCurrentCount_ = resetCountdown_;
	this->flushCallbacks();
	return true;
}

bool OdometryROS::resetToPose(rtabmap_ros::ResetPose::Request& req, rtabmap_ros::ResetPose::Response&)
{
	Transform pose(req.x, req.y, req.z, req.roll, req.pitch, req.yaw);
	NODELET_INFO( "visual_odometry: reset odom to pose %s!", pose.prettyPrint().c_str());
	odometry_->reset(pose);
	guess_.setNull();
	guessStamp_ = 0.0;
	resetCurrentCount_ = resetCountdown_;
	this->flushCallbacks();
	return true;
}

bool OdometryROS::pause(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(paused_)
	{
		NODELET_WARN( "visual_odometry: Already paused!");
	}
	else
	{
		paused_ = true;
		NODELET_INFO( "visual_odometry: paused!");
	}
	return true;
}

bool OdometryROS::resume(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	if(!paused_)
	{
		NODELET_WARN( "visual_odometry: Already running!");
	}
	else
	{
		paused_ = false;
		NODELET_INFO( "visual_odometry: resumed!");
	}
	return true;
}

bool OdometryROS::setLogDebug(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Debug");
	ULogger::setLevel(ULogger::kDebug);
	return true;
}
bool OdometryROS::setLogInfo(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Info");
	ULogger::setLevel(ULogger::kInfo);
	return true;
}
bool OdometryROS::setLogWarn(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Warning");
	ULogger::setLevel(ULogger::kWarning);
	return true;
}
bool OdometryROS::setLogError(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
	NODELET_INFO( "visual_odometry: Set log level to Error");
	ULogger::setLevel(ULogger::kError);
	return true;
}


}
