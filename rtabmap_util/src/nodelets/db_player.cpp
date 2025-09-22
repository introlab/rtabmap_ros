/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap_util/db_player.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_transport/image_transport.hpp>

#include <rtabmap_conversions/MsgConversion.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/SensorData.h>

#include <pcl_conversions/pcl_conversions.h>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace rtabmap_util
{

DbPlayer::DbPlayer(const rclcpp::NodeOptions & options) :
	rclcpp::Node("db_player", options),
    paused_(false),
    frameId_("base_link"),
    odomFrameId_("odom"),
    cameraFrameId_("camera_optical_link"),
    scanFrameId_("base_laser_link"),
    gtFrameId_("world"),
    gtBaseFrameId_("base_link_gt"),
    qos_(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
{
    //ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	//ULogger::setEventLevel(ULogger::kWarning);

    //parse input arguments
    bool publishClock = false;
    publishClock = this->declare_parameter("publish_clock", publishClock);
	std::vector<std::string> tmpList = get_node_options().arguments();
	std::vector<std::string> argList;
	for(unsigned int i=0; i<tmpList.size(); ++i)
	{
	   if(tmpList[i].compare("--clock") == 0)
		{
			publishClock = true;
		}
	}

	double rate = 1.0f;
	std::string databasePath = "";
	bool publishTf = true;
    bool ignoreOdom = false;
	int startId = 0;
    frameId_ =        this->declare_parameter("frame_id", frameId_);
    odomFrameId_ =    this->declare_parameter("odom_frame_id", odomFrameId_);
    cameraFrameId_ =  this->declare_parameter("camera_frame_id", cameraFrameId_);
    scanFrameId_ =    this->declare_parameter("scan_frame_id", scanFrameId_);
    gtFrameId_ =      this->declare_parameter("ground_truth_frame_id", gtFrameId_);
    gtBaseFrameId_ =  this->declare_parameter("ground_truth_base_frame_id", gtBaseFrameId_);
    rate =            this->declare_parameter("rate", rate); // Ratio of the database stamps
    databasePath =    this->declare_parameter("database", databasePath);
    publishTf =       this->declare_parameter("publish_tf", publishTf);
    ignoreOdom =      this->declare_parameter("ignore_odom", ignoreOdom);
    startId =         this->declare_parameter("start_id", startId);
	qos_ =            this->declare_parameter("qos", qos_);

    // A general 360 lidar with 0.5 deg increment
	scanAngleMin_ =         this->declare_parameter("scan_angle_min", -M_PI);
    scanAngleMax_ =         this->declare_parameter("scan_angle_max", M_PI);
    scanAngleIncrement_ =   this->declare_parameter("scan_angle_increment", M_PI / 720.0);
    scanRangeMin_ =         this->declare_parameter("scan_range_min", 0.0);
    scanRangeMax_ =         this->declare_parameter("scan_range_max", 60);

    RCLCPP_INFO(get_logger(), "frame_id = %s", frameId_.c_str());
	RCLCPP_INFO(get_logger(), "odom_frame_id = %s", odomFrameId_.c_str());
	RCLCPP_INFO(get_logger(), "camera_frame_id = %s", cameraFrameId_.c_str());
	RCLCPP_INFO(get_logger(), "scan_frame_id = %s", scanFrameId_.c_str());
	RCLCPP_INFO(get_logger(), "ground_truth_frame_id = %s", gtFrameId_.c_str());
	RCLCPP_INFO(get_logger(), "rate (factor) = %f", rate);
	RCLCPP_INFO(get_logger(), "publish_tf = %s", publishTf?"true":"false");
	RCLCPP_INFO(get_logger(), "start_id = %d", startId);
	RCLCPP_INFO(get_logger(), "Publish clock (--clock): %s", publishClock?"true":"false");
    RCLCPP_INFO(get_logger(), "qos = %d", qos_);

    if(databasePath.empty())
	{
		RCLCPP_ERROR(get_logger(), "Parameter \"database\" must be set (path to a RTAB-Map database).");
		exit(-1);
	}

    databasePath = uReplaceChar(databasePath, '~', UDirectory::homeDir());
	if(databasePath.size() && databasePath.at(0) != '/')
	{
		databasePath = UDirectory::currentDir(true) + databasePath;
	}
	RCLCPP_INFO(get_logger(), "database = %s", databasePath.c_str());

	reader_.reset(new rtabmap::DBReader(databasePath, -rate, ignoreOdom, false, false, startId));
	if(!reader_->init())
	{
		RCLCPP_ERROR(get_logger(), "Cannot open database \"%s\".", databasePath.c_str());
		exit(-1);
	}

    const std::string servicePrefix = get_name() + std::string("/");
    pauseSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "pause", std::bind(&DbPlayer::pauseCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
	resumeSrv_ = this->create_service<std_srvs::srv::Empty>(servicePrefix + "resume", std::bind(&DbPlayer::resumeCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    if(publishTf) {
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    if(publishClock)
	{
		clockPub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
	}
}

DbPlayer::~DbPlayer(){}

void DbPlayer::pauseCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    if(paused_)
	{
		RCLCPP_WARN(get_logger(), "Already paused!");
	}
	else
	{
		paused_ = true;
		RCLCPP_INFO(get_logger(), "paused!");
	}
}
void DbPlayer::resumeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
{
    if(!paused_)
	{
		RCLCPP_WARN(get_logger(), "Already running!");
	}
	else
	{
		paused_ = false;
		RCLCPP_INFO(get_logger(), "resumed!");
	}
}

bool DbPlayer::publishNextFrame()
{
	rtabmap::SensorCaptureInfo cameraInfo;
	rtabmap::SensorData data = reader_->takeImage(&cameraInfo);
	rtabmap::OdometryInfo odomInfo;
	odomInfo.reg.covariance = cameraInfo.odomCovariance;
	rtabmap::OdometryEvent odom(data, cameraInfo.odomPose, odomInfo);
    if(!odom.data().id())
    {
        return false;
    }

    RCLCPP_INFO(get_logger(), "Reading sensor data %d...", odom.data().id());

    rclcpp::Time time = rtabmap_conversions::timestampToROS(odom.data().stamp());

    if(clockPub_.get())
    {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = time;
        clockPub_->publish(msg);
    }

    sensor_msgs::msg::CameraInfo camInfoA; //rgb or left
    sensor_msgs::msg::CameraInfo camInfoB; //depth or right

    camInfoA.k.fill(0);
    camInfoA.k[0] = camInfoA.k[4] = camInfoA.k[8] = 1;
    camInfoA.r.fill(0);
    camInfoA.r[0] = camInfoA.r[4] = camInfoA.r[8] = 1;
    camInfoA.p.fill(0);
    camInfoA.p[10] = 1;

    camInfoA.header.frame_id = cameraFrameId_;
    camInfoA.header.stamp = time;

    camInfoB = camInfoA;

    if(!odom.data().depthRaw().empty() && (odom.data().depthRaw().type() == CV_32FC1 || odom.data().depthRaw().type() == CV_16UC1))
    {
        if(odom.data().cameraModels().size() > 1)
        {
            RCLCPP_WARN(get_logger(), "Multi-cameras detected in database but this node cannot send multi-images yet...");
        }
        else
        {
            //depth
            if(odom.data().cameraModels().size())
            {
                camInfoA.d.resize(5,0);

                camInfoA.p[0] = odom.data().cameraModels()[0].fx();
                camInfoA.k[0] = odom.data().cameraModels()[0].fx();
                camInfoA.p[5] = odom.data().cameraModels()[0].fy();
                camInfoA.k[4] = odom.data().cameraModels()[0].fy();
                camInfoA.p[2] = odom.data().cameraModels()[0].cx();
                camInfoA.k[2] = odom.data().cameraModels()[0].cx();
                camInfoA.p[6] = odom.data().cameraModels()[0].cy();
                camInfoA.k[5] = odom.data().cameraModels()[0].cy();

                camInfoB = camInfoA;
            }

            if(rgbPub_.getTopic().empty()) rgbPub_ = image_transport::create_publisher(this, "rgb/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
            if(depthPub_.getTopic().empty()) depthPub_ = image_transport::create_publisher(this, "depth/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
            if(!rgbInfoPub_.get()) rgbInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", 1);
            if(!depthInfoPub_.get()) depthInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 1);
        }
    }
    else if(!odom.data().rightRaw().empty() && odom.data().rightRaw().type() == CV_8U)
    {
        if(odom.data().stereoCameraModels().size() > 1)
        {
            RCLCPP_WARN(get_logger(), "Multi-cameras detected in database but this node cannot send multi-images yet...");
        }
        else
        {
            //stereo
            if(odom.data().stereoCameraModels()[0].isValidForProjection())
            {
                camInfoA.d.resize(8,0);

                camInfoA.p[0] = odom.data().stereoCameraModels()[0].left().fx();
                camInfoA.k[0] = odom.data().stereoCameraModels()[0].left().fx();
                camInfoA.p[5] = odom.data().stereoCameraModels()[0].left().fy();
                camInfoA.k[4] = odom.data().stereoCameraModels()[0].left().fy();
                camInfoA.p[2] = odom.data().stereoCameraModels()[0].left().cx();
                camInfoA.k[2] = odom.data().stereoCameraModels()[0].left().cx();
                camInfoA.p[6] = odom.data().stereoCameraModels()[0].left().cy();
                camInfoA.k[5] = odom.data().stereoCameraModels()[0].left().cy();

                camInfoB = camInfoA;
                camInfoB.p[3] = odom.data().stereoCameraModels()[0].right().Tx(); // Right_Tx = -baseline*fx
            }

            if(leftPub_.getTopic().empty()) leftPub_ = image_transport::create_publisher(this, "left/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
            if(rightPub_.getTopic().empty()) rightPub_ = image_transport::create_publisher(this, "right/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
            if(!leftInfoPub_.get()) leftInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
            if(!rightInfoPub_.get()) rightInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);
        }

    }
    else
    {
        if(imagePub_.getTopic().empty()) imagePub_ = image_transport::create_publisher(this, "image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
    }

    camInfoA.height = odom.data().imageRaw().rows;
    camInfoA.width = odom.data().imageRaw().cols;
    camInfoB.height = odom.data().depthOrRightRaw().rows;
    camInfoB.width = odom.data().depthOrRightRaw().cols;

    if(!odom.data().laserScanRaw().isEmpty())
    {
        if(!scanPub_.get() && odom.data().laserScanRaw().is2d())
        {
            scanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
            if(odom.data().laserScanRaw().angleIncrement() > 0.0f)
            {
                RCLCPP_INFO(get_logger(), "Scan will be published.");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Scan will be published with those parameters:");
                RCLCPP_INFO(get_logger(), "  scan_angle_min=%f", scanAngleMin_);
                RCLCPP_INFO(get_logger(), "  scan_angle_max=%f", scanAngleMax_);
                RCLCPP_INFO(get_logger(), "  scan_angle_increment=%f", scanAngleIncrement_);
                RCLCPP_INFO(get_logger(), "  scan_range_min=%f", scanRangeMin_);
                RCLCPP_INFO(get_logger(), "  scan_range_max=%f", scanRangeMax_);
            }
        }
        else if(!scanCloudPub_.get())
        {
            scanCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 1);
            RCLCPP_INFO(get_logger(), "Scan cloud will be published.");
        }
    }

    if(!odom.data().globalPose().isNull() &&
        odom.data().globalPoseCovariance().cols==6 &&
        odom.data().globalPoseCovariance().rows==6)
    {
        if(!globalPosePub_.get())
        {
            globalPosePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose", 1);
            RCLCPP_INFO(get_logger(), "Global pose will be published.");
        }
    }

    if(odom.data().gps().stamp() > 0.0)
    {
        if(!gpsFixPub_.get())
        {
            gpsFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 1);
            RCLCPP_INFO(get_logger(), "GPS will be published.");
        }
    }

    // publish transforms first
    if(tfBroadcaster_.get())
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
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        if(!localTransform.isNull())
        {
            geometry_msgs::msg::TransformStamped baseToCamera;
            baseToCamera.child_frame_id = cameraFrameId_;
            baseToCamera.header.frame_id = frameId_;
            baseToCamera.header.stamp = time;
            rtabmap_conversions::transformToGeometryMsg(localTransform, baseToCamera.transform);
            transforms.push_back(baseToCamera);
        }

        if(!odom.pose().isNull())
        {
            geometry_msgs::msg::TransformStamped odomToBase;
            odomToBase.child_frame_id = frameId_;
            odomToBase.header.frame_id = odomFrameId_;
            odomToBase.header.stamp = time;
            rtabmap_conversions::transformToGeometryMsg(odom.pose(), odomToBase.transform);
            transforms.push_back(odomToBase);
        }

        if(scanPub_.get() || scanCloudPub_.get())
        {
            geometry_msgs::msg::TransformStamped baseToLaserScan;
            baseToLaserScan.child_frame_id = scanFrameId_;
            baseToLaserScan.header.frame_id = frameId_;
            baseToLaserScan.header.stamp = time;
            rtabmap_conversions::transformToGeometryMsg(odom.data().laserScanCompressed().localTransform(), baseToLaserScan.transform);
            transforms.push_back(baseToLaserScan);
        }

        if(!odom.data().groundTruth().isNull()) {
            geometry_msgs::msg::TransformStamped worldToBase;
            worldToBase.child_frame_id = gtBaseFrameId_;
            worldToBase.header.frame_id = gtFrameId_;
            worldToBase.header.stamp = time;
            rtabmap_conversions::transformToGeometryMsg(odom.data().groundTruth(), worldToBase.transform);
            transforms.push_back(worldToBase);
        }
        tfBroadcaster_->sendTransform(transforms);
    }

    if(!odom.pose().isNull())
    {
        if(!odometryPub_.get()) odometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

        if(odometryPub_->get_subscription_count())
        {
            nav_msgs::msg::Odometry odomMsg;
            odomMsg.child_frame_id = frameId_;
            odomMsg.header.frame_id = odomFrameId_;
            odomMsg.header.stamp = time;
            rtabmap_conversions::transformToPoseMsg(odom.pose(), odomMsg.pose.pose);
            UASSERT(odomMsg.pose.covariance.size() == 36 &&
                    odom.covariance().total() == 36 &&
                    odom.covariance().type() == CV_64FC1);
            memcpy(odomMsg.pose.covariance.begin(), odom.covariance().data, 36*sizeof(double));
            odometryPub_->publish(odomMsg);
        }
    }

    // Publish async topics first (so that they can catched by rtabmap before the image topics)
    if( globalPosePub_.get() && 
        globalPosePub_->get_subscription_count() > 0 &&
        !odom.data().globalPose().isNull() &&
        odom.data().globalPoseCovariance().cols==6 &&
        odom.data().globalPoseCovariance().rows==6)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        rtabmap_conversions::transformToPoseMsg(odom.data().globalPose(), msg.pose.pose);
        memcpy(msg.pose.covariance.data(), odom.data().globalPoseCovariance().data, 36*sizeof(double));
        msg.header.frame_id = frameId_;
        msg.header.stamp = time;
        globalPosePub_->publish(msg);
    }

    if( gpsFixPub_.get() &&
        gpsFixPub_->get_subscription_count() > 0 &&
        odom.data().gps().stamp() > 0.0)
    {
        sensor_msgs::msg::NavSatFix msg;
        msg.longitude = odom.data().gps().longitude();
        msg.latitude = odom.data().gps().latitude();
        msg.altitude = odom.data().gps().altitude();
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        msg.position_covariance.at(0) = msg.position_covariance.at(4) = msg.position_covariance.at(8)= odom.data().gps().error()* odom.data().gps().error();
        msg.header.frame_id = frameId_;
        msg.header.stamp = rtabmap_conversions::timestampToROS(odom.data().gps().stamp());
        gpsFixPub_->publish(msg);
    }

    if( (imagePub_.getNumSubscribers()) || 
        (rgbPub_.getNumSubscribers()) || 
        (leftPub_.getNumSubscribers()))
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
        sensor_msgs::msg::Image imageRosMsg;
        img.toImageMsg(imageRosMsg);
        imageRosMsg.header.frame_id = cameraFrameId_;
        imageRosMsg.header.stamp = time;

        if(imagePub_.getNumSubscribers())
        {
            imagePub_.publish(imageRosMsg);
        }
        if(rgbPub_.getNumSubscribers())
        {
            rgbPub_.publish(imageRosMsg);
            rgbInfoPub_->publish(camInfoA);
        }
        if(leftPub_.getNumSubscribers())
        {
            leftPub_.publish(imageRosMsg);
            leftInfoPub_->publish(camInfoA);
        }
    }

    if(depthPub_.getNumSubscribers() && !odom.data().depthRaw().empty())
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
        sensor_msgs::msg::Image imageRosMsg;
        img.toImageMsg(imageRosMsg);
        imageRosMsg.header.frame_id = cameraFrameId_;
        imageRosMsg.header.stamp = time;

        depthPub_.publish(imageRosMsg);
        depthInfoPub_->publish(camInfoB);
    }

    if(rightPub_.getNumSubscribers() && !odom.data().rightRaw().empty())
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
        img.image = odom.data().rightRaw();
        sensor_msgs::msg::Image imageRosMsg;
        img.toImageMsg(imageRosMsg);
        imageRosMsg.header.frame_id = cameraFrameId_;
        imageRosMsg.header.stamp = time;

        rightPub_.publish(imageRosMsg);
        rightInfoPub_->publish(camInfoB);
    }

    if(!odom.data().laserScanRaw().isEmpty())
    {
        if(scanPub_.get() && scanPub_->get_subscription_count() && odom.data().laserScanRaw().is2d())
        {
            //inspired from pointcloud_to_laserscan package
            sensor_msgs::msg::LaserScan msg;
            msg.header.frame_id = scanFrameId_;
            msg.header.stamp = time;

            msg.angle_min = scanAngleMin_;
            msg.angle_max = scanAngleMax_;
            msg.angle_increment = scanAngleIncrement_;
            msg.time_increment = 0.0;
            msg.scan_time = 0;
            msg.range_min = scanRangeMin_;
            msg.range_max = scanRangeMax_;
            if(odom.data().laserScanRaw().angleIncrement() > 0.0f)
            {
                msg.angle_min = odom.data().laserScanRaw().angleMin();
                msg.angle_max = odom.data().laserScanRaw().angleMax();
                msg.angle_increment = odom.data().laserScanRaw().angleIncrement();
                msg.range_min = odom.data().laserScanRaw().rangeMin();
                msg.range_max = odom.data().laserScanRaw().rangeMax();
            }

            int rangesSize = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);
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

            scanPub_->publish(msg);
        }
        else if(scanCloudPub_.get() && scanCloudPub_->get_subscription_count())
        {
            sensor_msgs::msg::PointCloud2 msg;
            pcl_conversions::moveFromPCL(*rtabmap::util3d::laserScanToPointCloud2(odom.data().laserScanRaw()), msg);
            msg.header.frame_id = scanFrameId_;
            msg.header.stamp = time;
            scanCloudPub_->publish(msg);
        }
    }
    return true;
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::DbPlayer)
