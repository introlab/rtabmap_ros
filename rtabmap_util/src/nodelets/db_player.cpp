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
    imuFrameId_("imu_link"),
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
    imuFrameId_ =     this->declare_parameter("imu_frame_id", imuFrameId_);
    rate =            this->declare_parameter("rate", rate); // Ratio of the database stamps
    databasePath =    this->declare_parameter("database", databasePath);
    publishTf =       this->declare_parameter("publish_tf", publishTf);
    ignoreOdom =      this->declare_parameter("ignore_odom", ignoreOdom);
    startId =         this->declare_parameter("start_id", startId);
    qos_ =            this->declare_parameter("qos", qos_);
    qosCameraInfo_ =  this->declare_parameter("qos_camera_info", qos_);
    qosOdom_ =        this->declare_parameter("qos_odom", qos_);
    qosScan_ =        this->declare_parameter("qos_scan", qos_);
    qosScanCloud_ =   this->declare_parameter("qos_scan_cloud", qos_);
    qosGlobalPose_ =  this->declare_parameter("qos_global_pose", qos_);
    qosGps_ =         this->declare_parameter("qos_gps", qos_);
    qosImu_ =         this->declare_parameter("qos_imu", qos_);

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
    RCLCPP_INFO(get_logger(), "imu_frame_id = %s", imuFrameId_.c_str());
    RCLCPP_INFO(get_logger(), "rate (factor) = %f", rate);
    RCLCPP_INFO(get_logger(), "publish_tf = %s", publishTf?"true":"false");
    RCLCPP_INFO(get_logger(), "start_id = %d", startId);
    RCLCPP_INFO(get_logger(), "Publish clock (--clock): %s", publishClock?"true":"false");
    RCLCPP_INFO(get_logger(), "qos = %d", qos_);
    RCLCPP_INFO(get_logger(), "  qos_camera_info = %d", qosCameraInfo_);
    RCLCPP_INFO(get_logger(), "  qos_odom = %d", qosOdom_);
    RCLCPP_INFO(get_logger(), "  qos_scan = %d", qosScan_);
    RCLCPP_INFO(get_logger(), "  qos_scan_cloud = %d", qosScanCloud_);
    RCLCPP_INFO(get_logger(), "  qos_global_pose = %d", qosGlobalPose_);
    RCLCPP_INFO(get_logger(), "  qos_gps = %d", qosGps_);
    RCLCPP_INFO(get_logger(), "  qos_imu = %d", qosImu_);

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

void DbPlayer::initializePublishers(const rtabmap::OdometryEvent & odom)
{
    if(!odom.data().depthRaw().empty() && (odom.data().depthRaw().type() == CV_32FC1 || odom.data().depthRaw().type() == CV_16UC1))
    {
        if(odom.data().cameraModels().size() > 1)
        {
            if(rgbdImagePubs_.empty()) {
                 for(size_t i=0;i<odom.data().cameraModels().size(); ++i) {
                    rgbdImagePubs_.push_back(this->create_publisher<rtabmap_msgs::msg::RGBDImage>(uFormat("rgbd_image%ld", i), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_)));
                RCLCPP_INFO(get_logger(), "RGB-D image          \"%s\" will be published.", rgbdImagePubs_.back()->get_topic_name());
                }
            }
            else {
                UASSERT_MSG(rgbdImagePubs_.size() == odom.data().cameraModels().size(), uFormat("%ld versus %ld", rgbdImagePubs_.size(), odom.data().cameraModels().size()).c_str());
            }
        }
        else
        {
            if(rgbPub_.getTopic().empty()) {
                rgbPub_ = image_transport::create_publisher(this, "rgb/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
                RCLCPP_INFO(get_logger(), "Gray/RGB image       \"%s\" will be published.", rgbPub_.getTopic().c_str());
            }
            if(!rgbInfoPub_.get()) {
                rgbInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("rgb/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCameraInfo_));
                RCLCPP_INFO(get_logger(), "Gray/RGB calibration \"%s\" will be published.", rgbInfoPub_->get_topic_name());
            }
            if(depthPub_.getTopic().empty()) {
                depthPub_ = image_transport::create_publisher(this, "depth/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
                RCLCPP_INFO(get_logger(), "Depth image          \"%s\" will be published.", depthPub_.getTopic().c_str());
            }
            if(!depthInfoPub_.get()) {
                depthInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCameraInfo_));
                RCLCPP_INFO(get_logger(), "Depth calibration    \"%s\" will be published.", depthInfoPub_->get_topic_name());
            }
        }
    }
    else if(!odom.data().rightRaw().empty() && (odom.data().rightRaw().type() == CV_8U || odom.data().rightRaw().type() == CV_8UC3))
    {
        if(odom.data().stereoCameraModels().size() > 1)
        {
            if(rgbdImagePubs_.empty()) {
                 for(size_t i=0;i<odom.data().stereoCameraModels().size(); ++i) {
                    rgbdImagePubs_.push_back(this->create_publisher<rtabmap_msgs::msg::RGBDImage>(uFormat("stereo_image%ld", i), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_)));
                 RCLCPP_INFO(get_logger(), "Stereo image        \"%s\" will be published.", rgbdImagePubs_.back()->get_topic_name());
                }
            }
            else {
                UASSERT_MSG(rgbdImagePubs_.size() == odom.data().stereoCameraModels().size(), uFormat("%ld versus %ld", rgbdImagePubs_.size(), odom.data().stereoCameraModels().size()).c_str());
            }
        }
        else
        {
            if(leftPub_.getTopic().empty()) {
                leftPub_ = image_transport::create_publisher(this, "left/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
                RCLCPP_INFO(get_logger(), "Left image           \"%s\" will be published.", leftPub_.getTopic().c_str());
            }
            if(!leftInfoPub_.get()) {
                leftInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCameraInfo_));
                RCLCPP_INFO(get_logger(), "Left calibration     \"%s\" will be published.", leftInfoPub_->get_topic_name());
            }
            if(rightPub_.getTopic().empty()) {
                rightPub_ = image_transport::create_publisher(this, "right/image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
                RCLCPP_INFO(get_logger(), "Right image          \"%s\" will be published.", rightPub_.getTopic().c_str());
            }
            if(!rightInfoPub_.get()) {
                rightInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosCameraInfo_));
                RCLCPP_INFO(get_logger(), "Right calibration    \"%s\" will be published.", rightInfoPub_->get_topic_name());
            }
        }

    }
    else if(imagePub_.getTopic().empty())
    {
        imagePub_ = image_transport::create_publisher(this, "image", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos_).get_rmw_qos_profile());
                RCLCPP_INFO(get_logger(), "Image                \"%s\" without calibration will be published.", imagePub_.getTopic().c_str());
    }

    if(!odom.data().laserScanRaw().isEmpty())
    {
        if(!scanPub_.get() && odom.data().laserScanRaw().is2d())
        {
            scanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosScan_));
            if(odom.data().laserScanRaw().angleIncrement() > 0.0f)
            {
                RCLCPP_INFO(get_logger(), "LaserScan            \"%s\" will be published.", scanPub_->get_topic_name());
            }
            else
            {
                RCLCPP_INFO(get_logger(), "LaserScan            \"%s\" will be published with those parameters:", scanPub_->get_topic_name());
                RCLCPP_INFO(get_logger(), "  scan_angle_min=%f", scanAngleMin_);
                RCLCPP_INFO(get_logger(), "  scan_angle_max=%f", scanAngleMax_);
                RCLCPP_INFO(get_logger(), "  scan_angle_increment=%f", scanAngleIncrement_);
                RCLCPP_INFO(get_logger(), "  scan_range_min=%f", scanRangeMin_);
                RCLCPP_INFO(get_logger(), "  scan_range_max=%f", scanRangeMax_);
            }
        }
        else if(!scanCloudPub_.get())
        {
            scanCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosScanCloud_));
                RCLCPP_INFO(get_logger(), "PointCloud2          \"%s\" will be published.", scanCloudPub_->get_topic_name());
        }
    }

    if(!odom.data().globalPose().isNull() &&
        odom.data().globalPoseCovariance().cols==6 &&
        odom.data().globalPoseCovariance().rows==6)
    {
        if(!globalPosePub_.get())
        {
            globalPosePub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("global_pose", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosGlobalPose_));
                RCLCPP_INFO(get_logger(), "Global pose          \"%s\" will be published.", globalPosePub_->get_topic_name());
        }
    }

    if(!gpsFixPub_.get() && odom.data().gps().stamp() > 0.0)
    {
        gpsFixPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosGps_));
                RCLCPP_INFO(get_logger(), "GPS                  \"%s\" will be published.", gpsFixPub_->get_topic_name());
    }

    if(!odometryPub_.get() && !odom.pose().isNull())
    {
        odometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosOdom_));
                RCLCPP_INFO(get_logger(), "Odometry             \"%s\" will be published.", odometryPub_->get_topic_name());
    }
    if(!imuPub_.get() && !odom.data().imu().empty())
    {
        imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosImu_));
                RCLCPP_INFO(get_logger(), "IMU                  \"%s\" will be published.", imuPub_->get_topic_name());
    }
}

bool DbPlayer::cvImageToROS(const cv::Mat & image, sensor_msgs::msg::Image & rosImage)
{
    cv_bridge::CvImage img;
    if(image.type() == CV_32FC1)
    {
        img.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }
    else if(image.type() == CV_16UC1)
    {
        img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    }
    else if(image.type() == CV_8UC1)
    {
        img.encoding = sensor_msgs::image_encodings::MONO8;
    }
    else if(image.type() == CV_8UC3)
    {
        img.encoding = sensor_msgs::image_encodings::BGR8;
    }
    else {
        RCLCPP_ERROR(get_logger(), "Unsupported image format: cv type = %d", image.type());
        return false;
    }
    img.image = image;
    img.toImageMsg(rosImage);
    return true;
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

    ///////////////////////
    // Initialize publishers based on data in the database
    ///////////////////////
    initializePublishers(odom); // called everytime in case some data like global pose, imu, odometry was not available at the beggining.
    

    ///////////////////////
    // Publish topics
    ///////////////////////

    if(clockPub_.get())
    {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = time;
        clockPub_->publish(msg);
    }

    // publish transforms first
    if(tfBroadcaster_.get())
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        const std::vector<rtabmap::CameraModel> * models = &odom.data().cameraModels();
        std::vector<rtabmap::CameraModel> stereoModels;
        bool stereo = false;
        if(odom.data().stereoCameraModels().size())
        {
            for(const auto & cam: odom.data().stereoCameraModels()) {
                stereoModels.push_back(cam.left());
                stereoModels.push_back(cam.right());
            }
            models = &stereoModels;
            stereo = true;
        }
        int index = 0;
        for(const auto & cam: *models) {
            rtabmap::Transform localTransform = cam.localTransform();
            if(!localTransform.isNull()) {
                geometry_msgs::msg::TransformStamped baseToCamera;
                baseToCamera.child_frame_id = (stereo?index%2==0?"left_":"right_":"") + cameraFrameId_ + (models->size()>1?uNumber2Str(index/(stereo?2:1)):"");
                baseToCamera.header.frame_id = frameId_;
                baseToCamera.header.stamp = time;
                if(cam.Tx() != 0) {
                    localTransform *= rtabmap::Transform(-cam.Tx()/cam.fx(), 0, 0);
                }
                rtabmap_conversions::transformToGeometryMsg(localTransform, baseToCamera.transform);
                transforms.push_back(baseToCamera);
            }
            ++index;
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

        if(!odom.data().imu().empty()) {
            geometry_msgs::msg::TransformStamped baseToImu;
            baseToImu.child_frame_id = imuFrameId_;
            baseToImu.header.frame_id = frameId_;
            baseToImu.header.stamp = time;
            rtabmap_conversions::transformToGeometryMsg(odom.data().imu().localTransform(), baseToImu.transform);
            transforms.push_back(baseToImu);
        }
        tfBroadcaster_->sendTransform(transforms);
    }

    if( odometryPub_.get() && 
        !odom.pose().isNull() && 
        odometryPub_->get_subscription_count())
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

    if( imuPub_.get() &&
        imuPub_->get_subscription_count() > 0 &&
        !odom.data().imu().empty())
    {
        sensor_msgs::msg::Imu msg;
        rtabmap_conversions::imuToROS(odom.data().imu(), msg);
        msg.header.frame_id = imuFrameId_;
        msg.header.stamp = time;
        imuPub_->publish(msg);
    }

    // single camera
    if(imagePub_.getNumSubscribers() || (odom.data().cameraModels().size() <= 1 && odom.data().stereoCameraModels().size() <= 1))
    {
        if(!odom.data().imageRaw().empty() &&
            ((imagePub_.getNumSubscribers()) || 
            (rgbPub_.getNumSubscribers()) || 
            (leftPub_.getNumSubscribers())))
        {
            sensor_msgs::msg::Image imageRosMsg;
            if(cvImageToROS(odom.data().imageRaw(), imageRosMsg))
            {
                imageRosMsg.header.frame_id = cameraFrameId_;
                imageRosMsg.header.stamp = time;

                if(imagePub_.getNumSubscribers())
                {
                    imagePub_.publish(imageRosMsg);
                }
                if(rgbPub_.getNumSubscribers())
                {
                    rgbPub_.publish(imageRosMsg);
                    UASSERT(odom.data().cameraModels().size() == 1);
                    sensor_msgs::msg::CameraInfo info;
                    rtabmap_conversions::cameraModelToROS(odom.data().cameraModels()[0], info);
                    info.header = imageRosMsg.header;
                    rgbInfoPub_->publish(info);
                }
                if(leftPub_.getNumSubscribers())
                {
                    imageRosMsg.header.frame_id = "left_" + cameraFrameId_;
                    leftPub_.publish(imageRosMsg);
                    UASSERT(odom.data().stereoCameraModels().size() == 1);
                    sensor_msgs::msg::CameraInfo info;
                    rtabmap_conversions::cameraModelToROS(odom.data().stereoCameraModels()[0].left(), info);
                    info.header = imageRosMsg.header;
                    leftInfoPub_->publish(info);
                }
            }
        }

        if(!odom.data().depthRaw().empty() && depthPub_.getNumSubscribers())
        {
            sensor_msgs::msg::Image imageRosMsg;
            if(cvImageToROS(odom.data().depthRaw(), imageRosMsg))
            {
                imageRosMsg.header.frame_id = cameraFrameId_;
                imageRosMsg.header.stamp = time;

                depthPub_.publish(imageRosMsg);

                UASSERT(odom.data().cameraModels().size() == 1);
                sensor_msgs::msg::CameraInfo info;
                // We assume depth is registered with the RGB camera, so they share same calibration and TF frame
                rtabmap_conversions::cameraModelToROS(odom.data().cameraModels()[0], info);
                info.header = imageRosMsg.header;
                depthInfoPub_->publish(info);
            }
        }

        if(!odom.data().rightRaw().empty() && rightPub_.getNumSubscribers())
        {
            sensor_msgs::msg::Image imageRosMsg;
            if(cvImageToROS(odom.data().rightRaw(), imageRosMsg))
            {
                imageRosMsg.header.frame_id = "right_" + cameraFrameId_;
                imageRosMsg.header.stamp = time;

                rightPub_.publish(imageRosMsg);

                UASSERT(odom.data().stereoCameraModels().size() == 1);
                sensor_msgs::msg::CameraInfo info;
                rtabmap_conversions::cameraModelToROS(odom.data().stereoCameraModels()[0].right(), info);
                info.header = imageRosMsg.header;
                rightInfoPub_->publish(info);
            }
        }
    }

    // Multi-cameras
    if(!odom.data().imageRaw().empty())
    {
        std::vector<rtabmap_msgs::msg::RGBDImage> rgbdImages;
        if(odom.data().cameraModels().size() > 1)
        {
            UASSERT(odom.data().cameraModels().size() == rgbdImagePubs_.size());
            int subRgbImageWidth = odom.data().imageRaw().cols / odom.data().cameraModels().size();
            int subDepthImageWidth = odom.data().depthRaw().cols / odom.data().cameraModels().size();
            for(size_t i=0; i<rgbdImagePubs_.size(); ++i)
            {
                rtabmap_msgs::msg::RGBDImage msg;
                msg.header.stamp = time;
                msg.header.frame_id = cameraFrameId_ + uNumber2Str((int)i);

                cvImageToROS(cv::Mat(odom.data().imageRaw(), cv::Range::all(), cv::Range(i*subRgbImageWidth, (i+1)*subRgbImageWidth)), msg.rgb);     
                msg.rgb.header = msg.header;
                rtabmap_conversions::cameraModelToROS(odom.data().cameraModels()[i], msg.rgb_camera_info);
                msg.rgb_camera_info.header = msg.header;

                if(subDepthImageWidth) {
                    cvImageToROS(cv::Mat(odom.data().depthRaw(), cv::Range::all(), cv::Range(i*subDepthImageWidth, (i+1)*subDepthImageWidth)), msg.depth);     
                    msg.depth.header = msg.header;
                    UASSERT(subDepthImageWidth <= subRgbImageWidth);
                    if(subDepthImageWidth < subRgbImageWidth) {
                        rtabmap_conversions::cameraModelToROS(odom.data().cameraModels()[i].scaled(double(subDepthImageWidth)/double(subRgbImageWidth)), msg.depth_camera_info);
                    }
                    else {
                        rtabmap_conversions::cameraModelToROS(odom.data().cameraModels()[i], msg.depth_camera_info);
                    }
                    msg.depth_camera_info.header = msg.header;
                }

                rgbdImagePubs_[i]->publish(msg);
            }
        }
        else if(odom.data().stereoCameraModels().size() > 1)
        {
            UASSERT(odom.data().stereoCameraModels().size() == rgbdImagePubs_.size());
            int subImageWidth = odom.data().imageRaw().cols / odom.data().stereoCameraModels().size();
            UASSERT(odom.data().imageRaw().cols == odom.data().rightRaw().cols);
            for(size_t i=0; i<rgbdImagePubs_.size(); ++i)
            {
                rtabmap_msgs::msg::RGBDImage msg;
                msg.header.stamp = time;
                msg.header.frame_id = "left_" + cameraFrameId_ + uNumber2Str((int)i);

                cvImageToROS(cv::Mat(odom.data().imageRaw(), cv::Range::all(), cv::Range(i*subImageWidth, (i+1)*subImageWidth)), msg.rgb);     
                msg.rgb.header = msg.header;
                rtabmap_conversions::cameraModelToROS(odom.data().stereoCameraModels()[i].left(), msg.rgb_camera_info);
                msg.rgb_camera_info.header = msg.header;

                std::string rightFrame = "right_" + cameraFrameId_ + uNumber2Str((int)i);
                cvImageToROS(cv::Mat(odom.data().rightRaw(), cv::Range::all(), cv::Range(i*subImageWidth, (i+1)*subImageWidth)), msg.depth);     
                msg.depth.header = msg.header;
                msg.depth.header.frame_id = rightFrame;
                rtabmap_conversions::cameraModelToROS(odom.data().stereoCameraModels()[i].right(), msg.depth_camera_info);
                msg.depth_camera_info.header = msg.depth.header;

                rgbdImagePubs_[i]->publish(msg);
            }
        }
    }

    if(!odom.data().laserScanRaw().isEmpty())
    {
        if(scanPub_.get() && 
           scanPub_->get_subscription_count() && 
           odom.data().laserScanRaw().is2d())
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
        else if(scanCloudPub_.get() && 
                scanCloudPub_->get_subscription_count())
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
