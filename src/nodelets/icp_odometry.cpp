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

#include <rtabmap_ros/icp_odometry.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "rtabmap_ros/MsgConversion.h"

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

using namespace rtabmap;

namespace rtabmap_ros
{

ICPOdometry::ICPOdometry(const rclcpp::NodeOptions & options) :
	OdometryROS("icp_odometry", options),
	scanCloudMaxPoints_(0),
	scanDownsamplingStep_(1),
	scanRangeMin_(0),
	scanRangeMax_(0),
	scanVoxelSize_(0.0),
	scanNormalK_(0),
	scanNormalRadius_(0.0),
	scanNormalGroundUp_(0.0),
	//plugin_loader_("rtabmap_ros", "rtabmap_ros::PluginInterface"),
	scanReceived_(false),
	cloudReceived_(false)
{
	OdometryROS::init(false, false, true);
}

ICPOdometry::~ICPOdometry()
{
	//plugins_.clear();
}

void ICPOdometry::onOdomInit()
{
	int queueSize = 1;
	queueSize = this->declare_parameter("queue_size", queueSize);
	scanCloudMaxPoints_ = this->declare_parameter("scan_cloud_max_points", scanCloudMaxPoints_);
	scanDownsamplingStep_ = this->declare_parameter("scan_downsampling_step", scanDownsamplingStep_);
	scanRangeMin_ = this->declare_parameter("scan_range_min", scanRangeMin_);
	scanRangeMax_ = this->declare_parameter("scan_range_max", scanRangeMax_);
	scanVoxelSize_ = this->declare_parameter("scan_voxel_size", scanVoxelSize_);
	scanNormalK_ = this->declare_parameter("scan_normal_k", scanNormalK_);
	scanNormalRadius_ = this->declare_parameter("scan_normal_radius", scanNormalRadius_);
	scanNormalGroundUp_ = this->declare_parameter("scan_normal_ground_up", scanNormalGroundUp_);

	/*if (pnh.hasParam("plugins"))
	{
		XmlRpc::XmlRpcValue pluginsList;
		pnh.getParam("plugins", pluginsList);

		for (int32_t i = 0; i < pluginsList.size(); ++i)
		{
			std::string pluginName = static_cast<std::string>(pluginsList[i]["name"]);
			std::string type = static_cast<std::string>(pluginsList[i]["type"]);
			RCLCPP_INFO(this->get_logger(), "IcpOdometry: Using plugin %s of type \"%s\"", pluginName.c_str(), type.c_str());
			try {
				boost::shared_ptr<rtabmap_ros::PluginInterface> plugin = plugin_loader_.createInstance(type);
				plugins_.push_back(plugin);
				plugin->initialize(pluginName, pnh);
				if(!plugin->isEnabled())
				{
					RCLCPP_WARN(this->get_logger(), "Plugin: %s is not enabled, filtering will not occur. \"enabled_\" member "
								 "should be managed in subclasses. This can be ignored if the "
								 "plugin should really be initialized as disabled.",
								 plugin->getName().c_str());
				}
			}
			catch(pluginlib::PluginlibException & ex) {
				RCLCPP_ERROR(this->get_logger(), "Failed to load plugin %s. Error: %s", pluginName.c_str(), ex.what());
			}

		}
	}*/

	RCLCPP_INFO(this->get_logger(), "IcpOdometry: queue_size             = %d", queueSize);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: qos                    = %d", (int)qos());
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_cloud_max_points  = %d", scanCloudMaxPoints_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_downsampling_step = %d", scanDownsamplingStep_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_range_min         = %f m", scanRangeMin_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_range_max         = %f m", scanRangeMax_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_voxel_size        = %f m", scanVoxelSize_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_k          = %d", scanNormalK_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_radius     = %f m", scanNormalRadius_);
	RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_ground_up  = %f", scanNormalGroundUp_);

	scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos()), std::bind(&ICPOdometry::callbackScan, this, std::placeholders::_1));
	cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("scan_cloud", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos()), std::bind(&ICPOdometry::callbackCloud, this, std::placeholders::_1));

	filtered_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("odom_filtered_input_scan", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos()));
}

void ICPOdometry::updateParameters(ParametersMap & parameters)
{
	//make sure we are using Reg/Strategy=0
	ParametersMap::iterator iter = parameters.find(Parameters::kRegStrategy());
	if(iter != parameters.end() && iter->second.compare("1") != 0)
	{
		RCLCPP_WARN(this->get_logger(), "ICP odometry works only with \"Reg/Strategy\"=1. Ignoring value %s.", iter->second.c_str());
	}
	uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "1"));

	iter = parameters.find(Parameters::kIcpDownsamplingStep());
	if(iter != parameters.end())
	{
		int value = uStr2Int(iter->second);
		if(value > 1)
		{
			if(!this->has_parameter("scan_downsampling_step"))
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_downsampling_step\" for convenience. \"%s\" is set to 1.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
				scanDownsamplingStep_ = value;
				iter->second = "1";
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_downsampling_step\" are set.", iter->first.c_str());
			}
		}
	}
	iter = parameters.find(Parameters::kIcpRangeMin());
	if(iter != parameters.end())
	{
		float value = uStr2Float(iter->second);
		if(value != 0.0f)
		{
			if(!this->has_parameter("scan_range_min"))
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_range_min\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
				scanRangeMin_ = value;
				iter->second = "0";
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_range_min\" are set.", iter->first.c_str());
			}
		}
	}
	iter = parameters.find(Parameters::kIcpRangeMax());
	if(iter != parameters.end())
	{
		float value = uStr2Float(iter->second);
		if(value != 0.0f)
		{
			if(!this->has_parameter("scan_range_max"))
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_range_max\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
				scanRangeMax_ = value;
				iter->second = "0";
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_range_max\" are set.", iter->first.c_str());
			}
		}
	}
	iter = parameters.find(Parameters::kIcpVoxelSize());
	if(iter != parameters.end())
	{
		float value = uStr2Float(iter->second);
		if(value != 0.0f)
		{
			if(!this->has_parameter("scan_voxel_size"))
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_voxel_size\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
				scanVoxelSize_ = value;
				iter->second = "0";
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_voxel_size\" are set.", iter->first.c_str());
			}
		}
	}
	else if(this->has_parameter("scan_voxel_size"))
	{
		RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_voxel_size is set (%f), setting %s to 0", scanVoxelSize_, Parameters::kIcpVoxelSize().c_str());
		parameters.insert(ParametersPair(Parameters::kIcpVoxelSize(), "0"));
	}
	iter = parameters.find(Parameters::kIcpPointToPlaneK());
	if(iter != parameters.end())
	{
		int value = uStr2Int(iter->second);
		if(value != 0)
		{
			if(!this->has_parameter("scan_normal_k"))
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_normal_k\" for convenience.", iter->second.c_str(), iter->first.c_str());
				scanNormalK_ = value;
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_k is set (%d), setting %s to same value.", scanNormalK_, Parameters::kIcpPointToPlaneK().c_str());
				iter->second = uNumber2Str(scanNormalK_);
			}
		}
	}
	else if(this->has_parameter("scan_normal_k"))
	{
		RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_k is set (%d), setting %s to same value.", scanNormalK_, Parameters::kIcpPointToPlaneK().c_str());
		parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneK(), uNumber2Str(scanNormalK_)));
	}
	iter = parameters.find(Parameters::kIcpPointToPlaneRadius());
	if(iter != parameters.end())
	{
		float value = uStr2Float(iter->second);
		if(value != 0.0f)
		{
			if(!this->has_parameter("scan_normal_radius"))
			{
				RCLCPP_WARN(this->get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_normal_radius\" for convenience.", iter->second.c_str(), iter->first.c_str());
				scanNormalRadius_ = value;
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_radius is set (%f), setting %s to same value.", scanNormalRadius_, Parameters::kIcpPointToPlaneRadius().c_str());
				iter->second = uNumber2Str(scanNormalK_);
			}
		}
	}
	else if(this->has_parameter("scan_normal_radius"))
	{
		RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_radius is set (%f), setting %s to same value.", scanNormalRadius_, Parameters::kIcpPointToPlaneRadius().c_str());
		parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneRadius(), uNumber2Str(scanNormalRadius_)));
	}
	iter = parameters.find(Parameters::kIcpPointToPlaneGroundNormalsUp());
	if(iter != parameters.end())
	{
		float value = uStr2Float(iter->second);
		if(value != 0.0f)
		{
			if(!this->has_parameter("scan_normal_ground_up"))
			{
				RCLCPP_WARN(get_logger(), "IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_normal_ground_up\" for convenience.", iter->second.c_str(), iter->first.c_str());
				scanNormalGroundUp_ = value;
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_ground_up is set (%f), setting %s to same value.", scanNormalGroundUp_, Parameters::kIcpPointToPlaneGroundNormalsUp().c_str());
				iter->second = uNumber2Str(scanNormalK_);
			}
		}
	}
	else if(this->has_parameter("scan_normal_ground_up"))
	{
		RCLCPP_INFO(this->get_logger(), "IcpOdometry: scan_normal_ground_up is set (%f), setting %s to same value.", scanNormalGroundUp_, Parameters::kIcpPointToPlaneGroundNormalsUp().c_str());
		parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneGroundNormalsUp(), uNumber2Str(scanNormalGroundUp_)));
	}
}

void ICPOdometry::callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg)
{
	if(cloudReceived_)
	{
		RCLCPP_ERROR(this->get_logger(), "%s is already receiving clouds on \"%s\", but also "
				"just received a scan on \"%s\". Both subscribers cannot be "
				"used at the same time! Disabling scan subscriber.",
				get_name(), cloud_sub_->get_topic_name(), scan_sub_->get_topic_name());
		scan_sub_.reset();
		return;
	}
	scanReceived_ = true;
	if(this->isPaused())
	{
		return;
	}

	// make sure the frame of the laser is updated too
	Transform localScanTransform = getTransform(this->frameId(),
			scanMsg->header.frame_id,
			rclcpp::Time(scanMsg->header.stamp.sec, scanMsg->header.stamp.nanosec) + rclcpp::Duration::from_seconds(scanMsg->ranges.size()*scanMsg->time_increment*10e9),
			tfBuffer(), waitForTransform());
	if(localScanTransform.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "TF of received laser scan topic at time %fs is not set, aborting odometry update.", timestampFromROS(scanMsg->header.stamp));
		return;
	}

	//transform in frameId_ frame
	sensor_msgs::msg::PointCloud2 scanOut;
	laser_geometry::LaserProjection projection;
	projection.transformLaserScanToPointCloud(scanMsg->header.frame_id, *scanMsg, scanOut, this->tfBuffer());

	bool hasIntensity = false;
	for(unsigned int i=0; i<scanOut.fields.size(); ++i)
	{
		if(scanOut.fields[i].name.compare("intensity") == 0)
		{
			if(scanOut.fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32)
			{
				hasIntensity = true;
			}
			else
			{
				static bool warningShown = false;
				if(!warningShown)
				{
					RCLCPP_WARN(get_logger(), "The input scan cloud has an \"intensity\" field "
							"but the datatype (%d) is not supported. Intensity will be ignored. "
							"This message is only shown once.", scanOut.fields[i].datatype);
					warningShown = true;
				}
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr pclScanI(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);

	if(hasIntensity)
	{
		pcl::fromROSMsg(scanOut, *pclScanI);
		pclScanI->is_dense = true;
	}
	else
	{
		pcl::fromROSMsg(scanOut, *pclScan);
		pclScan->is_dense = true;
	}

	LaserScan scan;
	int maxLaserScans = (int)scanMsg->ranges.size();
	if(!pclScan->empty() || !pclScanI->empty())
	{
		if(scanDownsamplingStep_ > 1)
		{
			if(hasIntensity)
			{
				pclScanI = util3d::downsample(pclScanI, scanDownsamplingStep_);
			}
			else
			{
				pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
			}
			maxLaserScans /= scanDownsamplingStep_;
		}
		if(scanVoxelSize_ > 0.0f)
		{
			float pointsBeforeFiltering;
			float pointsAfterFiltering;
			if(hasIntensity)
			{
				pointsBeforeFiltering = (float)pclScanI->size();
				pclScanI = util3d::voxelize(pclScanI, scanVoxelSize_);
				pointsAfterFiltering = (float)pclScanI->size();
			}
			else
			{
				pointsBeforeFiltering = (float)pclScan->size();
				pclScan = util3d::voxelize(pclScan, scanVoxelSize_);
				pointsAfterFiltering = (float)pclScan->size();
			}
			float ratio = pointsAfterFiltering / pointsBeforeFiltering;
			maxLaserScans = int(float(maxLaserScans) * ratio);
		}
		if(scanNormalK_ > 0 || scanNormalRadius_>0.0f)
		{
			//compute normals
			pcl::PointCloud<pcl::Normal>::Ptr normals;
			if(scanVoxelSize_ > 0.0f)
			{
				if(hasIntensity)
				{
					normals = util3d::computeNormals2D(pclScanI, scanNormalK_, scanNormalRadius_);
				}
				else
				{
					normals = util3d::computeNormals2D(pclScan, scanNormalK_, scanNormalRadius_);
				}
			}
			else
			{
				if(hasIntensity)
				{
					normals = util3d::computeFastOrganizedNormals2D(pclScanI, scanNormalK_, scanNormalRadius_);
				}
				else
				{
					normals = util3d::computeFastOrganizedNormals2D(pclScan, scanNormalK_, scanNormalRadius_);
				}
			}
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclScanINormal;
			pcl::PointCloud<pcl::PointNormal>::Ptr pclScanNormal;
			if(hasIntensity)
			{
				pclScanINormal.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
				pcl::concatenateFields(*pclScanI, *normals, *pclScanINormal);
				scan = util3d::laserScan2dFromPointCloud(*pclScanINormal);
			}
			else
			{
				pclScanNormal.reset(new pcl::PointCloud<pcl::PointNormal>);
				pcl::concatenateFields(*pclScan, *normals, *pclScanNormal);
				scan = util3d::laserScan2dFromPointCloud(*pclScanNormal);
			}
		}
		else
		{
			if(hasIntensity)
			{
				scan = util3d::laserScan2dFromPointCloud(*pclScanI);
			}
			else
			{
				scan = util3d::laserScan2dFromPointCloud(*pclScan);
			}
		}
	}

	if(scanRangeMin_ > 0 || scanRangeMax_ > 0)
	{
		scan = util3d::rangeFiltering(scan, scanRangeMin_, scanRangeMax_);
	}

	rtabmap::SensorData data(
			LaserScan(scan,
					maxLaserScans,
					scanRangeMax_>0&&scanRangeMax_<scanMsg->range_max?scanRangeMax_:scanMsg->range_max,
					localScanTransform),
			cv::Mat(),
			cv::Mat(),
			CameraModel(),
			0,
			rtabmap_ros::timestampFromROS(scanMsg->header.stamp));

	this->processData(data, scanMsg->header);
}

void ICPOdometry::callbackCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloudMsg)
{
	UASSERT_MSG(pointCloudMsg->data.size() == pointCloudMsg->row_step*pointCloudMsg->height,
			uFormat("data=%d row_step=%d height=%d", pointCloudMsg->data.size(), pointCloudMsg->row_step, pointCloudMsg->height).c_str());

	if(scanReceived_)
	{
		RCLCPP_ERROR(this->get_logger(), "%s is already receiving scans on \"%s\", but also "
				"just received a cloud on \"%s\". Both subscribers cannot be "
				"used at the same time! Disabling cloud subscriber.",
				this->get_name(), scan_sub_->get_topic_name(), cloud_sub_->get_topic_name());
		cloud_sub_.reset();
		return;
	}
	cloudReceived_ = true;
	if(this->isPaused())
	{
		return;
	}

	sensor_msgs::msg::PointCloud2 cloudMsg;
	/*if (!plugins_.empty())
	{
		if (plugins_[0]->isEnabled())
		{
			cloudMsg = plugins_[0]->filterPointCloud(*pointCloudMsg);
		}
		else
		{
			cloudMsg = *pointCloudMsg;
		}

		if (plugins_.size() > 1)
		{
			for (int i = 1; i < plugins_.size(); i++) {
				if (plugins_[i]->isEnabled()) {
					cloudMsg = plugins_[i]->filterPointCloud(cloudMsg);
				}
			}
		}
	}
	else */
	{
		cloudMsg = *pointCloudMsg;
	}

	LaserScan scan;
	bool hasNormals = false;
	bool hasIntensity = false;
	for(unsigned int i=0; i<cloudMsg.fields.size(); ++i)
	{
		if(scanVoxelSize_ == 0.0f && cloudMsg.fields[i].name.compare("normal_x") == 0)
		{
			hasNormals = true;
		}
		if(cloudMsg.fields[i].name.compare("intensity") == 0)
		{
			if(cloudMsg.fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32)
			{
				hasIntensity = true;
			}
			else
			{
				static bool warningShown = false;
				if(!warningShown)
				{
					RCLCPP_WARN(this->get_logger(), "The input scan cloud has an \"intensity\" field "
							"but the datatype (%d) is not supported. Intensity will be ignored. "
							"This message is only shown once.", cloudMsg.fields[i].datatype);
					warningShown = true;
				}
			}
		}
	}

	Transform localScanTransform = getTransform(this->frameId(), cloudMsg.header.frame_id, cloudMsg.header.stamp, tfBuffer(), waitForTransform());
	if(localScanTransform.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "TF of received scan cloud at time %fs is not set, aborting rtabmap update.", timestampFromROS(cloudMsg.header.stamp));
		return;
	}
	if(scanCloudMaxPoints_ == 0 && cloudMsg.height > 1)
	{
		scanCloudMaxPoints_ = cloudMsg.height * cloudMsg.width;
		RCLCPP_WARN(this->get_logger(), "IcpOdometry: \"scan_cloud_max_points\" is not set but input "
				"cloud is not dense, for convenience it will be set to %d (%dx%d)",
				scanCloudMaxPoints_, cloudMsg.width, cloudMsg.height);
	}
	else if(cloudMsg.height > 1 && scanCloudMaxPoints_ < int(cloudMsg.height * cloudMsg.width))
	{
		RCLCPP_WARN(this->get_logger(), "IcpOdometry: \"scan_cloud_max_points\" is set to %d but input "
				"cloud is not dense and has a size of %d (%dx%d), setting to this later size.",
				scanCloudMaxPoints_, cloudMsg.width *cloudMsg.height, cloudMsg.width, cloudMsg.height);
		scanCloudMaxPoints_ = cloudMsg.width *cloudMsg.height;
	}
	int maxLaserScans = scanCloudMaxPoints_;

	if(hasNormals && hasIntensity)
	{
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::fromROSMsg(cloudMsg, *pclScan);
		if(pclScan->size() && scanDownsamplingStep_ > 1)
		{
			pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
			if(pclScan->height>1)
			{
				maxLaserScans = pclScan->height * pclScan->width;
			}
			else
			{
				maxLaserScans /= scanDownsamplingStep_;
			}
		}
		scan = util3d::laserScanFromPointCloud(*pclScan);
	}
	else if(hasNormals)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(cloudMsg, *pclScan);
		if(pclScan->size() && scanDownsamplingStep_ > 1)
		{
			pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
			if(pclScan->height>1)
			{
				maxLaserScans = pclScan->height * pclScan->width;
			}
			else
			{
				maxLaserScans /= scanDownsamplingStep_;
			}
		}
		scan = util3d::laserScanFromPointCloud(*pclScan);
	}
	else if(hasIntensity)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(cloudMsg, *pclScan);
		if(pclScan->size() && scanDownsamplingStep_ > 1)
		{
			pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
			if(pclScan->height>1)
			{
				maxLaserScans = pclScan->height * pclScan->width;
			}
			else
			{
				maxLaserScans /= scanDownsamplingStep_;
			}
		}
		if(!pclScan->is_dense)
		{
			pclScan = util3d::removeNaNFromPointCloud(pclScan);
		}

		if(pclScan->size())
		{
			if(scanVoxelSize_ > 0.0f)
			{
				float pointsBeforeFiltering = (float)pclScan->size();
				pclScan = util3d::voxelize(pclScan, scanVoxelSize_);
				float ratio = float(pclScan->size()) / pointsBeforeFiltering;
				maxLaserScans = int(float(maxLaserScans) * ratio);
			}
			if(scanNormalK_ > 0 || scanNormalRadius_>0.0f)
			{
				//compute normals
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(pclScan, scanNormalK_, scanNormalRadius_);
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclScanNormal(new pcl::PointCloud<pcl::PointXYZINormal>);
				pcl::concatenateFields(*pclScan, *normals, *pclScanNormal);
				scan = util3d::laserScanFromPointCloud(*pclScanNormal);
			}
			else
			{
				scan = util3d::laserScanFromPointCloud(*pclScan);
			}
		}
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(cloudMsg, *pclScan);
		if(pclScan->size() && scanDownsamplingStep_ > 1)
		{
			pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
			if(pclScan->height>1)
			{
				maxLaserScans = pclScan->height * pclScan->width;
			}
			else
			{
				maxLaserScans /= scanDownsamplingStep_;
			}

		}
		if(!pclScan->is_dense)
		{
			pclScan = util3d::removeNaNFromPointCloud(pclScan);
		}

		if(pclScan->size())
		{
			if(scanVoxelSize_ > 0.0f)
			{
				float pointsBeforeFiltering = (float)pclScan->size();
				pclScan = util3d::voxelize(pclScan, scanVoxelSize_);
				float ratio = float(pclScan->size()) / pointsBeforeFiltering;
				maxLaserScans = int(float(maxLaserScans) * ratio);
			}
			if(scanNormalK_ > 0 || scanNormalRadius_>0.0f)
			{
				//compute normals
				pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(pclScan, scanNormalK_, scanNormalRadius_);
				pcl::PointCloud<pcl::PointNormal>::Ptr pclScanNormal(new pcl::PointCloud<pcl::PointNormal>);
				pcl::concatenateFields(*pclScan, *normals, *pclScanNormal);
				scan = util3d::laserScanFromPointCloud(*pclScanNormal);
			}
			else
			{
				scan = util3d::laserScanFromPointCloud(*pclScan);
			}
		}
	}

	LaserScan laserScan(scan,
			maxLaserScans,
			0,
			localScanTransform);
	if(scanRangeMin_ > 0 || scanRangeMax_ > 0)
	{
		laserScan = util3d::rangeFiltering(laserScan, scanRangeMin_, scanRangeMax_);
	}
	if(!laserScan.isEmpty() && laserScan.hasNormals() && !laserScan.is2d() && scanNormalGroundUp_)
	{
		laserScan = util3d::adjustNormalsToViewPoint(laserScan, Eigen::Vector3f(0,0,10), (float)scanNormalGroundUp_);
	}

	rtabmap::SensorData data(
			laserScan,
			cv::Mat(),
			cv::Mat(),
			CameraModel(),
			0,
			rtabmap_ros::timestampFromROS(cloudMsg.header.stamp));

	this->processData(data, cloudMsg.header);
}

void ICPOdometry::flushCallbacks()
{
	// flush callbacks
}

void ICPOdometry::postProcessData(const SensorData & data, const std_msgs::msg::Header & header) const
{
	if(filtered_scan_pub_->get_subscription_count())
	{
		sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::fromPCL(*rtabmap::util3d::laserScanToPointCloud2(data.laserScanRaw()), *msg);
		msg->header = header;
		filtered_scan_pub_->publish(std::move(msg));
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::ICPOdometry)
