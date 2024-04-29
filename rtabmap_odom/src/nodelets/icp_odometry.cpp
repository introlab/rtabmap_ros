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

#include <rtabmap_odom/OdometryROS.h>

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

#include <nodelet/nodelet.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "rtabmap_conversions/MsgConversion.h"
#include "rtabmap_odom/PluginInterface.h"

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

using namespace rtabmap;

namespace rtabmap_odom
{

class ICPOdometry : public OdometryROS
{
public:
	ICPOdometry() :
		OdometryROS(false, false, true),
		scanCloudMaxPoints_(-1),
		scanCloudIs2d_(false),
		scanDownsamplingStep_(1),
		scanRangeMin_(0),
		scanRangeMax_(0),
		scanVoxelSize_(0.0),
		scanNormalK_(0),
		scanNormalRadius_(0.0),
		scanNormalGroundUp_(0.0),
		deskewing_(false),
		deskewingSlerp_(false),
		plugin_loader_("rtabmap_odom", "rtabmap_odom::PluginInterface"),
		scanReceived_(false),
		cloudReceived_(false)
	{
	}

	virtual ~ICPOdometry()
	{
		plugins_.clear();
	}

private:

	virtual void onOdomInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 1;
		pnh.param("queue_size",  queueSize, queueSize);
		pnh.param("scan_cloud_max_points",  scanCloudMaxPoints_, scanCloudMaxPoints_);
		pnh.param("scan_cloud_is_2d",  scanCloudIs2d_, scanCloudIs2d_);
		pnh.param("scan_downsampling_step", scanDownsamplingStep_, scanDownsamplingStep_);
		pnh.param("scan_range_min",  scanRangeMin_, scanRangeMin_);
		pnh.param("scan_range_max",  scanRangeMax_, scanRangeMax_);
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);
		pnh.param("scan_normal_k",   scanNormalK_, scanNormalK_);
		pnh.param("scan_normal_radius", scanNormalRadius_, scanNormalRadius_);
		pnh.param("scan_normal_ground_up", scanNormalGroundUp_, scanNormalGroundUp_);
		pnh.param("deskewing",  deskewing_, deskewing_);
		pnh.param("deskewing_slerp",  deskewingSlerp_, deskewingSlerp_);

		if (pnh.hasParam("plugins"))
		{
			XmlRpc::XmlRpcValue pluginsList;
			pnh.getParam("plugins", pluginsList);

			for (int32_t i = 0; i < pluginsList.size(); ++i)
			{
				std::string pluginName = static_cast<std::string>(pluginsList[i]["name"]);
				std::string type = static_cast<std::string>(pluginsList[i]["type"]);
				NODELET_INFO("IcpOdometry: Using plugin %s of type \"%s\"", pluginName.c_str(), type.c_str());
				try {
					boost::shared_ptr<rtabmap_odom::PluginInterface> plugin = plugin_loader_.createInstance(type);
					plugins_.push_back(plugin);
					plugin->initialize(pluginName, pnh);
					if(!plugin->isEnabled())
					{
						NODELET_WARN("Plugin: %s is not enabled, filtering will not occur. \"enabled_\" member "
									 "should be managed in subclasses. This can be ignored if the "
									 "plugin should really be initialized as disabled.",
									 plugin->getName().c_str());
					}
				}
				catch(pluginlib::PluginlibException & ex) {
					ROS_ERROR("Failed to load plugin %s. Error: %s", pluginName.c_str(), ex.what());
				}

			}
		}

		if(pnh.hasParam("scan_cloud_normal_k") && !pnh.hasParam("scan_normal_k"))
		{
			ROS_WARN("rtabmap: Parameter \"scan_cloud_normal_k\" has been renamed to \"scan_normal_k\". "
					"The value is still used. Use \"scan_normal_k\" to avoid this warning.");
			pnh.param("scan_cloud_normal_k", scanNormalK_, scanNormalK_);
		}

		NODELET_INFO("IcpOdometry: queue_size             = %d", queueSize);
		NODELET_INFO("IcpOdometry: scan_cloud_max_points  = %d", scanCloudMaxPoints_);
		NODELET_INFO("IcpOdometry: scan_cloud_is_2d       = %s", scanCloudIs2d_?"true":"false");
		NODELET_INFO("IcpOdometry: scan_downsampling_step = %d", scanDownsamplingStep_);
		NODELET_INFO("IcpOdometry: scan_range_min         = %f m", scanRangeMin_);
		NODELET_INFO("IcpOdometry: scan_range_max         = %f m", scanRangeMax_);
		NODELET_INFO("IcpOdometry: scan_voxel_size        = %f m", scanVoxelSize_);
		NODELET_INFO("IcpOdometry: scan_normal_k          = %d", scanNormalK_);
		NODELET_INFO("IcpOdometry: scan_normal_radius     = %f m", scanNormalRadius_);
		NODELET_INFO("IcpOdometry: scan_normal_ground_up  = %f", scanNormalGroundUp_);
		NODELET_INFO("IcpOdometry: deskewing              = %s", deskewing_?"true":"false");
		NODELET_INFO("IcpOdometry: deskewing_slerp        = %s", deskewingSlerp_?"true":"false");

		scan_sub_ = nh.subscribe("scan", queueSize, &ICPOdometry::callbackScan, this);
		cloud_sub_ = nh.subscribe("scan_cloud", queueSize, &ICPOdometry::callbackCloud, this);

		filtered_scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("odom_filtered_input_scan", 1);

		initDiagnosticMsg(uFormat("\n%s subscribed to %s and %s (make sure only one of this topic is published, otherwise remap one to a dummy topic name).",
					getName().c_str(),
					scan_sub_.getTopic().c_str(),
					cloud_sub_.getTopic().c_str()), true);
	}

	virtual void updateParameters(ParametersMap & parameters)
	{
		//make sure we are using Reg/Strategy=0
		ParametersMap::iterator iter = parameters.find(Parameters::kRegStrategy());
		if(iter != parameters.end() && iter->second.compare("1") != 0)
		{
			ROS_WARN("ICP odometry works only with \"Reg/Strategy\"=1. Ignoring value %s.", iter->second.c_str());
		}
		uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "1"));

		ros::NodeHandle & pnh = getPrivateNodeHandle();
		iter = parameters.find(Parameters::kIcpDownsamplingStep());
		if(iter != parameters.end())
		{
			int value = uStr2Int(iter->second);
			if(value > 1)
			{
				if(!pnh.hasParam("scan_downsampling_step"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_downsampling_step\" for convenience. \"%s\" is set to 1.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
					scanDownsamplingStep_ = value;
					iter->second = "1";
				}
				else
				{
					ROS_WARN("IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_downsampling_step\" are set.", iter->first.c_str());
				}
			}
		}
		iter = parameters.find(Parameters::kIcpRangeMin());
		if(iter != parameters.end())
		{
			float value = uStr2Float(iter->second);
			if(value != 0.0f)
			{
				if(!pnh.hasParam("scan_range_min"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_range_min\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
					scanRangeMin_ = value;
					iter->second = "0";
				}
				else
				{
					ROS_WARN("IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_range_min\" are set.", iter->first.c_str());
				}
			}
		}
		iter = parameters.find(Parameters::kIcpRangeMax());
		if(iter != parameters.end())
		{
			float value = uStr2Float(iter->second);
			if(value != 0.0f)
			{
				if(!pnh.hasParam("scan_range_max"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_range_max\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
					scanRangeMax_ = value;
					iter->second = "0";
				}
				else
				{
					ROS_WARN("IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_range_max\" are set.", iter->first.c_str());
				}
			}
		}
		iter = parameters.find(Parameters::kIcpVoxelSize());
		if(iter != parameters.end())
		{
			float value = uStr2Float(iter->second);
			if(value != 0.0f)
			{
				if(!pnh.hasParam("scan_voxel_size"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_voxel_size\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
					scanVoxelSize_ = value;
					iter->second = "0";
				}
				else
				{
					ROS_WARN("IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_voxel_size\" are set.", iter->first.c_str());
				}
			}
		}
		else if(pnh.hasParam("scan_voxel_size"))
		{
			NODELET_INFO("IcpOdometry: scan_voxel_size is set (%f), setting %s to 0", scanVoxelSize_, Parameters::kIcpVoxelSize().c_str());
			parameters.insert(ParametersPair(Parameters::kIcpVoxelSize(), "0"));
		}
		iter = parameters.find(Parameters::kIcpPointToPlaneK());
		if(iter != parameters.end())
		{
			int value = uStr2Int(iter->second);
			if(value != 0)
			{
				if(!pnh.hasParam("scan_normal_k"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_normal_k\" for convenience.", iter->second.c_str(), iter->first.c_str());
					scanNormalK_ = value;
				}
				else
				{
					NODELET_INFO("IcpOdometry: scan_normal_k is set (%d), setting %s to same value.", scanNormalK_, Parameters::kIcpPointToPlaneK().c_str());
					iter->second = uNumber2Str(scanNormalK_);
				}
			}
		}
		else if(pnh.hasParam("scan_normal_k"))
		{
			NODELET_INFO("IcpOdometry: scan_normal_k is set (%d), setting %s to same value.", scanNormalK_, Parameters::kIcpPointToPlaneK().c_str());
			parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneK(), uNumber2Str(scanNormalK_)));
		}
		iter = parameters.find(Parameters::kIcpPointToPlaneRadius());
		if(iter != parameters.end())
		{
			float value = uStr2Float(iter->second);
			if(value != 0.0f)
			{
				if(!pnh.hasParam("scan_normal_radius"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_normal_radius\" for convenience.", iter->second.c_str(), iter->first.c_str());
					scanNormalRadius_ = value;
				}
				else
				{
					NODELET_INFO("IcpOdometry: scan_normal_radius is set (%f), setting %s to same value.", scanNormalRadius_, Parameters::kIcpPointToPlaneRadius().c_str());
					iter->second = uNumber2Str(scanNormalK_);
				}
			}
		}
		else if(pnh.hasParam("scan_normal_radius"))
		{
			NODELET_INFO("IcpOdometry: scan_normal_radius is set (%f), setting %s to same value.", scanNormalRadius_, Parameters::kIcpPointToPlaneRadius().c_str());
			parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneRadius(), uNumber2Str(scanNormalRadius_)));
		}
		iter = parameters.find(Parameters::kIcpPointToPlaneGroundNormalsUp());
		if(iter != parameters.end())
		{
			float value = uStr2Float(iter->second);
			if(value != 0.0f)
			{
				if(!pnh.hasParam("scan_normal_ground_up"))
				{
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_normal_ground_up\" for convenience.", iter->second.c_str(), iter->first.c_str());
					scanNormalGroundUp_ = value;
				}
				else
				{
					NODELET_INFO("IcpOdometry: scan_normal_ground_up is set (%f), setting %s to same value.", scanNormalGroundUp_, Parameters::kIcpPointToPlaneGroundNormalsUp().c_str());
					iter->second = uNumber2Str(scanNormalK_);
				}
			}
		}
		else if(pnh.hasParam("scan_normal_ground_up"))
		{
			NODELET_INFO("IcpOdometry: scan_normal_ground_up is set (%f), setting %s to same value.", scanNormalGroundUp_, Parameters::kIcpPointToPlaneGroundNormalsUp().c_str());
			parameters.insert(ParametersPair(Parameters::kIcpPointToPlaneGroundNormalsUp(), uNumber2Str(scanNormalGroundUp_)));
		}
	}

	void callbackScan(const sensor_msgs::LaserScanConstPtr& scanMsg)
	{
		if(cloudReceived_)
		{
			ROS_ERROR("%s is already receiving clouds on \"%s\", but also "
					"just received a scan on \"%s\". Both subscribers cannot be "
					"used at the same time! Disabling scan subscriber.",
					this->getName().c_str(), cloud_sub_.getTopic().c_str(), scan_sub_.getTopic().c_str());
			scan_sub_.shutdown();
			return;
		}

		scanReceived_ = true;
		if(this->isPaused())
		{
			return;
		}

		// make sure the frame of the laser is updated
		Transform localScanTransform = rtabmap_conversions::getTransform(this->frameId(),
				scanMsg->header.frame_id,
				scanMsg->header.stamp,
				this->tfListener(),
				this->waitForTransformDuration());
		if(localScanTransform.isNull())
		{
			ROS_ERROR("TF of received laser scan topic at time %fs is not set, aborting odometry update.", scanMsg->header.stamp.toSec());
			return;
		}

		//transform in scan frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;

		if(deskewing_ && (!guessFrameId().empty() || (frameId().compare(scanMsg->header.frame_id) != 0)))
		{
			// make sure the frame of the laser is updated during the whole scan time
			rtabmap::Transform tmpT = rtabmap_conversions::getMovingTransform(
					scanMsg->header.frame_id,
					guessFrameId().empty()?frameId():guessFrameId(),
					scanMsg->header.stamp,
					scanMsg->header.stamp + ros::Duration().fromSec(scanMsg->ranges.size()*scanMsg->time_increment),
					this->tfListener(),
					this->waitForTransformDuration());
			if(tmpT.isNull())
			{
				return;
			}

			projection.transformLaserScanToPointCloud(
					guessFrameId().empty()?frameId():guessFrameId(),
					*scanMsg, 
					scanOut,
					this->tfListener(),
					-1.0,
					laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Timestamp);

			if(guessFrameId().empty() && previousStamp() > 0 && !velocityGuess().isNull())
			{
				// deskew with constant velocity model (we are in frameId)
				sensor_msgs::PointCloud2 scanOutDeskewed;
				if(!rtabmap_conversions::deskew(scanOut, scanOutDeskewed, previousStamp(), velocityGuess()))
				{
					ROS_ERROR("Failed to deskew input cloud, aborting odometry update!");
					return;
				}
				scanOut = scanOutDeskewed;
			}

			sensor_msgs::PointCloud2 scanOutDeskewed;
			if(!pcl_ros::transformPointCloud(scanMsg->header.frame_id, scanOut, scanOutDeskewed, this->tfListener()))
			{
				ROS_ERROR("Cannot transform back projected scan from \"%s\" frame to \"%s\" frame at time %fs.",
						(guessFrameId().empty()?frameId():guessFrameId()).c_str(), scanMsg->header.frame_id.c_str(), scanMsg->header.stamp.toSec());
				return;
			}
			scanOut = scanOutDeskewed;
		}
		else
		{
			projection.projectLaser(*scanMsg, scanOut, -1.0, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Timestamp);

			if(deskewing_ && previousStamp() > 0 && !velocityGuess().isNull())
			{
				// deskew with constant velocity model
				sensor_msgs::PointCloud2 scanOutDeskewed;
				if(!rtabmap_conversions::deskew(scanOut, scanOutDeskewed, previousStamp(), velocityGuess()))
				{
					ROS_ERROR("Failed to deskew input cloud, aborting odometry update!");
					return;
				}
				scanOut = scanOutDeskewed;
			}
		}

		bool hasIntensity = false;
		for(unsigned int i=0; i<scanOut.fields.size(); ++i)
		{
			if(scanOut.fields[i].name.compare("intensity") == 0)
			{
				if(scanOut.fields[i].datatype == sensor_msgs::PointField::FLOAT32)
				{
					hasIntensity = true;
				}
				else
				{
					static bool warningShown = false;
					if(!warningShown)
					{
						ROS_WARN("The input scan cloud has an \"intensity\" field "
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
				rtabmap::CameraModel(),
				0,
				rtabmap_conversions::timestampFromROS(scanMsg->header.stamp));

		this->processData(data, scanMsg->header);
	}

	void callbackCloud(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
	{
		UASSERT_MSG(pointCloudMsg->data.size() == pointCloudMsg->row_step*pointCloudMsg->height,
				uFormat("data=%d row_step=%d height=%d", pointCloudMsg->data.size(), pointCloudMsg->row_step, pointCloudMsg->height).c_str());
		
		if(scanReceived_)
		{
			ROS_ERROR("%s is already receiving scans on \"%s\", but also "
					"just received a cloud on \"%s\". Both subscribers cannot be "
					"used at the same time! Disabling cloud subscriber.",
					this->getName().c_str(), scan_sub_.getTopic().c_str(), cloud_sub_.getTopic().c_str());
			cloud_sub_.shutdown();
			return;
		}

		cloudReceived_ = true;
		if(this->isPaused())
		{
			return;
		}

		sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
		if (!plugins_.empty())
		{
			if (plugins_[0]->isEnabled())
			{
				*cloudMsg = plugins_[0]->filterPointCloud(*pointCloudMsg);
			}
			else
			{
				*cloudMsg = *pointCloudMsg;
			}

			if (plugins_.size() > 1)
			{
				for (int i = 1; i < plugins_.size(); i++) {
					if (plugins_[i]->isEnabled()) {
						*cloudMsg = plugins_[i]->filterPointCloud(*cloudMsg);
					}
				}
			}
		}
		else
		{
			*cloudMsg = *pointCloudMsg;
		}

		Transform localScanTransform = rtabmap_conversions::getTransform(this->frameId(), cloudMsg->header.frame_id, cloudMsg->header.stamp, this->tfListener(), this->waitForTransformDuration());
		if(localScanTransform.isNull())
		{
			ROS_ERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", cloudMsg->header.stamp.toSec());
			return;
		}

		if(deskewing_)
		{
			if(!guessFrameId().empty())
			{
				// deskew with TF
				if(!rtabmap_conversions::deskew(*pointCloudMsg, *cloudMsg, guessFrameId(), tfListener(), waitForTransformDuration(), deskewingSlerp_))
				{
					ROS_ERROR("Failed to deskew input cloud, aborting odometry update!");
					return;
				}
			}
			else if(previousStamp() > 0 && !velocityGuess().isNull())
			{
				// deskew with constant velocity model
				bool alreadyInBaseFrame = frameId().compare(pointCloudMsg->header.frame_id) == 0;
				sensor_msgs::PointCloud2Ptr cloudInBaseFrame;
				sensor_msgs::PointCloud2Ptr cloudPtr = cloudMsg;
				if(!alreadyInBaseFrame)
				{
					// transform in base frame
					cloudInBaseFrame.reset(new sensor_msgs::PointCloud2);
					if(!pcl_ros::transformPointCloud(frameId(), *pointCloudMsg, *cloudInBaseFrame, this->tfListener()))
					{
						ROS_ERROR("Cannot transform back projected scan from \"%s\" frame to \"%s\" frame at time %fs.",
								pointCloudMsg->header.frame_id.c_str(), frameId().c_str(), pointCloudMsg->header.stamp.toSec());
						return;
					}
					cloudPtr = cloudInBaseFrame;
				}

				sensor_msgs::PointCloud2::Ptr cloudDeskewed(new sensor_msgs::PointCloud2);
				if(!rtabmap_conversions::deskew(*cloudPtr, *cloudDeskewed, previousStamp(), velocityGuess()))
				{
					ROS_ERROR("Failed to deskew input cloud, aborting odometry update!");
					return;
				}

				if(!alreadyInBaseFrame)
				{
					// put back in scan frame
					if(!pcl_ros::transformPointCloud(pointCloudMsg->header.frame_id.c_str(), *cloudDeskewed, *cloudMsg, this->tfListener()))
					{
						ROS_ERROR("Cannot transform back projected scan from \"%s\" frame to \"%s\" frame at time %fs.",
								frameId().c_str(), pointCloudMsg->header.frame_id.c_str(), pointCloudMsg->header.stamp.toSec());
						return;
					}
				}
				else
				{
					cloudMsg = cloudDeskewed;
				}
			}
		}

		LaserScan scan;
		bool hasNormals = false;
		bool hasIntensity = false;
		bool is3D = false;
		for(unsigned int i=0; i<cloudMsg->fields.size(); ++i)
		{
			if(scanVoxelSize_ == 0.0f && cloudMsg->fields[i].name.compare("normal_x") == 0)
			{
				hasNormals = true;
			}
			if(cloudMsg->fields[i].name.compare("z") == 0 && !scanCloudIs2d_)
			{
				is3D = true;
			}
			if(cloudMsg->fields[i].name.compare("intensity") == 0)
			{
				if(cloudMsg->fields[i].datatype == sensor_msgs::PointField::FLOAT32)
				{
					hasIntensity = true;
				}
				else
				{
					static bool warningShown = false;
					if(!warningShown)
					{
						ROS_WARN("The input scan cloud has an \"intensity\" field "
								"but the datatype (%d) is not supported. Intensity will be ignored. "
								"This message is only shown once.", cloudMsg->fields[i].datatype);
						warningShown = true;
					}
				}
			}
		}

        if(cloudMsg->height > 1) // organized cloud
        {
		    if(scanCloudMaxPoints_ == -1)
		    {
			    scanCloudMaxPoints_ = cloudMsg->height * cloudMsg->width;
			    NODELET_WARN("IcpOdometry: \"scan_cloud_max_points\" is not set but input "
					    "cloud is not dense, for convenience it will be set to %d (%dx%d)",
					    scanCloudMaxPoints_, cloudMsg->width, cloudMsg->height);
		    }
		    else if(scanCloudMaxPoints_ > 0 && scanCloudMaxPoints_ < cloudMsg->height * cloudMsg->width)
		    {
			    NODELET_WARN("IcpOdometry: \"scan_cloud_max_points\" is set to %d but input "
					    "cloud is not dense and has a size of %d (%dx%d), setting to this later size.",
					    scanCloudMaxPoints_, cloudMsg->width *cloudMsg->height, cloudMsg->width, cloudMsg->height);
			    scanCloudMaxPoints_ = cloudMsg->width *cloudMsg->height;
		    }
		}
		if(scanCloudMaxPoints_ == -1)
		{
		    scanCloudMaxPoints_ = 0;
		}
		int maxLaserScans = scanCloudMaxPoints_;

		if(hasNormals && hasIntensity)
		{
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::fromROSMsg(*cloudMsg, *pclScan);
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
			scan = is3D?util3d::laserScanFromPointCloud(*pclScan):util3d::laserScan2dFromPointCloud(*pclScan);
		}
		else if(hasNormals)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromROSMsg(*cloudMsg, *pclScan);
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
			scan = is3D?util3d::laserScanFromPointCloud(*pclScan):util3d::laserScan2dFromPointCloud(*pclScan);
		}
		else if(hasIntensity)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::fromROSMsg(*cloudMsg, *pclScan);
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
					pcl::PointCloud<pcl::Normal>::Ptr normals = is3D?
							util3d::computeNormals(pclScan, scanNormalK_, scanNormalRadius_):
							util3d::computeNormals2D(pclScan, scanNormalK_, scanNormalRadius_);
					pcl::PointCloud<pcl::PointXYZINormal>::Ptr pclScanNormal(new pcl::PointCloud<pcl::PointXYZINormal>);
					pcl::concatenateFields(*pclScan, *normals, *pclScanNormal);
					scan = is3D?util3d::laserScanFromPointCloud(*pclScanNormal):util3d::laserScan2dFromPointCloud(*pclScanNormal);
				}
				else
				{
					scan = is3D?util3d::laserScanFromPointCloud(*pclScan):util3d::laserScan2dFromPointCloud(*pclScan);
				}
			}
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*cloudMsg, *pclScan);
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
					pcl::PointCloud<pcl::Normal>::Ptr normals = is3D?
							util3d::computeNormals(pclScan, scanNormalK_, scanNormalRadius_):
							util3d::computeNormals2D(pclScan, scanNormalK_, scanNormalRadius_);
					pcl::PointCloud<pcl::PointNormal>::Ptr pclScanNormal(new pcl::PointCloud<pcl::PointNormal>);
					pcl::concatenateFields(*pclScan, *normals, *pclScanNormal);
					scan = is3D?util3d::laserScanFromPointCloud(*pclScanNormal):util3d::laserScan2dFromPointCloud(*pclScanNormal);
				}
				else
				{
					scan = is3D?util3d::laserScanFromPointCloud(*pclScan):util3d::laserScan2dFromPointCloud(*pclScan);
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
				rtabmap::CameraModel(),
				0,
				rtabmap_conversions::timestampFromROS(cloudMsg->header.stamp));

		this->processData(data, cloudMsg->header);
	}

protected:
	virtual void flushCallbacks()
	{
		// flush callbacks
	}

	void postProcessData(const SensorData & data, const std_msgs::Header & header) const
	{
		if(filtered_scan_pub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 msg;
			pcl_conversions::fromPCL(*rtabmap::util3d::laserScanToPointCloud2(data.laserScanRaw()), msg);
			msg.header = header;
			filtered_scan_pub_.publish(msg);
		}
	}

private:
	ros::Subscriber scan_sub_;
	ros::Subscriber cloud_sub_;
	ros::Publisher filtered_scan_pub_;
	int scanCloudMaxPoints_;
	bool scanCloudIs2d_;
	int scanDownsamplingStep_;
	double scanRangeMin_;
	double scanRangeMax_;
	double scanVoxelSize_;
	int scanNormalK_;
	double scanNormalRadius_;
	double scanNormalGroundUp_;
	bool deskewing_;
	bool deskewingSlerp_;
	std::vector<boost::shared_ptr<rtabmap_odom::PluginInterface> > plugins_;
	pluginlib::ClassLoader<rtabmap_odom::PluginInterface> plugin_loader_;
	bool scanReceived_ = false;
	bool cloudReceived_ = false;

};

PLUGINLIB_EXPORT_CLASS(rtabmap_odom::ICPOdometry, nodelet::Nodelet);

}
