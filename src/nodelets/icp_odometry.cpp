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

#include <rtabmap_ros/OdometryROS.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
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

class ICPOdometry : public rtabmap_ros::OdometryROS
{
public:
	ICPOdometry() :
		OdometryROS(false, false, true),
		scanCloudMaxPoints_(0),
		scanDownsamplingStep_(1),
		scanVoxelSize_(0.0),
		scanNormalK_(0),
		scanNormalRadius_(0.0)
	{
	}

	virtual ~ICPOdometry()
	{
	}

private:

	virtual void onOdomInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		pnh.param("scan_cloud_max_points",  scanCloudMaxPoints_, scanCloudMaxPoints_);
		pnh.param("scan_downsampling_step", scanDownsamplingStep_, scanDownsamplingStep_);
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);
		pnh.param("scan_normal_k", scanNormalK_, scanNormalK_);
		if(pnh.hasParam("scan_cloud_normal_k") && !pnh.hasParam("scan_normal_k"))
		{
			ROS_WARN("rtabmap: Parameter \"scan_cloud_normal_k\" has been renamed to \"scan_normal_k\". "
					"The value is still used. Use \"scan_normal_k\" to avoid this warning.");
			pnh.param("scan_cloud_normal_k", scanNormalK_, scanNormalK_);
		}
		pnh.param("scan_normal_radius", scanNormalRadius_, scanNormalRadius_);

		NODELET_INFO("IcpOdometry: scan_cloud_max_points  = %d", scanCloudMaxPoints_);
		NODELET_INFO("IcpOdometry: scan_downsampling_step = %d", scanDownsamplingStep_);
		NODELET_INFO("IcpOdometry: scan_voxel_size        = %f", scanVoxelSize_);
		NODELET_INFO("IcpOdometry: scan_normal_k          = %d", scanNormalK_);
		NODELET_INFO("IcpOdometry: scan_normal_radius     = %f", scanNormalRadius_);

		scan_sub_ = nh.subscribe("scan", 1, &ICPOdometry::callbackScan, this);
		cloud_sub_ = nh.subscribe("scan_cloud", 1, &ICPOdometry::callbackCloud, this);
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
					ROS_WARN("IcpOdometry: Transferring value %s of \"%s\" to ros parameter \"scan_downsampling_step\" for convenience. \"%s\" is set to 0.", iter->second.c_str(), iter->first.c_str(), iter->first.c_str());
					scanDownsamplingStep_ = value;
					iter->second = "1";
				}
				else
				{
					ROS_WARN("IcpOdometry: Both parameter \"%s\" and ros parameter \"scan_downsampling_step\" are set.", iter->first.c_str());
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
			}
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
			}
		}
	}

	void callbackScan(const sensor_msgs::LaserScanConstPtr& scanMsg)
	{
		// make sure the frame of the laser is updated too
		Transform localScanTransform = getTransform(this->frameId(),
				scanMsg->header.frame_id,
				scanMsg->header.stamp + ros::Duration().fromSec(scanMsg->ranges.size()*scanMsg->time_increment));
		if(localScanTransform.isNull())
		{
			ROS_ERROR("TF of received laser scan topic at time %fs is not set, aborting odometry update.", scanMsg->header.stamp.toSec());
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(scanMsg->header.frame_id, *scanMsg, scanOut, this->tfListener());
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(scanOut, *pclScan);
		pclScan->is_dense = true;

		cv::Mat scan;
		int maxLaserScans = (int)scanMsg->ranges.size();
		if(pclScan->size())
		{
			if(scanDownsamplingStep_ > 1)
			{
				pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
				maxLaserScans /= scanDownsamplingStep_;
			}
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
				pcl::PointCloud<pcl::Normal>::Ptr normals;
				if(scanVoxelSize_ > 0.0f)
				{
					normals = util3d::computeNormals2D(pclScan, scanNormalK_, scanNormalRadius_);
				}
				else
				{
					normals = util3d::computeFastOrganizedNormals2D(pclScan, scanNormalK_, scanNormalRadius_);
				}
				pcl::PointCloud<pcl::PointNormal>::Ptr pclScanNormal(new pcl::PointCloud<pcl::PointNormal>);
				pcl::concatenateFields(*pclScan, *normals, *pclScanNormal);
				scan = util3d::laserScan2dFromPointCloud(*pclScanNormal);
			}
			else
			{
				scan = util3d::laserScan2dFromPointCloud(*pclScan);
			}
		}

		rtabmap::SensorData data(
				LaserScan::backwardCompatibility(scan, maxLaserScans, scanMsg->range_max, localScanTransform),
				cv::Mat(),
				cv::Mat(),
				CameraModel(),
				0,
				rtabmap_ros::timestampFromROS(scanMsg->header.stamp));

		this->processData(data, scanMsg->header.stamp);
	}

	void callbackCloud(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
	{
		cv::Mat scan;
		bool containNormals = false;
		if(scanVoxelSize_ == 0.0f)
		{
			for(unsigned int i=0; i<cloudMsg->fields.size(); ++i)
			{
				if(cloudMsg->fields[i].name.compare("normal_x") == 0)
				{
					containNormals = true;
					break;
				}
			}
		}

		Transform localScanTransform = getTransform(this->frameId(), cloudMsg->header.frame_id, cloudMsg->header.stamp);
		if(localScanTransform.isNull())
		{
			ROS_ERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", cloudMsg->header.stamp.toSec());
			return;
		}

		int maxLaserScans = scanCloudMaxPoints_;
		if(containNormals)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromROSMsg(*cloudMsg, *pclScan);
			if(pclScan->size() && scanDownsamplingStep_ > 1)
			{
				pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
				maxLaserScans /= scanDownsamplingStep_;
			}
			scan = util3d::laserScanFromPointCloud(*pclScan);
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromROSMsg(*cloudMsg, *pclScan);
			if(pclScan->size() && scanDownsamplingStep_ > 1)
			{
				pclScan = util3d::downsample(pclScan, scanDownsamplingStep_);
				maxLaserScans /= scanDownsamplingStep_;
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

		rtabmap::SensorData data(
				LaserScan::backwardCompatibility(scan, maxLaserScans, 0, localScanTransform),
				cv::Mat(),
				cv::Mat(),
				CameraModel(),
				0,
				rtabmap_ros::timestampFromROS(cloudMsg->header.stamp));

		this->processData(data, cloudMsg->header.stamp);
	}

protected:
	virtual void flushCallbacks()
	{
		// flush callbacks
	}

private:
	ros::Subscriber scan_sub_;
	ros::Subscriber cloud_sub_;
	int scanCloudMaxPoints_;
	int scanDownsamplingStep_;
	double scanVoxelSize_;
	int scanNormalK_;
	double scanNormalRadius_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ICPOdometry, nodelet::Nodelet);

}
