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
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rtabmap_conversions/MsgConversion.h"

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

class RGBDICPOdometry : public OdometryROS
{
public:
	RGBDICPOdometry() :
		OdometryROS(false, true, true),
		approxScanSync_(0),
		exactScanSync_(0),
		approxCloudSync_(0),
		exactCloudSync_(0),
		queueSize_(1),
		syncQueueSize_(5),
		keepColor_(false),
		scanCloudMaxPoints_(0),
		scanVoxelSize_(0.0),
		scanNormalK_(0),
		scanNormalRadius_(0.0)
	{
	}

	virtual ~RGBDICPOdometry()
	{
		if(approxScanSync_)
		{
			delete approxScanSync_;
		}
		if(exactScanSync_)
		{
			delete exactScanSync_;
		}
		if(approxCloudSync_)
		{
			delete approxCloudSync_;
		}
		if(exactCloudSync_)
		{
			delete exactCloudSync_;
		}
	}

private:

	virtual void onOdomInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		bool approxSync = true;
		bool subscribeScanCloud = false;
		double approxSyncMaxInterval = 0.0;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("approx_sync_max_interval", approxSyncMaxInterval, approxSyncMaxInterval);
		pnh.param("topic_queue_size", queueSize_, queueSize_);
		if(pnh.hasParam("queue_size") && !pnh.hasParam("sync_queue_size"))
		{
			pnh.param("queue_size", syncQueueSize_, syncQueueSize_);
			ROS_WARN("Parameter \"queue_size\" has been renamed "
					"to \"sync_queue_size\" and will be removed "
					"in future versions! The value (%d) is still copied to "
					"\"sync_queue_size\".", syncQueueSize_);
		}
		else
		{
			pnh.param("sync_queue_size", syncQueueSize_, syncQueueSize_);
		}
		pnh.param("subscribe_scan_cloud", subscribeScanCloud, subscribeScanCloud);
		pnh.param("scan_cloud_max_points",  scanCloudMaxPoints_, scanCloudMaxPoints_);
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);
		pnh.param("scan_normal_k", scanNormalK_, scanNormalK_);
		if(pnh.hasParam("scan_cloud_normal_k") && !pnh.hasParam("scan_normal_k"))
		{
			ROS_WARN("rtabmap: Parameter \"scan_cloud_normal_k\" has been renamed to \"scan_normal_k\". "
					"The value is still used. Use \"scan_normal_k\" to avoid this warning.");
			pnh.param("scan_cloud_normal_k", scanNormalK_, scanNormalK_);
		}
		pnh.param("scan_normal_radius", scanNormalRadius_, scanNormalRadius_);
		pnh.param("keep_color", keepColor_, keepColor_);

		NODELET_INFO("RGBDIcpOdometry: approx_sync           = %s", approxSync?"true":"false");
		if(approxSync)
			NODELET_INFO("RGBDIcpOdometry: approx_sync_max_interval = %f", approxSyncMaxInterval);
		NODELET_INFO("RGBDIcpOdometry: topic_queue_size      = %d", queueSize_);
		NODELET_INFO("RGBDIcpOdometry: sync_queue_size       = %d", syncQueueSize_);
		NODELET_INFO("RGBDIcpOdometry: subscribe_scan_cloud  = %s", subscribeScanCloud?"true":"false");
		NODELET_INFO("RGBDIcpOdometry: scan_cloud_max_points = %d", scanCloudMaxPoints_);
		NODELET_INFO("RGBDIcpOdometry: scan_voxel_size       = %f", scanVoxelSize_);
		NODELET_INFO("RGBDIcpOdometry: scan_normal_k         = %d", scanNormalK_);
		NODELET_INFO("RGBDIcpOdometry: scan_normal_radius    = %f", scanNormalRadius_);
		NODELET_INFO("RGBDIcpOdometry: keep_color            = %s", keepColor_?"true":"false");

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		image_mono_sub_.subscribe(rgb_it, rgb_nh.resolveName("image"), queueSize_, hintsRgb);
		image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image"), queueSize_, hintsDepth);
		info_sub_.subscribe(rgb_nh, "camera_info", queueSize_);

		std::string subscribedTopicsMsg;
		if(subscribeScanCloud)
		{
			cloud_sub_.subscribe(nh, "scan_cloud", queueSize_);
			if(approxSync)
			{
				approxCloudSync_ = new message_filters::Synchronizer<MyApproxCloudSyncPolicy>(MyApproxCloudSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, cloud_sub_);
				if(approxSyncMaxInterval > 0.0)
					approxCloudSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
				approxCloudSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackCloud, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
			}
			else
			{
				exactCloudSync_ = new message_filters::Synchronizer<MyExactCloudSyncPolicy>(MyExactCloudSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, cloud_sub_);
				exactCloudSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackCloud, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
			}

			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s,\n   %s,\n   %s, \n   %s",
					getName().c_str(),
					approxSync?"approx":"exact",
					approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
					image_mono_sub_.getTopic().c_str(),
					image_depth_sub_.getTopic().c_str(),
					info_sub_.getTopic().c_str(),
					cloud_sub_.getTopic().c_str());
		}
		else
		{
			scan_sub_.subscribe(nh, "scan", queueSize_);
			if(approxSync)
			{
				approxScanSync_ = new message_filters::Synchronizer<MyApproxScanSyncPolicy>(MyApproxScanSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, scan_sub_);
				if(approxSyncMaxInterval > 0.0)
					approxScanSync_->setMaxIntervalDuration(ros::Duration(approxSyncMaxInterval));
				approxScanSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackScan, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
			}
			else
			{
				exactScanSync_ = new message_filters::Synchronizer<MyExactScanSyncPolicy>(MyExactScanSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, scan_sub_);
				exactScanSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackScan, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
			}

			subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync%s):\n   %s \\\n   %s \\\n   %s \\\n   %s",
					getName().c_str(),
					approxSync?"approx":"exact",
					approxSync&&approxSyncMaxInterval!=0.0?uFormat(", max interval=%fs", approxSyncMaxInterval).c_str():"",
					image_mono_sub_.getTopic().c_str(),
					image_depth_sub_.getTopic().c_str(),
					info_sub_.getTopic().c_str(),
					scan_sub_.getTopic().c_str());
		}
		initDiagnosticMsg(subscribedTopicsMsg, approxSync);
	}

	virtual void updateParameters(ParametersMap & parameters)
	{
		//make sure we are using Reg/Strategy=0
		ParametersMap::iterator iter = parameters.find(Parameters::kRegStrategy());
		if(iter != parameters.end() && iter->second.compare("2") != 0)
		{
			ROS_WARN("RGBDICP odometry works only with \"Reg/Strategy\"=2. Ignoring value %s.", iter->second.c_str());
		}
		uInsert(parameters, ParametersPair(Parameters::kRegStrategy(), "2"));
	}

	void callbackScan(
			const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo,
			const sensor_msgs::LaserScanConstPtr& scanMsg)
	{
		sensor_msgs::PointCloud2ConstPtr cloudMsg;
		callbackCommon(image, depth, cameraInfo, scanMsg, cloudMsg);
	}

	void callbackCloud(
			const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo,
			const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
	{
		sensor_msgs::LaserScanConstPtr scanMsg;
		callbackCommon(image, depth, cameraInfo, scanMsg, cloudMsg);
	}

	void callbackCommon(
			const sensor_msgs::ImageConstPtr& image,
			const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo,
			const sensor_msgs::LaserScanConstPtr& scanMsg,
			const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
	{
		if(!this->isPaused())
		{
			if(!(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::BGRA8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::RGBA8) == 0 ||
				 image->encoding.compare(sensor_msgs::image_encodings::BAYER_GRBG8) == 0) ||
			   !(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
				 depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0 ||
				 depth->encoding.compare(sensor_msgs::image_encodings::MONO16)==0))
			{
				NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8,rgba8,bgra8 (mono8 "
						  "recommended) and image_depth=16UC1,32FC1,mono16. Types detected: %s %s",
						image->encoding.c_str(), depth->encoding.c_str());
				return;
			}

			// use the highest stamp to make sure that there will be no future interpolation required when synchronized with another node
			ros::Time stamp = image->header.stamp > depth->header.stamp? image->header.stamp : depth->header.stamp;
			if(scanMsg.get() != 0)
			{
				if(stamp < scanMsg->header.stamp)
				{
					stamp = scanMsg->header.stamp;
				}
			}
			else if(cloudMsg.get() != 0)
			{
				if(stamp < cloudMsg->header.stamp)
				{
					stamp = cloudMsg->header.stamp;
				}
			}

			Transform localTransform = rtabmap_conversions::getTransform(this->frameId(), image->header.frame_id, stamp, this->tfListener(), this->waitForTransformDuration());
			if(localTransform.isNull())
			{
				return;
			}

			double stampDiff = fabs(image->header.stamp.toSec() - depth->header.stamp.toSec());
			if(stampDiff > 0.010)
			{
				NODELET_WARN("The time difference between rgb and depth frames is "
						"high (diff=%fs, rgb=%fs, depth=%fs). You may want "
						"to set approx_sync_max_interval lower than 0.01s to reject spurious bad synchronizations or use "
						"approx_sync=false if streams have all the exact same timestamp.",
						stampDiff,
						image->header.stamp.toSec(),
						depth->header.stamp.toSec());
			}

			if(image->data.size() && depth->data.size() && cameraInfo->K[4] != 0)
			{
				rtabmap::CameraModel rtabmapModel = rtabmap_conversions::cameraModelFromROS(*cameraInfo, localTransform);
				cv_bridge::CvImagePtr ptrImage = cv_bridge::toCvCopy(image,
						image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0 ||
						image->encoding.compare(sensor_msgs::image_encodings::MONO8)==0?"":
								keepColor_ && image->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0?"bgr8":"mono8");
				cv_bridge::CvImagePtr ptrDepth = cv_bridge::toCvCopy(depth);

				LaserScan scan;
				Transform localScanTransform = Transform::getIdentity();
				int maxLaserScans = 0;
				if(scanMsg.get() != 0)
				{
					// make sure the frame of the laser is updated too
					localScanTransform = rtabmap_conversions::getTransform(this->frameId(),
							scanMsg->header.frame_id,
							scanMsg->header.stamp + ros::Duration().fromSec(scanMsg->ranges.size()*scanMsg->time_increment),
							this->tfListener(),
							this->waitForTransformDuration());
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

					maxLaserScans = (int)scanMsg->ranges.size();
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
				}
				else if(cloudMsg.get() != 0)
				{
					UASSERT_MSG(cloudMsg->data.size() == cloudMsg->row_step*cloudMsg->height,
							uFormat("data=%d row_step=%d height=%d", cloudMsg->data.size(), cloudMsg->row_step, cloudMsg->height).c_str());


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
					localScanTransform = rtabmap_conversions::getTransform(this->frameId(), cloudMsg->header.frame_id, cloudMsg->header.stamp, this->tfListener(), this->waitForTransformDuration());
					if(localScanTransform.isNull())
					{
						ROS_ERROR("TF of received scan cloud at time %fs is not set, aborting rtabmap update.", cloudMsg->header.stamp.toSec());
						return;
					}

					maxLaserScans = scanCloudMaxPoints_;
					if(containNormals)
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr pclScan(new pcl::PointCloud<pcl::PointNormal>);
						pcl::fromROSMsg(*cloudMsg, *pclScan);
						if(!pclScan->is_dense)
						{
							pclScan = util3d::removeNaNNormalsFromPointCloud(pclScan);
						}
						scan = util3d::laserScanFromPointCloud(*pclScan);
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr pclScan(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::fromROSMsg(*cloudMsg, *pclScan);
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
				}

				rtabmap::SensorData data(
						LaserScan(scan,
								scanMsg.get() != 0 || cloudMsg.get() != 0?maxLaserScans:0,
								scanMsg.get() != 0?scanMsg->range_max:0,
								localScanTransform),
						ptrImage->image,
						ptrDepth->image,
						rtabmapModel,
						0,
						rtabmap_conversions::timestampFromROS(stamp));

				std_msgs::Header header;
				header.stamp = stamp;
				header.frame_id = image->header.frame_id;
				this->processData(data, header);
			}
		}
	}

protected:
	virtual void flushCallbacks()
	{
		// flush callbacks
		if(approxScanSync_)
		{
			delete approxScanSync_;
			approxScanSync_ = new message_filters::Synchronizer<MyApproxScanSyncPolicy>(MyApproxScanSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, scan_sub_);
			approxScanSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackScan, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
		if(exactScanSync_)
		{
			delete exactScanSync_;
			exactScanSync_ = new message_filters::Synchronizer<MyExactScanSyncPolicy>(MyExactScanSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, scan_sub_);
			exactScanSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackScan, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
		if(approxCloudSync_)
		{
			delete approxCloudSync_;
			approxCloudSync_ = new message_filters::Synchronizer<MyApproxCloudSyncPolicy>(MyApproxCloudSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, cloud_sub_);
			approxCloudSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackCloud, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
		if(exactCloudSync_)
		{
			delete exactCloudSync_;
			exactCloudSync_ = new message_filters::Synchronizer<MyExactCloudSyncPolicy>(MyExactCloudSyncPolicy(syncQueueSize_), image_mono_sub_, image_depth_sub_, info_sub_, cloud_sub_);
			exactCloudSync_->registerCallback(boost::bind(&RGBDICPOdometry::callbackCloud, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4));
		}
	}

private:
	image_transport::SubscriberFilter image_mono_sub_;
	image_transport::SubscriberFilter image_depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan> MyApproxScanSyncPolicy;
	message_filters::Synchronizer<MyApproxScanSyncPolicy> * approxScanSync_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan> MyExactScanSyncPolicy;
	message_filters::Synchronizer<MyExactScanSyncPolicy> * exactScanSync_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MyApproxCloudSyncPolicy;
	message_filters::Synchronizer<MyApproxCloudSyncPolicy> * approxCloudSync_;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MyExactCloudSyncPolicy;
	message_filters::Synchronizer<MyExactCloudSyncPolicy> * exactCloudSync_;
	int queueSize_;
	int syncQueueSize_;
	bool keepColor_;
	int scanCloudMaxPoints_;
	double scanVoxelSize_;
	int scanNormalK_;
	double scanNormalRadius_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_odom::RGBDICPOdometry, nodelet::Nodelet);

}
