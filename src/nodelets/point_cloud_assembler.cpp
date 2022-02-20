/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/OdomInfo.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

//ADC
#include <sensor_msgs/Range.h>

namespace rtabmap_ros
{

/**
 * This nodelet can assemble a number of clouds (max_clouds) coming
 * from the same sensor, taking into account the displacement of the robot based on
 * fixed_frame_id, then publish the resulting cloud.
 * If fixed_frame_id is set to "" (empty), the nodelet will subscribe to
 * an odom topic that should have the exact same stamp than to input cloud.
 * The output cloud has the same stamp and frame than the last assembled cloud.
 */
class PointCloudAssembler : public nodelet::Nodelet
{
public:
	PointCloudAssembler() :
		warningThread_(0),
		callbackCalled_(false),
		exactSync_(0),
		exactInfoSync_(0),
		maxClouds_(0),
		assemblingTime_(0),
		skipClouds_(0),
		cloudsSkipped_(0),
		circularBuffer_(false),
		waitForTransformDuration_(0.1),
		rangeMin_(0),
		rangeMax_(0),
		voxelSize_(0),
		noiseRadius_(0),
		noiseMinNeighbors_(5),
		fixedFrameId_("odom"),
		frameId_(""),
		//ADC
		use_lidar_topics_(false)
	{}

	virtual ~PointCloudAssembler()
	{
		delete exactSync_;
		delete exactInfoSync_;

		if(warningThread_)
		{
			callbackCalled_=true;
			warningThread_->join();
			delete warningThread_;
		}
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 5;
		bool subscribeOdomInfo = false;

		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("max_clouds", maxClouds_, maxClouds_);
		pnh.param("assembling_time", assemblingTime_, assemblingTime_);
		pnh.param("skip_clouds", skipClouds_, skipClouds_);
		pnh.param("circular_buffer", circularBuffer_, circularBuffer_);
		pnh.param("wait_for_transform_duration", waitForTransformDuration_, waitForTransformDuration_);
		pnh.param("range_min", rangeMin_, rangeMin_);
		pnh.param("range_max", rangeMax_, rangeMax_);
		pnh.param("voxel_size", voxelSize_, voxelSize_);
		pnh.param("noise_radius", noiseRadius_, noiseRadius_);
		pnh.param("noise_min_neighbors", noiseMinNeighbors_, noiseMinNeighbors_);
		pnh.param("subscribe_odom_info", subscribeOdomInfo, subscribeOdomInfo);
		ROS_ASSERT(maxClouds_>0 || assemblingTime_ >0.0);
		//ADC
		pnh.param("use_lidar_topics", use_lidar_topics_, use_lidar_topics_);
	    std::vector<std::string> lidar_topics;
  		pnh.param("lidar_topics", lidar_topics, lidar_topics);
		lidarSub_.resize(lidar_topics.size());


		ROS_INFO("%s: queue_size=%d", getName().c_str(), queueSize);
		ROS_INFO("%s: fixed_frame_id=%s", getName().c_str(), fixedFrameId_.c_str());
		ROS_INFO("%s: frame_id=%s", getName().c_str(), frameId_.c_str());
		ROS_INFO("%s: max_clouds=%d", getName().c_str(), maxClouds_);
		ROS_INFO("%s: assembling_time=%fs", getName().c_str(), assemblingTime_);
		ROS_INFO("%s: skip_clouds=%d", getName().c_str(), skipClouds_);
		ROS_INFO("%s: circular_buffer=%s", getName().c_str(), circularBuffer_?"true":"false");
		ROS_INFO("%s: wait_for_transform_duration=%f", getName().c_str(), waitForTransformDuration_);
		ROS_INFO("%s: range_min=%f", getName().c_str(), rangeMin_);
		ROS_INFO("%s: range_max=%f", getName().c_str(), rangeMax_);
		ROS_INFO("%s: voxel_size=%fm", getName().c_str(), voxelSize_);
		ROS_INFO("%s: noise_radius=%fm", getName().c_str(), noiseRadius_);
		ROS_INFO("%s: noise_min_neighbors=%d", getName().c_str(), noiseMinNeighbors_);
		//ADC
		ROS_INFO("%s: use_lidar_topics=%d", getName().c_str(), use_lidar_topics_);

		cloudsSkipped_ = skipClouds_;

		std::string subscribedTopicsMsg;
		if(!fixedFrameId_.empty())
		{
			if(!use_lidar_topics_)
			{
				ROS_INFO("ADC: fixedFrameId != empty");
				cloudSub_ = nh.subscribe("cloud", queueSize, &PointCloudAssembler::callbackCloud, this);
				subscribedTopicsMsg = uFormat("\n%s subscribed to %s",
									getName().c_str(),
									cloudSub_.getTopic().c_str());
			}
			else
			{
				if (lidar_topics.size() == 0)
				{
					ROS_WARN("Missing lidar topics!");
				}
				for (int i = 0; i < lidar_topics.size(); ++i)
				{
					ROS_INFO("Subscribing to lidar_topics `%s`", lidar_topics[i].c_str());
					lidarSub_[i] = nh.subscribe(lidar_topics[i], queueSize, &PointCloudAssembler::callbackLidar, this);  
				}
			}
		}
		else if(subscribeOdomInfo)
		{
			syncCloudSub_.subscribe(nh, "cloud", 1);
			syncOdomSub_.subscribe(nh, "odom", 1);
			syncOdomInfoSub_.subscribe(nh, "odom_info", 1);
			exactInfoSync_ = new message_filters::Synchronizer<syncInfoPolicy>(syncInfoPolicy(queueSize), syncCloudSub_, syncOdomSub_, syncOdomInfoSub_);
			exactInfoSync_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAssembler::callbackCloudOdomInfo, this, _1, _2, _3));
			subscribedTopicsMsg = uFormat("\n%s subscribed to (exact sync):\n   %s,\n   %s",
								getName().c_str(),
								syncCloudSub_.getTopic().c_str(),
								syncOdomSub_.getTopic().c_str(),
								syncOdomInfoSub_.getTopic().c_str());

			warningThread_ = new boost::thread(boost::bind(&PointCloudAssembler::warningLoop, this, subscribedTopicsMsg));
		}
		else
		{
			syncCloudSub_.subscribe(nh, "cloud", 1);
			syncOdomSub_.subscribe(nh, "odom", 1);
			exactSync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(queueSize), syncCloudSub_, syncOdomSub_);
			exactSync_->registerCallback(boost::bind(&rtabmap_ros::PointCloudAssembler::callbackCloudOdom, this, _1, _2));
			subscribedTopicsMsg = uFormat("\n%s subscribed to (exact sync):\n   %s,\n   %s",
								getName().c_str(),
								syncCloudSub_.getTopic().c_str(),
								syncOdomSub_.getTopic().c_str());

			warningThread_ = new boost::thread(boost::bind(&PointCloudAssembler::warningLoop, this, subscribedTopicsMsg));
		}

		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_cloud", 1);

		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	}

	void callbackCloudOdom(
			const sensor_msgs::PointCloud2ConstPtr & cloudMsg,
			const nav_msgs::OdometryConstPtr & odomMsg)
	{
		callbackCalled_ = true;
		rtabmap::Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
		if(!odom.isNull())
		{
			fixedFrameId_ = odomMsg->header.frame_id;
			callbackCloud(cloudMsg);
		}
		else
		{
			NODELET_WARN("Reseting point cloud assembler as null odometry has been received.");
			clouds_.clear();
		}
	}

	void callbackCloudOdomInfo(
			const sensor_msgs::PointCloud2ConstPtr & cloudMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const rtabmap_ros::OdomInfoConstPtr & odomInfoMsg)
	{
		callbackCalled_ = true;
		rtabmap::Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
		if(!odom.isNull())
		{
			if(odomInfoMsg->keyFrameAdded)
			{
				fixedFrameId_ = odomMsg->header.frame_id;
				callbackCloud(cloudMsg);
			}
			else
			{
				NODELET_INFO("Skipping non keyframe...");
			}
		}
		else
		{
			NODELET_WARN("Reseting point cloud assembler as null odometry has been received.");
			clouds_.clear();
		}
	}

	void callbackLidar(const sensor_msgs::RangeConstPtr & lidarMsg)
	{
		if(lidarMsg->range > 0.1)
		{
			callbackCalled_ = true;

			//transform lidar to PC
			pcl::PointCloud<pcl::PointXYZ>* pcl_cloud = new pcl::PointCloud<pcl::PointXYZ>;
			pcl_cloud->push_back (pcl::PointXYZ (lidarMsg->range, 0, 0));

			//const sensor_msgs::PointCloudConstPtr pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr assembledGround_(pcl);
			sensor_msgs::PointCloud2::Ptr pc2(new sensor_msgs::PointCloud2);

			pcl::toROSMsg(*pcl_cloud, *pc2);
			pc2->header = lidarMsg->header;
			callbackCloud(pc2);
		}

	}



	void callbackCloud(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		if(cloudPub_.getNumSubscribers())
		{
			UASSERT_MSG(cloudMsg->data.size() == cloudMsg->row_step*cloudMsg->height,
					uFormat("data=%d row_step=%d height=%d", cloudMsg->data.size(), cloudMsg->row_step, cloudMsg->height).c_str());

			if(skipClouds_<=0 || cloudsSkipped_ >= skipClouds_)
			{
				cloudsSkipped_ = 0;
				rtabmap::Transform t = rtabmap_ros::getTransform(
						fixedFrameId_, //fromFrame
						cloudMsg->header.frame_id, //toFrame
						cloudMsg->header.stamp,
						*tfListener_,
						waitForTransformDuration_);

				if(t.isNull())
				{
					ROS_WARN("Cloud not use transform! Ignoring...");
					//clouds_.clear();    //comment out to prevent PC reset
					return;
				}

				pcl::PCLPointCloud2::Ptr newCloud(new pcl::PCLPointCloud2);
				if(rangeMin_ > 0.0 || rangeMax_ > 0.0 || voxelSize_ > 0.0f)
				{
					pcl_conversions::toPCL(*cloudMsg, *newCloud);
					rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*newCloud);
					scan = rtabmap::util3d::commonFiltering(scan, 1, rangeMin_, rangeMax_, voxelSize_);
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
					std::uint64_t stamp = newCloud->header.stamp;
#else
					pcl::uint64_t stamp = newCloud->header.stamp;
#endif
					newCloud = rtabmap::util3d::laserScanToPointCloud2(scan, t);
					newCloud->header.stamp = stamp;
				}
				else
				{
					sensor_msgs::PointCloud2 output;
					pcl_ros::transformPointCloud(t.toEigen4f(), *cloudMsg, output);
					pcl_conversions::toPCL(output, *newCloud);
				}

				if(!newCloud->is_dense)
				{
					// remove nans
					newCloud = rtabmap::util3d::removeNaNFromPointCloud(newCloud);
				}

				clouds_.push_back(newCloud);

#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
				bool reachedMaxSize =
						((int)clouds_.size() >= maxClouds_ && maxClouds_ > 0)
						||
						((*newCloud).header.stamp >= clouds_.front()->header.stamp + static_cast<std::uint64_t>(assemblingTime_*1000000.0) && assemblingTime_ > 0.0);
#else
				bool reachedMaxSize =
						((int)clouds_.size() >= maxClouds_ && maxClouds_ > 0)
						||
						((*newCloud).header.stamp >= clouds_.front()->header.stamp + static_cast<pcl::uint64_t>(assemblingTime_*1000000.0) && assemblingTime_ > 0.0);
#endif

				if( circularBuffer_ || reachedMaxSize )
				{
					pcl::PCLPointCloud2Ptr assembled(new pcl::PCLPointCloud2);
					for(std::list<pcl::PCLPointCloud2::Ptr>::iterator iter=clouds_.begin(); iter!=clouds_.end(); ++iter)
					{
						if(assembled->data.empty())
						{
							*assembled = *(*iter);
						}
						else
						{
							pcl::PCLPointCloud2Ptr assembledTmp(new pcl::PCLPointCloud2);
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
							pcl::concatenate(*assembled, *(*iter), *assembledTmp);
#else
							pcl::concatenatePointCloud(*assembled, *(*iter), *assembledTmp);
#endif
							//Make sure row_step is the sum of both
							assembledTmp->row_step = assembled->row_step + (*iter)->row_step;
							assembled = assembledTmp;
						}
					}

					sensor_msgs::PointCloud2 rosCloud;
					if(voxelSize_>0.0)
					{
						pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
						filter.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
						filter.setInputCloud(assembled);
						pcl::PCLPointCloud2Ptr output(new pcl::PCLPointCloud2);
						filter.filter(*output);
						assembled = output;
					}
					if(noiseRadius_>0.0 && noiseMinNeighbors_>0)
					{
						pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter;
						filter.setRadiusSearch(noiseRadius_);
						filter.setMinNeighborsInRadius(noiseMinNeighbors_);
						filter.setInputCloud(assembled);
						pcl::PCLPointCloud2Ptr output(new pcl::PCLPointCloud2);
						filter.filter(*output);
						assembled = output;
					}

					pcl_conversions::moveFromPCL(*assembled, rosCloud);
					if(!frameId_.empty())
					{
						// transform in target frame_id instead of sensor frame
						t = rtabmap_ros::getTransform(
								fixedFrameId_, //fromFrame
								frameId_, //toFrame
								cloudMsg->header.stamp,
								*tfListener_,
								waitForTransformDuration_);
						if(t.isNull())
						{
							ROS_ERROR("Cloud not transform back assembled clouds in target frame \"%s\"! Ignoring...", frameId_.c_str());
							//clouds_.clear();   // don;t clear the cloud
							return;
						}
					}
					pcl_ros::transformPointCloud(t.toEigen4f().inverse(), rosCloud, rosCloud);


					rosCloud.header = cloudMsg->header;
					if(!frameId_.empty())
					{
						rosCloud.header.frame_id = frameId_;
					}
					ros::Duration rate_sleep = ros::Duration(0.1);
					if((rosCloud.header.stamp - cloudPub_timestamp) > rate_sleep )
					{
						cloudPub_timestamp = rosCloud.header.stamp;
						cloudPub_.publish(rosCloud);
					}
					if(circularBuffer_)
					{
						if(reachedMaxSize)
						{
							clouds_.pop_front();
						}
					}
					else
					{
						clouds_.clear();
					}
				}
			}
			else
			{
				++cloudsSkipped_;
			}
		}
	}

	void warningLoop(const std::string & subscribedTopicsMsg)
	{
		ros::Duration r(5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s",
						getName().c_str(),
						subscribedTopicsMsg.c_str());
			}
		}
	}

private:
	boost::thread * warningThread_;
	bool callbackCalled_;

	ros::Subscriber cloudSub_;
	ros::Publisher cloudPub_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry, rtabmap_ros::OdomInfo> syncInfoPolicy;
	message_filters::Synchronizer<syncPolicy>* exactSync_;
	message_filters::Synchronizer<syncInfoPolicy>* exactInfoSync_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> syncCloudSub_;
	message_filters::Subscriber<nav_msgs::Odometry> syncOdomSub_;
	message_filters::Subscriber<rtabmap_ros::OdomInfo> syncOdomInfoSub_;

	int maxClouds_;
	int skipClouds_;
	int cloudsSkipped_;
	bool circularBuffer_;
	double assemblingTime_;
	double waitForTransformDuration_;
	double rangeMin_;
	double rangeMax_;
	double voxelSize_;
	double noiseRadius_;
	int noiseMinNeighbors_;
	std::string fixedFrameId_;
	std::string frameId_;
	//tf::TransformListener tfListener_;
	ros::Duration not_time = ros::Duration(1000);
	tf::TransformListener* tfListener_  = new tf::TransformListener(not_time, true);

	std::list<pcl::PCLPointCloud2::Ptr> clouds_;

	//ADC
	bool use_lidar_topics_;
	std::vector<ros::Subscriber> lidarSub_;
	sensor_msgs::PointCloud2 LidarCloudMsg;
	ros::Time cloudPub_timestamp = ros::Time(0.0);
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::PointCloudAssembler, nodelet::Nodelet);
}

