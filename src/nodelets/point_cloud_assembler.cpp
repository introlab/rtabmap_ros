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

#include <rtabmap_ros/point_cloud_assembler.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

namespace rtabmap_ros
{

PointCloudAssembler::PointCloudAssembler(const rclcpp::NodeOptions & options) :
	Node("pointcloud_to_depthimage", options),
	warningThread_(0),
	callbackCalled_(false),
	exactSync_(0),
	maxClouds_(0),
	skipClouds_(0),
	cloudsSkipped_(0),
	assemblingTime_(0),
	waitForTransformDuration_(0.1),
	rangeMin_(0),
	rangeMax_(0),
	voxelSize_(0),
	fixedFrameId_("odom")
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	int queueSize = 5;

	queueSize = this->declare_parameter("queue_size", queueSize);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	maxClouds_ = this->declare_parameter("max_clouds", maxClouds_);
	assemblingTime_ = this->declare_parameter("assembling_time", assemblingTime_);
	skipClouds_ = this->declare_parameter("skip_clouds", skipClouds_);
	waitForTransformDuration_ = this->declare_parameter("wait_for_transform", waitForTransformDuration_);
	rangeMin_ = this->declare_parameter("range_min", rangeMin_);
	rangeMax_ = this->declare_parameter("range_max", rangeMax_);
	voxelSize_ = this->declare_parameter("voxel_size", voxelSize_);
	UASSERT(maxClouds_>0 || assemblingTime_ >0.0);

	cloudsSkipped_ = skipClouds_;

	cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("assembled_cloud", 1);

	std::string subscribedTopicsMsg;
	if(!fixedFrameId_.empty())
	{
		cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS(), std::bind(&PointCloudAssembler::callbackCloud, this, std::placeholders::_1));
		subscribedTopicsMsg = uFormat("\n%s subscribed to %s",
							get_name(),
							cloudSub_->get_topic_name());
	}
	else
	{
		syncCloudSub_.subscribe(this, "cloud", rmw_qos_profile_sensor_data);
		syncOdomSub_.subscribe(this, "odom", rmw_qos_profile_sensor_data);
		exactSync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(queueSize), syncCloudSub_, syncOdomSub_);
		exactSync_->registerCallback(std::bind(&rtabmap_ros::PointCloudAssembler::callbackCloudOdom, this, std::placeholders::_1, std::placeholders::_2));
		subscribedTopicsMsg = uFormat("\n%s subscribed to (exact sync):\n   %s,\n   %s",
							get_name(),
							syncCloudSub_.getTopic().c_str(),
							syncOdomSub_.getTopic().c_str());

		warningThread_ = new std::thread([&](){
				rclcpp::Rate r(1.0/5.0);
				while(!callbackCalled_)
				{
					r.sleep();
					if(!callbackCalled_)
					{
						RCLCPP_WARN(this->get_logger(),
								"%s: Did not receive data since 5 seconds! Make sure the input topics are "
								"published (\"$ rostopic hz my_topic\") and the timestamps in their "
								"header are set. %s",
								get_name(),
								subscribedTopicsMsg.c_str());
					}
				}
			});

	}

	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg.c_str());
}

PointCloudAssembler::~PointCloudAssembler()
{
	delete exactSync_;

	if(warningThread_)
	{
		callbackCalled_=true;
		warningThread_->join();
		delete warningThread_;
	}
}

void PointCloudAssembler::callbackCloudOdom(
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg,
		const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg)
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
		RCLCPP_WARN(this->get_logger(), "Reseting point cloud assembler as null odometry has been received.");
		clouds_.clear();
	}
}

void PointCloudAssembler::callbackCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg)
{
	if(cloudPub_->get_subscription_count())
	{
		if(skipClouds_<=0 || cloudsSkipped_ >= skipClouds_)
		{
			cloudsSkipped_ = 0;

			sensor_msgs::msg::PointCloud2::SharedPtr cpy(new sensor_msgs::msg::PointCloud2);
			*cpy = *cloudMsg;
			clouds_.push_back(cpy);

			if(  ((int)clouds_.size() >= maxClouds_ && maxClouds_ > 0)
				||
				 (timestampFromROS((*cpy).header.stamp) >= timestampFromROS(clouds_[0]->header.stamp) + assemblingTime_ && assemblingTime_ > 0.0))
			{
				pcl::PCLPointCloud2Ptr assembled(new pcl::PCLPointCloud2);
				pcl_conversions::toPCL(*clouds_.back(), *assembled);

				for(size_t i=0; i<clouds_.size()-1; ++i)
				{
					rtabmap::Transform t = rtabmap_ros::getTransform(
							clouds_[i]->header.frame_id, //sourceTargetFrame
							fixedFrameId_, //fixedFrame
							clouds_[i]->header.stamp, //stampSource
							clouds_.back()->header.stamp, //stampTarget
							*tfBuffer_,
							waitForTransformDuration_);

					if(t.isNull())
					{
						RCLCPP_ERROR(this->get_logger(), "Cloud not transform all clouds! Resetting...");
						clouds_.clear();
						return;
					}

					pcl::PCLPointCloud2Ptr assembledTmp(new pcl::PCLPointCloud2);
					if(rangeMin_ > 0.0 || rangeMax_ > 0.0)
					{
						pcl::PCLPointCloud2 output2;
						pcl_conversions::toPCL(*clouds_[i], output2);
						rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(output2);
						if(rangeMin_ > 0.0 || rangeMax_ > 0.0)
						{
							scan = rtabmap::util3d::rangeFiltering(scan, rangeMin_, rangeMax_);
						}
						pcl::concatenatePointCloud(*assembled, *rtabmap::util3d::laserScanToPointCloud2(scan, t), *assembledTmp);
					}
					else
					{
						sensor_msgs::msg::PointCloud2 output;
						rtabmap_ros::transformPointCloud(t.toEigen4f(), *clouds_[i], output);
						pcl::PCLPointCloud2 output2;
						pcl_conversions::toPCL(output, output2);
						pcl::concatenatePointCloud(*assembled, output2, *assembledTmp);
					}

					assembled = assembledTmp;
				}

				sensor_msgs::msg::PointCloud2 rosCloud;
				if(voxelSize_>0.0)
				{
					pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
					filter.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
					filter.setInputCloud(assembled);
					pcl::PCLPointCloud2Ptr output(new pcl::PCLPointCloud2);
					filter.filter(*output);
					pcl_conversions::moveFromPCL(*output, rosCloud);
				}
				else
				{
					pcl_conversions::moveFromPCL(*assembled, rosCloud);
				}
				rosCloud.header = cloudMsg->header;
				cloudPub_->publish(rosCloud);
				clouds_.clear();
			}
		}
		else
		{
			++cloudsSkipped_;
		}
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::PointCloudAssembler)
