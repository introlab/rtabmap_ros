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
#include <pcl/filters/radius_outlier_removal.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/msg/odom_info.hpp>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Version.h>

namespace rtabmap_ros
{

PointCloudAssembler::PointCloudAssembler(const rclcpp::NodeOptions & options) :
	Node("point_cloud_assembler", options),
	warningThread_(0),
	callbackCalled_(false),
	exactSync_(0),
	exactInfoSync_(0),
	maxClouds_(0),
	skipClouds_(0),
	cloudsSkipped_(0),
	circularBuffer_(false),
	linearUpdate_(0),
	angularUpdate_(0),
	assemblingTime_(0),
	waitForTransform_(0.1),
	rangeMin_(0),
	rangeMax_(0),
	voxelSize_(0),
	noiseRadius_(0),
	noiseMinNeighbors_(5),
	removeZ_(false),
	fixedFrameId_("odom"),
	frameId_("")
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	int queueSize = 5;
	int qos = 0;
	bool subscribeOdomInfo = false;

	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	int qosOdom = this->declare_parameter("qos_odom", qos);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	frameId_ = this->declare_parameter("frame_id", frameId_);
	maxClouds_ = this->declare_parameter("max_clouds", maxClouds_);
	assemblingTime_ = this->declare_parameter("assembling_time", assemblingTime_);
	skipClouds_ = this->declare_parameter("skip_clouds", skipClouds_);
	circularBuffer_ = this->declare_parameter("circular_buffer", circularBuffer_);
	linearUpdate_ = this->declare_parameter("linear_update", linearUpdate_);
	angularUpdate_ = this->declare_parameter("angular_update", angularUpdate_);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	rangeMin_ = this->declare_parameter("range_min", rangeMin_);
	rangeMax_ = this->declare_parameter("range_max", rangeMax_);
	voxelSize_ = this->declare_parameter("voxel_size", voxelSize_);
	noiseRadius_ = this->declare_parameter("noise_radius", noiseRadius_);
	noiseMinNeighbors_ = this->declare_parameter("noise_min_neighbors", noiseMinNeighbors_);
	removeZ_ = this->declare_parameter("remove_z", removeZ_);
	subscribeOdomInfo = this->declare_parameter("subscribe_odom_info", subscribeOdomInfo);

	RCLCPP_INFO(this->get_logger(), "%s: queue_size=%d", get_name(), queueSize);
	RCLCPP_INFO(this->get_logger(), "%s: qos=%d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: qos_odom=%d", get_name(), qosOdom);
	RCLCPP_INFO(this->get_logger(), "%s: fixed_frame_id=%s", get_name(), fixedFrameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "%s: frame_id=%s", get_name(), frameId_.c_str());
	RCLCPP_INFO(this->get_logger(), "%s: max_clouds=%d", get_name(), maxClouds_);
	RCLCPP_INFO(this->get_logger(), "%s: assembling_time=%fs", get_name(), assemblingTime_);
	RCLCPP_INFO(this->get_logger(), "%s: skip_clouds=%d", get_name(), skipClouds_);
	RCLCPP_INFO(this->get_logger(), "%s: circular_buffer=%s", get_name(), circularBuffer_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: linear_update=%f m", get_name(), linearUpdate_);
	RCLCPP_INFO(this->get_logger(), "%s: angular_update=%f rad", get_name(), angularUpdate_);
	RCLCPP_INFO(this->get_logger(), "%s: wait_for_transform=%f", get_name(), waitForTransform_);
	RCLCPP_INFO(this->get_logger(), "%s: range_min=%f", get_name(), rangeMin_);
	RCLCPP_INFO(this->get_logger(), "%s: range_max=%f", get_name(), rangeMax_);
	RCLCPP_INFO(this->get_logger(), "%s: voxel_size=%fm", get_name(), voxelSize_);
	RCLCPP_INFO(this->get_logger(), "%s: noise_radius=%fm", get_name(), noiseRadius_);
	RCLCPP_INFO(this->get_logger(), "%s: noise_min_neighbors=%d", get_name(), noiseMinNeighbors_);
	RCLCPP_INFO(this->get_logger(), "%s: remove_z=%s", get_name(), removeZ_?"true":"false");

	if(maxClouds_==0 && assemblingTime_ ==0.0)
	{
		RCLCPP_ERROR(get_logger(), "point_cloud_assembler: max_cloud or assembling_time parameters should be set!");
		exit(-1);
	}

	cloudsSkipped_ = skipClouds_;

	cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("assembled_cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	if(!fixedFrameId_.empty())
	{
		cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&PointCloudAssembler::callbackCloud, this, std::placeholders::_1));
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to %s",
							get_name(),
							cloudSub_->get_topic_name());
	}
	else if(subscribeOdomInfo)
	{
		syncCloudSub_.subscribe(this, "cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
		syncOdomSub_.subscribe(this, "odom", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosOdom).get_rmw_qos_profile());
		syncOdomInfoSub_.subscribe(this, "odom_info", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosOdom).get_rmw_qos_profile());
		exactInfoSync_ = new message_filters::Synchronizer<syncInfoPolicy>(syncInfoPolicy(queueSize), syncCloudSub_, syncOdomSub_, syncOdomInfoSub_);
		exactInfoSync_->registerCallback(std::bind(&rtabmap_ros::PointCloudAssembler::callbackCloudOdomInfo, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (exact sync):\n   %s,\n   %s",
							get_name(),
							syncCloudSub_.getTopic().c_str(),
							syncOdomSub_.getTopic().c_str(),
							syncOdomInfoSub_.getTopic().c_str());
	}
	else
	{
		syncCloudSub_.subscribe(this, "cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
		syncOdomSub_.subscribe(this, "odom", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qosOdom).get_rmw_qos_profile());
		exactSync_ = new message_filters::Synchronizer<syncPolicy>(syncPolicy(queueSize), syncCloudSub_, syncOdomSub_);
		exactSync_->registerCallback(std::bind(&rtabmap_ros::PointCloudAssembler::callbackCloudOdom, this, std::placeholders::_1, std::placeholders::_2));
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (exact sync):\n   %s,\n   %s",
							get_name(),
							syncCloudSub_.getTopic().c_str(),
							syncOdomSub_.getTopic().c_str());
	}

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
						subscribedTopicsMsg_.c_str());
			}
		}
	});

	RCLCPP_INFO(this->get_logger(), "%s", subscribedTopicsMsg_.c_str());
}

PointCloudAssembler::~PointCloudAssembler()
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

sensor_msgs::msg::PointCloud2 removeField(const sensor_msgs::msg::PointCloud2 & input, const std::string & field)
{
	sensor_msgs::msg::PointCloud2 output;
	int offset = 0;
	std::vector<int> inputFieldIndex;
	for(size_t i=0; i<input.fields.size(); ++i)
	{
		if(input.fields[i].name.compare(field) == 0)
		{
			continue;
		}
		else
		{
			sensor_msgs::msg::PointField outputField = input.fields[i];
			outputField.offset = offset;
			offset += outputField.count * sizeOfPointField(outputField.datatype);
			output.fields.push_back(outputField);
			inputFieldIndex.push_back(i);
		}
	}
	output.header = input.header;
	output.height = input.height;
	output.width = input.width;
	output.is_bigendian = input.is_bigendian;
	output.is_dense = input.is_dense;
	output.point_step = offset;
	output.row_step = output.width * output.point_step;
	output.data.resize(output.height*output.row_step);
	int total = output.height*output.width;
	for(int i=0; i<total; ++i)
	{
		// for each point, copy fields
		int oi = i*output.point_step;
		int pi = i*input.point_step;
		for(size_t j=0;j<output.fields.size(); ++j)
		{
			memcpy(&output.data[oi + output.fields[j].offset],
				   &input.data[pi + input.fields[inputFieldIndex[j]].offset],
				   output.fields[j].count * sizeOfPointField(output.fields[j].datatype));
		}
	}
	return output;
}

void PointCloudAssembler::callbackCloudOdomInfo(
			const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg,
			const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg,
			const rtabmap_ros::msg::OdomInfo::ConstSharedPtr odomInfoMsg)
{
	callbackCalled_ = true;
	rtabmap::Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
	if(!odom.isNull())
	{
		if(odomInfoMsg->key_frame_added)
		{
			fixedFrameId_ = odomMsg->header.frame_id;
			callbackCloud(cloudMsg);
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Skipping non keyframe...");
		}
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
		UASSERT_MSG(cloudMsg->data.size() == cloudMsg->row_step*cloudMsg->height,
						uFormat("data=%d row_step=%d height=%d", cloudMsg->data.size(), cloudMsg->row_step, cloudMsg->height).c_str());

		if(skipClouds_<=0 || cloudsSkipped_ >= skipClouds_)
		{
			cloudsSkipped_ = 0;

			rtabmap::Transform pose = rtabmap_ros::getTransform(
					fixedFrameId_, //fromFrame
					cloudMsg->header.frame_id, //toFrame
					cloudMsg->header.stamp,
					*tfBuffer_,
					waitForTransform_);

			if(pose.isNull())
			{
				RCLCPP_ERROR(get_logger(), "Cloud not transform all clouds! Resetting...");
				clouds_.clear();
				return;
			}

			bool isMoving = true;
			if(!previousPose_.isNull() && (linearUpdate_>0 || angularUpdate_>0))
			{
				rtabmap::Transform delta = previousPose_.inverse()*pose;
				float roll, pitch, yaw;
				delta.getEulerAngles(roll, pitch, yaw);
				isMoving = fabs(delta.x()) > linearUpdate_ ||
								fabs(delta.y()) > linearUpdate_ ||
								fabs(delta.z()) > linearUpdate_ ||
								(angularUpdate_>0.0f && (
									fabs(roll) > angularUpdate_ ||
									fabs(pitch) > angularUpdate_ ||
									fabs(yaw) > angularUpdate_));
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
				newCloud = rtabmap::util3d::laserScanToPointCloud2(scan, pose);
				newCloud->header.stamp = stamp;
			}
			else
			{
				sensor_msgs::msg::PointCloud2 output;
				transformPointCloud(pose.toEigen4f(), *cloudMsg, output);
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

				sensor_msgs::msg::PointCloud2 rosCloud;
				if(voxelSize_>0.0)
				{
					// estimate if there would be an overflow
					int x_idx=-1, y_idx=-1, z_idx=-1;
					for (std::size_t d = 0; d < assembled->fields.size (); ++d)
					{
						if (assembled->fields[d].name.compare("x")==0)
							x_idx = d;
						if (assembled->fields[d].name.compare("y")==0)
							y_idx = d;
						if (assembled->fields[d].name.compare("z")==0)
							z_idx = d;
					}
					bool overflow = false;
					if(x_idx>=0 && y_idx>=0 && z_idx>=0) {
						Eigen::Vector4f min_p, max_p;
						pcl::getMinMax3D(assembled, x_idx, y_idx, z_idx, min_p, max_p);
						float inverseVoxelSize = 1.0f/voxelSize_;
						std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverseVoxelSize)+1;
						std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverseVoxelSize)+1;
						std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverseVoxelSize)+1;

						if ((dx*dy*dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()))
						{
							overflow = true;
						}
					}
					if(overflow)
					{
						rtabmap::LaserScan scan = rtabmap::util3d::laserScanFromPointCloud(*assembled);
						scan = rtabmap::util3d::commonFiltering(scan, 1, 0, 0, voxelSize_);
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
						std::uint64_t stamp = assembled->header.stamp;
#else
						pcl::uint64_t stamp = assembled->header.stamp;
#endif
						assembled = rtabmap::util3d::laserScanToPointCloud2(scan);
						assembled->header.stamp = stamp;
					}
					else
					{
						pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
						filter.setLeafSize(voxelSize_, voxelSize_, voxelSize_);
						filter.setInputCloud(assembled);
						pcl::PCLPointCloud2Ptr output(new pcl::PCLPointCloud2);
						filter.filter(*output);
						assembled = output;
					}
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
				rtabmap::Transform t = pose;
				if(!frameId_.empty())
				{
					// transform in target frame_id instead of sensor frame
					t = rtabmap_ros::getTransform(
							fixedFrameId_, //fromFrame
							frameId_, //toFrame
							cloudMsg->header.stamp,
							*tfBuffer_,
							waitForTransform_);
					if(t.isNull())
					{
						RCLCPP_ERROR(this->get_logger(), "Cloud not transform back assembled clouds in target frame \"%s\"! Resetting...", frameId_.c_str());
						clouds_.clear();
						return;
					}
				}
				transformPointCloud(t.toEigen4f().inverse(), rosCloud, rosCloud);

				if(removeZ_)
				{
					rosCloud = removeField(rosCloud, "z");
				}

				rosCloud.header = cloudMsg->header;
				if(!frameId_.empty())
				{
					rosCloud.header.frame_id = frameId_;
				}
				cloudPub_->publish(rosCloud);
				if(circularBuffer_)
				{
					if(!isMoving)
					{
						clouds_.pop_back();
					}
					else
					{
						previousPose_ = pose;
						if(reachedMaxSize)
						{
							clouds_.pop_front();
						}
					}
				}
				else
				{
					clouds_.clear();
					previousPose_.setNull();
				}
			}
			else if(!isMoving)
			{
				clouds_.pop_back();
			}
			else
			{
				previousPose_ = pose;
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
