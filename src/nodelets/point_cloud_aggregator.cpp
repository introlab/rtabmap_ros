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

#include <rtabmap_ros/point_cloud_aggregator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d_filtering.h>

namespace rtabmap_ros
{

PointCloudAggregator::PointCloudAggregator(const rclcpp::NodeOptions & options) :
	Node("point_cloud_aggregator", options),
	warningThread_(0),
	callbackCalled_(false),
	exactSync4_(0),
	approxSync4_(0),
	exactSync3_(0),
	approxSync3_(0),
	exactSync2_(0),
	approxSync2_(0),
	waitForTransform_(0.1)
{
	tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
	//	this->get_node_base_interface(),
	//	this->get_node_timers_interface());
	//tfBuffer_->setCreateTimerInterface(timer_interface);
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

	int queueSize = 5;
	int count = 2;
	bool approx=true;
	int qos;
	queueSize = this->declare_parameter("queue_size", queueSize);
	qos = this->declare_parameter("qos", qos);
	frameId_ = this->declare_parameter("frame_id", frameId_);
	fixedFrameId_ = this->declare_parameter("fixed_frame_id", fixedFrameId_);
	approx = this->declare_parameter("approx_sync", approx);
	count = this->declare_parameter("count", count);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);

	cloudPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("combined_cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	cloudSub_1_.subscribe(this, "cloud1", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
	cloudSub_2_.subscribe(this, "cloud2", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());

	std::string subscribedTopicsMsg;
	if(count == 4)
	{
		cloudSub_3_.subscribe(this, "cloud3", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
		cloudSub_4_.subscribe(this, "cloud4", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
		if(approx)
		{
			approxSync4_ = new message_filters::Synchronizer<ApproxSync4Policy>(ApproxSync4Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
			approxSync4_->registerCallback(std::bind(&rtabmap_ros::PointCloudAggregator::clouds4_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}
		else
		{
			exactSync4_ = new message_filters::Synchronizer<ExactSync4Policy>(ExactSync4Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_, cloudSub_4_);
			exactSync4_->registerCallback(std::bind(&rtabmap_ros::PointCloudAggregator::clouds4_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
		}
		subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s,\n   %s",
				get_name(),
				approx?"approx":"exact",
				cloudSub_1_.getTopic().c_str(),
				cloudSub_2_.getTopic().c_str(),
				cloudSub_3_.getTopic().c_str(),
				cloudSub_4_.getTopic().c_str());
	}
	else if(count == 3)
	{
		cloudSub_3_.subscribe(this, "cloud3", rclcpp::QoS(queueSize).reliability((rmw_qos_reliability_policy_t)qos).get_rmw_qos_profile());
		if(approx)
		{
			approxSync3_ = new message_filters::Synchronizer<ApproxSync3Policy>(ApproxSync3Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
			approxSync3_->registerCallback(std::bind(&rtabmap_ros::PointCloudAggregator::clouds3_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		}
		else
		{
			exactSync3_ = new message_filters::Synchronizer<ExactSync3Policy>(ExactSync3Policy(queueSize), cloudSub_1_, cloudSub_2_, cloudSub_3_);
			exactSync3_->registerCallback(std::bind(&rtabmap_ros::PointCloudAggregator::clouds3_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
		}
		subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s",
				this->get_name(),
				approx?"approx":"exact",
				cloudSub_1_.getTopic().c_str(),
				cloudSub_2_.getTopic().c_str(),
				cloudSub_3_.getTopic().c_str());
	}
	else
	{
		if(approx)
		{
			approxSync2_ = new message_filters::Synchronizer<ApproxSync2Policy>(ApproxSync2Policy(queueSize), cloudSub_1_, cloudSub_2_);
			approxSync2_->registerCallback(std::bind(&rtabmap_ros::PointCloudAggregator::clouds2_callback, this, std::placeholders::_1, std::placeholders::_2));
		}
		else
		{
			exactSync2_ = new message_filters::Synchronizer<ExactSync2Policy>(ExactSync2Policy(queueSize), cloudSub_1_, cloudSub_2_);
			exactSync2_->registerCallback(std::bind(&rtabmap_ros::PointCloudAggregator::clouds2_callback, this, std::placeholders::_1, std::placeholders::_2));
		}
		subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s",
				this->get_name(),
				approx?"approx":"exact",
				cloudSub_1_.getTopic().c_str(),
				cloudSub_2_.getTopic().c_str());
	}


	warningThread_ = new std::thread([&](){
		rclcpp::Rate r(1.0/5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				RCLCPP_WARN(this->get_logger(), "%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						this->get_name(),
						approx?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
							subscribedTopicsMsg.c_str());
			}
		}
	});
}

PointCloudAggregator::~PointCloudAggregator()
{
	delete exactSync4_;
	delete approxSync4_;
	delete exactSync3_;
	delete approxSync3_;
	delete exactSync2_;
	delete approxSync2_;

	if(warningThread_)
	{
		callbackCalled_=true;
		warningThread_->join();
		delete warningThread_;
	}
}

void PointCloudAggregator::clouds4_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_3,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_4)
{
	std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
	clouds.push_back(cloudMsg_1);
	clouds.push_back(cloudMsg_2);
	clouds.push_back(cloudMsg_3);
	clouds.push_back(cloudMsg_4);

	combineClouds(clouds);
}
void PointCloudAggregator::clouds3_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_3)
{
	std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
	clouds.push_back(cloudMsg_1);
	clouds.push_back(cloudMsg_2);
	clouds.push_back(cloudMsg_3);

	combineClouds(clouds);
}
void PointCloudAggregator::clouds2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_1,
					 const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg_2)
{
	std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
	clouds.push_back(cloudMsg_1);
	clouds.push_back(cloudMsg_2);

	combineClouds(clouds);
}
void PointCloudAggregator::combineClouds(const std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> & cloudMsgs)
{
	callbackCalled_ = true;
	UASSERT(cloudMsgs.size() > 1);
	if(cloudPub_->get_subscription_count())
	{
		pcl::PCLPointCloud2::Ptr output(new pcl::PCLPointCloud2);

		std::string frameId = frameId_;
		if(!frameId.empty() && frameId.compare(cloudMsgs[0]->header.frame_id) != 0)
		{
			sensor_msgs::msg::PointCloud2 tmp;
			rtabmap::Transform t = rtabmap_ros::getTransform(frameId, cloudMsgs[0]->header.frame_id, cloudMsgs[0]->header.stamp, *tfBuffer_, waitForTransform_);
			if(t.isNull())
			{
				return;
			}
			rtabmap_ros::transformPointCloud(t.toEigen4f(), *cloudMsgs[0], tmp);
			pcl_conversions::toPCL(tmp, *output);
		}
		else
		{
			pcl_conversions::toPCL(*cloudMsgs[0], *output);
			frameId = cloudMsgs[0]->header.frame_id;
		}

		for(unsigned int i=1; i<cloudMsgs.size(); ++i)
		{
			rtabmap::Transform cloudDisplacement;
			if(!fixedFrameId_.empty() &&
			   cloudMsgs[0]->header.stamp != cloudMsgs[i]->header.stamp)
			{
				// approx sync
				cloudDisplacement = rtabmap_ros::getTransform(
						frameId, //sourceTargetFrame
						fixedFrameId_, //fixedFrame
						cloudMsgs[i]->header.stamp, //stampSource
						cloudMsgs[0]->header.stamp, //stampTarget
						*tfBuffer_,
						waitForTransform_);
			}

			pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
			if(frameId.compare(cloudMsgs[i]->header.frame_id) != 0)
			{
				sensor_msgs::msg::PointCloud2 tmp;
				rtabmap::Transform t = rtabmap_ros::getTransform(frameId, cloudMsgs[i]->header.frame_id, cloudMsgs[i]->header.stamp, *tfBuffer_, waitForTransform_);
				rtabmap_ros::transformPointCloud(t.toEigen4f(), *cloudMsgs[i], tmp);
				if(!cloudDisplacement.isNull())
				{
					sensor_msgs::msg::PointCloud2 tmp2;
					rtabmap_ros::transformPointCloud(cloudDisplacement.toEigen4f(), tmp, tmp2);
					pcl_conversions::toPCL(tmp2, *cloud2);
				}
				else
				{
					pcl_conversions::toPCL(tmp, *cloud2);
				}
			}
			else
			{
				if(!cloudDisplacement.isNull())
				{
					sensor_msgs::msg::PointCloud2 tmp;
					rtabmap_ros::transformPointCloud(cloudDisplacement.toEigen4f(), *cloudMsgs[i], tmp);
					pcl_conversions::toPCL(tmp, *cloud2);
				}
				else
				{
					pcl_conversions::toPCL(*cloudMsgs[i], *cloud2);
				}
			}

			if(!cloud2->is_dense)
			{
				// remove nans
				cloud2 = rtabmap::util3d::removeNaNFromPointCloud(cloud2);
			}

			pcl::PCLPointCloud2::Ptr tmp_output(new pcl::PCLPointCloud2);
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
			pcl::concatenate(*output, *cloud2, *tmp_output);
#else
			pcl::concatenatePointCloud(*output, *cloud2, *tmp_output);
#endif
			//Make sure row_step is the sum of both
			tmp_output->row_step = tmp_output->width * tmp_output->point_step;
			output = tmp_output;
		}

		sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
		pcl_conversions::moveFromPCL(*output, *rosCloud);
		rosCloud->header.stamp = cloudMsgs[0]->header.stamp;
		rosCloud->header.frame_id = frameId;
		cloudPub_->publish(std::move(rosCloud));
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_ros::PointCloudAggregator)
