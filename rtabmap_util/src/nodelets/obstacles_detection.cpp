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

#include <rtabmap_util/obstacles_detection.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <rtabmap_conversions/MsgConversion.h>

#include "rtabmap/utilite/UStl.h"

namespace rtabmap_util
{

ObstaclesDetection::ObstaclesDetection(const rclcpp::NodeOptions & options) :
	Node("obstacles_detection", options),
	frameId_("base_link"),
	waitForTransform_(0.2),
	mapFrameProjection_(rtabmap::Parameters::defaultGridMapFrameProjection()),
	warned_(false)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	frameId_ = this->declare_parameter("frame_id", frameId_);
	mapFrameId_ = this->declare_parameter("map_frame_id", mapFrameId_);
	waitForTransform_ = this->declare_parameter("wait_for_transform", waitForTransform_);
	int qos = 0;
	qos = this->declare_parameter("qos", qos);

	rtabmap::ParametersMap gridParameters = rtabmap::Parameters::getDefaultParameters("Grid");
	for(rtabmap::ParametersMap::iterator iter=gridParameters.begin(); iter!=gridParameters.end(); ++iter)
	{
		std::string vStr = declare_parameter(iter->first, iter->second);
		if(vStr.compare(iter->second) != 0)
		{
			RCLCPP_INFO(this->get_logger(), "obstacles_detection: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
	}

	UASSERT(uContains(gridParameters, rtabmap::Parameters::kGridMapFrameProjection()));
	mapFrameProjection_ = uStr2Bool(gridParameters.at(rtabmap::Parameters::kGridMapFrameProjection()));
	if(mapFrameProjection_ && mapFrameId_.empty())
	{
		RCLCPP_ERROR(this->get_logger(), "obstacles_detection: Parameter \"%s\" is true but map_frame_id is not set!", rtabmap::Parameters::kGridMapFrameProjection().c_str());
	}

	grid_.parseParameters(gridParameters);

	tfBuffer_ = std::make_shared< tf2_ros::Buffer >(this->get_clock());
	tfListener_ = std::make_shared< tf2_ros::TransformListener >(*tfBuffer_);

	groundPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("ground", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	obstaclesPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obstacles", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));
	projObstaclesPub_ = create_publisher<sensor_msgs::msg::PointCloud2>("proj_obstacles", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos));

	cloudSub_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)qos), std::bind(&ObstaclesDetection::callback, this, std::placeholders::_1));
}



void ObstaclesDetection::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloudMsg)
{
	rclcpp::Time time = now();

	if (groundPub_->get_subscription_count() == 0 && obstaclesPub_->get_subscription_count() == 0 && projObstaclesPub_->get_subscription_count() == 0)
	{
		// no one wants the results
		return;
	}

	rtabmap::Transform localTransform = rtabmap::Transform::getIdentity();
	localTransform = rtabmap_conversions::getTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, *tfBuffer_, waitForTransform_);
	if(localTransform.isNull())
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to get transform between %s and %s frames", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
		return;
	}

	rtabmap::Transform pose = rtabmap::Transform::getIdentity();
	if(!mapFrameId_.empty())
	{
		pose = rtabmap_conversions::getTransform(mapFrameId_, frameId_, cloudMsg->header.stamp, *tfBuffer_, waitForTransform_);
		if(pose.isNull())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get transform between %s and %s frames", mapFrameId_.c_str(), frameId_.c_str());
			return;
		}
	}

	UASSERT_MSG(cloudMsg->data.size() == cloudMsg->row_step*cloudMsg->height,
			uFormat("data=%d row_step=%d height=%d", cloudMsg->data.size(), cloudMsg->row_step, cloudMsg->height).c_str());

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloudMsg, *inputCloud);
	if(inputCloud->isOrganized())
	{
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*inputCloud, *inputCloud, indices);
	}
	else if(!inputCloud->is_dense && inputCloud->height == 1)
	{
		if(!warned_)
		{
			RCLCPP_WARN(this->get_logger(), "Detected possible wrong format of point cloud \"%s\", it is "
					"indicated that it is not dense, but there is only one row. "
					"Assuming it is dense... This message will only appear once.", cloudSub_->get_topic_name());
			warned_ = true;
		}
		inputCloud->is_dense = true;
	}

	//Common variables for all strategies
	pcl::IndicesPtr ground, obstacles;
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloudWithoutFlatSurfaces(new pcl::PointCloud<pcl::PointXYZ>);

	if(inputCloud->size())
	{
		inputCloud = rtabmap::util3d::transformPointCloud(inputCloud, localTransform);

		pcl::IndicesPtr flatObstacles(new std::vector<int>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = grid_.segmentCloud<pcl::PointXYZ>(
				inputCloud,
				pcl::IndicesPtr(new std::vector<int>),
				pose,
				cv::Point3f(localTransform.x(), localTransform.y(), localTransform.z()),
				ground,
				obstacles,
				&flatObstacles);

		if(cloud->size() && ((ground.get() && ground->size()) || (obstacles.get() && obstacles->size())))
		{
			if(groundPub_->get_subscription_count() &&
					ground.get() && ground->size())
			{
				pcl::copyPointCloud(*cloud, *ground, *groundCloud);
			}

			if((obstaclesPub_->get_subscription_count() || projObstaclesPub_->get_subscription_count()) &&
					obstacles.get() && obstacles->size())
			{
				// remove flat obstacles from obstacles
				std::set<int> flatObstaclesSet;
				if(projObstaclesPub_->get_subscription_count())
				{
					flatObstaclesSet.insert(flatObstacles->begin(), flatObstacles->end());
				}

				obstaclesCloud->resize(obstacles->size());
				obstaclesCloudWithoutFlatSurfaces->resize(obstacles->size());

				int oi=0;
				for(unsigned int i=0; i<obstacles->size(); ++i)
				{
					obstaclesCloud->points[i] = cloud->at(obstacles->at(i));
					if(flatObstaclesSet.size() == 0 ||
					   flatObstaclesSet.find(obstacles->at(i))==flatObstaclesSet.end())
					{
						obstaclesCloudWithoutFlatSurfaces->points[oi] = obstaclesCloud->points[i];
						obstaclesCloudWithoutFlatSurfaces->points[oi].z = 0;
						++oi;
					}

				}

				obstaclesCloudWithoutFlatSurfaces->resize(oi);
			}

			if(!localTransform.isIdentity() || !pose.isIdentity())
			{
				//transform back in topic frame for 3d clouds and base frame for 2d clouds

				float roll, pitch, yaw;
				pose.getEulerAngles(roll, pitch, yaw);
				rtabmap::Transform t = rtabmap::Transform(0,0, mapFrameProjection_?pose.z():0, roll, pitch, 0);

				if(obstaclesCloudWithoutFlatSurfaces->size() && !pose.isIdentity())
				{
					obstaclesCloudWithoutFlatSurfaces = rtabmap::util3d::transformPointCloud(obstaclesCloudWithoutFlatSurfaces, t.inverse());
				}

				t = (t*localTransform).inverse();
				if(groundCloud->size())
				{
					groundCloud = rtabmap::util3d::transformPointCloud(groundCloud, t);
				}
				if(obstaclesCloud->size())
				{
					obstaclesCloud = rtabmap::util3d::transformPointCloud(obstaclesCloud, t);
				}
			}
		}
	}
	else
	{
		RCLCPP_WARN(this->get_logger(), "obstacles_detection: Input cloud is empty! (%d x %d, is_dense=%d)", cloudMsg->width, cloudMsg->height, cloudMsg->is_dense?1:0);
	}

	if(groundPub_->get_subscription_count())
	{
		sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
		pcl::toROSMsg(*groundCloud, *rosCloud);
		rosCloud->header = cloudMsg->header;

		//publish the message
		groundPub_->publish(std::move(rosCloud));
	}

	if(obstaclesPub_->get_subscription_count())
	{
		sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
		pcl::toROSMsg(*obstaclesCloud, *rosCloud);
		rosCloud->header = cloudMsg->header;

		//publish the message
		obstaclesPub_->publish(std::move(rosCloud));
	}

	if(projObstaclesPub_->get_subscription_count())
	{
		sensor_msgs::msg::PointCloud2::UniquePtr rosCloud(new sensor_msgs::msg::PointCloud2);
		pcl::toROSMsg(*obstaclesCloudWithoutFlatSurfaces, *rosCloud);
		rosCloud->header.stamp = cloudMsg->header.stamp;
		rosCloud->header.frame_id = frameId_;

		//publish the message
		projObstaclesPub_->publish(std::move(rosCloud));
	}

	RCLCPP_DEBUG(this->get_logger(), "Obstacles segmentation time = %f s", (now() - time).seconds());
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::ObstaclesDetection)
