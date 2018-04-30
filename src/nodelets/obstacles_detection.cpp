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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <rtabmap_ros/MsgConversion.h>

#include "rtabmap/core/OccupancyGrid.h"
#include "rtabmap/utilite/UStl.h"

namespace rtabmap_ros
{

class ObstaclesDetection : public nodelet::Nodelet
{
public:
	ObstaclesDetection() :
		frameId_("base_link"),
		waitForTransform_(false),
		mapFrameProjection_(rtabmap::Parameters::defaultGridMapFrameProjection())
	{}

	virtual ~ObstaclesDetection()
	{}

private:

	void parameterMoved(
			ros::NodeHandle & nh,
			const std::string & rosName,
			const std::string & parameterName,
			rtabmap::ParametersMap & parameters)
	{
		if(nh.hasParam(rosName))
		{
			rtabmap::ParametersMap gridParameters = rtabmap::Parameters::getDefaultParameters("Grid");
			rtabmap::ParametersMap::const_iterator iter =gridParameters.find(parameterName);
			if(iter != gridParameters.end())
			{
				NODELET_ERROR("obstacles_detection: Parameter \"%s\" has moved from "
						 "rtabmap_ros to rtabmap library. Use "
						 "parameter \"%s\" instead. The value is still "
						 "copied to new parameter name.",
						 rosName.c_str(),
						 parameterName.c_str());
				std::string type = rtabmap::Parameters::getType(parameterName);
				if(type.compare("float") || type.compare("double"))
				{
					double v = uStr2Double(iter->second);
					nh.getParam(rosName, v);
					parameters.insert(rtabmap::ParametersPair(parameterName, uNumber2Str(v)));
				}
				else if(type.compare("int") || type.compare("unsigned int"))
				{
					int v = uStr2Int(iter->second);
					nh.getParam(rosName, v);
					parameters.insert(rtabmap::ParametersPair(parameterName, uNumber2Str(v)));
				}
				else
				{
					NODELET_ERROR("Not handled type \"%s\" for parameter \"%s\"", type.c_str(), parameterName.c_str());
				}
			}
			else
			{
				NODELET_ERROR("Parameter \"%s\" not found in default parameters.", parameterName.c_str());
			}
		}
	}

	virtual void onInit()
	{
		ROS_DEBUG("_"); // not sure why, but all NODELET_*** log are not shown if a normal ROS_*** is not called!?
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);

		if(pnh.hasParam("optimize_for_close_objects"))
		{
			NODELET_ERROR("\"optimize_for_close_objects\" parameter doesn't exist "
					"anymore. Use rtabmap_ros/obstacles_detection_old nodelet to use "
					"the old interface.");
		}

		rtabmap::ParametersMap parameters;

		// Backward compatibility
		for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=rtabmap::Parameters::getRemovedParameters().begin();
			iter!=rtabmap::Parameters::getRemovedParameters().end();
			++iter)
		{
			std::string vStr;
			bool vBool;
			int vInt;
			double vDouble;
			std::string paramValue;
			if(pnh.getParam(iter->first, vStr))
			{
				paramValue = vStr;
			}
			else if(pnh.getParam(iter->first, vBool))
			{
				paramValue = uBool2Str(vBool);
			}
			else if(pnh.getParam(iter->first, vDouble))
			{
				paramValue = uNumber2Str(vDouble);
			}
			else if(pnh.getParam(iter->first, vInt))
			{
				paramValue = uNumber2Str(vInt);
			}
			if(!paramValue.empty())
			{
				if(iter->second.first)
				{
					// can be migrated
					uInsert(parameters, rtabmap::ParametersPair(iter->second.second, paramValue));
					NODELET_ERROR("obstacles_detection: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
							iter->first.c_str(), iter->second.second.c_str(), paramValue.c_str());
				}
				else
				{
					if(iter->second.second.empty())
					{
						NODELET_ERROR("obstacles_detection: Parameter \"%s\" doesn't exist anymore!",
								iter->first.c_str());
					}
					else
					{
						NODELET_ERROR("obstacles_detection: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
								iter->first.c_str(), iter->second.second.c_str());
					}
				}
			}
		}

		rtabmap::ParametersMap gridParameters2 = rtabmap::Parameters::getDefaultParameters();
		rtabmap::ParametersMap gridParameters = rtabmap::Parameters::getDefaultParameters("Grid");
		for(rtabmap::ParametersMap::iterator iter=gridParameters.begin(); iter!=gridParameters.end(); ++iter)
		{
			std::string vStr;
			bool vBool;
			int vInt;
			double vDouble;
			if(pnh.getParam(iter->first, vStr))
			{
				NODELET_INFO("obstacles_detection: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
				iter->second = vStr;
			}
			else if(pnh.getParam(iter->first, vBool))
			{
				NODELET_INFO("obstacles_detection: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
				iter->second = uBool2Str(vBool);
			}
			else if(pnh.getParam(iter->first, vDouble))
			{
				NODELET_INFO("obstacles_detection: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
				iter->second = uNumber2Str(vDouble);
			}
			else if(pnh.getParam(iter->first, vInt))
			{
				NODELET_INFO("obstacles_detection: Setting parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
				iter->second = uNumber2Str(vInt);
			}
		}
		uInsert(parameters, gridParameters);
		parameterMoved(pnh, "proj_voxel_size", rtabmap::Parameters::kGridCellSize(), parameters);
		parameterMoved(pnh, "ground_normal_angle", rtabmap::Parameters::kGridMaxGroundAngle(), parameters);
		parameterMoved(pnh, "min_cluster_size", rtabmap::Parameters::kGridMinClusterSize(), parameters);
		parameterMoved(pnh, "normal_estimation_radius", rtabmap::Parameters::kGridClusterRadius(), parameters);
		parameterMoved(pnh, "cluster_radius", rtabmap::Parameters::kGridClusterRadius(), parameters);
		parameterMoved(pnh, "max_obstacles_height", rtabmap::Parameters::kGridMaxObstacleHeight(), parameters);
		parameterMoved(pnh, "max_ground_height", rtabmap::Parameters::kGridMaxGroundHeight(), parameters);
		parameterMoved(pnh, "detect_flat_obstacles", rtabmap::Parameters::kGridFlatObstacleDetected(), parameters);
		parameterMoved(pnh, "normal_k", rtabmap::Parameters::kGridNormalK(), parameters);

		UASSERT(uContains(parameters, rtabmap::Parameters::kGridMapFrameProjection()));
		mapFrameProjection_ = uStr2Bool(parameters.at(rtabmap::Parameters::kGridMapFrameProjection()));
		if(mapFrameProjection_ && mapFrameId_.empty())
		{
			NODELET_ERROR("obstacles_detection: Parameter \"%s\" is true but map_frame_id is not set!", rtabmap::Parameters::kGridMapFrameProjection().c_str());
		}

		grid_.parseParameters(parameters);

		cloudSub_ = nh.subscribe("cloud", 1, &ObstaclesDetection::callback, this);

		groundPub_ = nh.advertise<sensor_msgs::PointCloud2>("ground", 1);
		obstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
		projObstaclesPub_ = nh.advertise<sensor_msgs::PointCloud2>("proj_obstacles", 1);
	}



	void callback(const sensor_msgs::PointCloud2ConstPtr & cloudMsg)
	{
		ros::WallTime time = ros::WallTime::now();

		if (groundPub_.getNumSubscribers() == 0 && obstaclesPub_.getNumSubscribers() == 0 && projObstaclesPub_.getNumSubscribers() == 0)
		{
			// no one wants the results
			return;
		}

		rtabmap::Transform localTransform = rtabmap::Transform::getIdentity();
		try
		{
			if(waitForTransform_)
			{
				if(!tfListener_.waitForTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, ros::Duration(1)))
				{
					NODELET_ERROR("Could not get transform from %s to %s after 1 second!", frameId_.c_str(), cloudMsg->header.frame_id.c_str());
					return;
				}
			}
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, cloudMsg->header.frame_id, cloudMsg->header.stamp, tmp);
			localTransform = rtabmap_ros::transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			NODELET_ERROR("%s",ex.what());
			return;
		}

		rtabmap::Transform pose = rtabmap::Transform::getIdentity();
		if(!mapFrameId_.empty())
		{
			try
			{
				if(waitForTransform_)
				{
					if(!tfListener_.waitForTransform(mapFrameId_, frameId_, cloudMsg->header.stamp, ros::Duration(1)))
					{
						NODELET_ERROR("Could not get transform from %s to %s after 1 second!", mapFrameId_.c_str(), frameId_.c_str());
						return;
					}
				}
				tf::StampedTransform tmp;
				tfListener_.lookupTransform(mapFrameId_, frameId_, cloudMsg->header.stamp, tmp);
				pose = rtabmap_ros::transformFromTF(tmp);
			}
			catch(tf::TransformException & ex)
			{
				NODELET_ERROR("%s",ex.what());
				return;
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloudMsg, *inputCloud);

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

			if(cloud->size() && (ground->size() || obstacles->size()))
			{
				if(groundPub_.getNumSubscribers() &&
						ground.get() && ground->size())
				{
					pcl::copyPointCloud(*cloud, *ground, *groundCloud);
				}

				if((obstaclesPub_.getNumSubscribers() || projObstaclesPub_.getNumSubscribers()) &&
						obstacles.get() && obstacles->size())
				{
					// remove flat obstacles from obstacles
					std::set<int> flatObstaclesSet;
					if(projObstaclesPub_.getNumSubscribers())
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

		if(groundPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*groundCloud, rosCloud);
			rosCloud.header = cloudMsg->header;

			//publish the message
			groundPub_.publish(rosCloud);
		}

		if(obstaclesPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*obstaclesCloud, rosCloud);
			rosCloud.header = cloudMsg->header;

			//publish the message
			obstaclesPub_.publish(rosCloud);
		}

		if(projObstaclesPub_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 rosCloud;
			pcl::toROSMsg(*obstaclesCloudWithoutFlatSurfaces, rosCloud);
			rosCloud.header.stamp = cloudMsg->header.stamp;
			rosCloud.header.frame_id = frameId_;

			//publish the message
			projObstaclesPub_.publish(rosCloud);
		}

		NODELET_DEBUG("Obstacles segmentation time = %f s", (ros::WallTime::now() - time).toSec());
	}

private:
	std::string frameId_;
	std::string mapFrameId_;
	bool waitForTransform_;

	rtabmap::OccupancyGrid grid_;
	bool mapFrameProjection_;

	tf::TransformListener tfListener_;

	ros::Publisher groundPub_;
	ros::Publisher obstaclesPub_;
	ros::Publisher projObstaclesPub_;

	ros::Subscriber cloudSub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesDetection, nodelet::Nodelet);
}


