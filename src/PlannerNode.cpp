/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <tf/transform_listener.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/GetMap.h>

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap/core/Graph.h>

#include <queue>

class Planner
{

public:
	Planner() :
		frameId_("base_link"),
		mapFrameId_("map"),
		plan2d_(true),
		goalError_(1.0),
		graphChangeError_(0.2),
		currentGoalIndex_(0)
	{
		ros::NodeHandle pnh("~");
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("plan_2d", plan2d_, plan2d_);
		pnh.param("goal_error", goalError_, goalError_);
		pnh.param("graph_change_error", graphChangeError_, graphChangeError_);


		ros::NodeHandle nh;
		goalTopic_ = nh.subscribe("goal", 1, &Planner::goalCallback, this);
		mapDataTopic_ = nh.subscribe("mapData", 1, &Planner::mapDataCallback, this);

		pubNextGoal_ = nh.advertise<geometry_msgs::PoseStamped>("next_goal", 1);
		pubGoalReached_ = nh.advertise<std_msgs::Empty>("goal_reached", 1);
		pubPath_ = nh.advertise<nav_msgs::Path>("path", 1);
	}

	~Planner()
	{
	}

	void goalCallback(const std_msgs::Int32ConstPtr & msg)
	{
		int id = msg->data;
		ROS_INFO("planner: set goal %d", id);
		rtabmap_ros::GetMap getMapSrv;
		getMapSrv.request.global = true;
		getMapSrv.request.optimized = true;
		getMapSrv.request.graphOnly = true;
		if(!ros::service::call("get_map", getMapSrv))
		{
			ROS_WARN("Can't call \"get_map\" service");
			return;
		}

		std::map<int, rtabmap::Transform> poses;
		std::map<int, int> mapIds;
		std::multimap<int, rtabmap::Link> constraints;
		rtabmap::Transform mapToOdom;
		rtabmap_ros::mapGraphFromROS(getMapSrv.response.data.graph, poses, mapIds, constraints, mapToOdom);
		std::multimap<int, int> links;
		for(std::multimap<int, rtabmap::Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
		{
			links.insert(std::make_pair(iter->first, iter->second.to()));
			links.insert(std::make_pair(iter->second.to(), iter->first)); // <->
		}

		if(!uContains(poses, id))
		{
			ROS_WARN("Goal %d not found in the global graph!", id);
			return;
		}

		// find the nearest node of the current location
		rtabmap::Transform currentPose;
		try
		{
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(mapFrameId_, frameId_, ros::Time(0), tmp);
			currentPose = rtabmap_ros::transformFromTF(tmp);
			if(plan2d_)
			{
				currentPose.z() = 0;
			}
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		int nearestPose = -1;
		float nearestDistance = -1.0f;
		for(std::map<int, rtabmap::Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(plan2d_)
			{
				iter->second.z() = 0;
			}
			float d = currentPose.getDistance(iter->second);
			if(nearestPose == -1 || d < nearestDistance)
			{
				nearestPose = iter->first;
				nearestDistance = d;
			}
		}

		if(nearestPose <= 0)
		{
			ROS_WARN("Nearest pose not found!?!?");
			return;
		}

		ROS_INFO("Computing path from location %d to %d", nearestPose, id);
		path_ = rtabmap::computePath(poses, links, nearestPose, id);
		currentGoalIndex_ = 0;
		currentGoal_.setNull();
		if(path_.size()<2)
		{
			path_.clear();
			ROS_WARN("Cannot compute a path (or goal is already reached)!");
		}
		else
		{
			ROS_INFO("Path generated! Size=%d", (int)path_.size());
			if(pubPath_.getNumSubscribers())
			{
				nav_msgs::Path path;
				path.header.frame_id = mapFrameId_;
				path.header.stamp = ros::Time::now();
				path.poses.resize(path_.size());
				for(unsigned int i=0; i<path_.size(); ++i)
				{
					path.poses[i].header = path.header;
					rtabmap_ros::transformToPoseMsg(poses[path_[i]], path.poses[i].pose);
				}
				pubPath_.publish(path);
			}
		}
	}

	void mapDataCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		if(path_.size())
		{
			// Get current pose
			rtabmap::Transform currentPose;
			try
			{
				tf::StampedTransform tmp;
				tfListener_.lookupTransform(mapFrameId_, frameId_, ros::Time(0), tmp);
				currentPose = rtabmap_ros::transformFromTF(tmp);
				if(plan2d_)
				{
					currentPose.z() = 0;
				}
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				return;
			}

			// Convert graph data
			std::map<int, rtabmap::Transform> poses;
			std::map<int, int> mapIds;
			std::multimap<int, rtabmap::Link> constraints;
			rtabmap::Transform mapToOdom;
			rtabmap_ros::mapGraphFromROS(msg->graph, poses, mapIds, constraints, mapToOdom);

			// Post a new goal if:
			// - no current goal is set,
			// - the current goal is reached,
			// - the current goal has moved (graph was optimized with new constraints)
			if(!currentGoal_.isNull())
			{
				std::map<int, rtabmap::Transform>::iterator iter=poses.find(path_[currentGoalIndex_]);
				if(iter != poses.end())
				{
					rtabmap::Transform currentGoal = iter->second;
					if(plan2d_)
					{
						currentGoal.z() = 0;
					}
					float d = currentGoal.getDistance(currentGoal_);
					if(d > graphChangeError_)
					{
						// reset the goal
						ROS_INFO("Goal reset because the local graph has changed (%f m) newCurent=%s oldCurrent=%s...",
								d, currentGoal.prettyPrint().c_str(), currentGoal_.prettyPrint().c_str());
						currentGoal_.setNull();
					}

					if(!currentGoal_.isNull())
					{
						float d = currentPose.getDistance(currentGoal_);
						if(d < goalError_)
						{
							//Current goal reached! Reset it.
							currentGoal_.setNull();
							if(currentGoalIndex_ == path_.size()-1)
							{
								// last goal reached!
								ROS_INFO("Goal %d reached!", path_[currentGoalIndex_]);
								pubGoalReached_.publish(std_msgs::Empty());
								path_.clear();
								return; // EXIT
							}

						}
					}
				}

			}
			if(currentGoal_.isNull())
			{
				// find the farthest node that can be reached in the local map
				for(unsigned int i=path_.size()-1; i>=currentGoalIndex_; --i)
				{
					std::map<int, rtabmap::Transform>::iterator iter=poses.find(path_[i]);
					if(iter != poses.end())
					{
						currentGoalIndex_ = i;
						currentGoal_ = iter->second;
						if(plan2d_)
						{
							currentGoal_.z() = 0;
						}
						break;
					}
				}
				if(currentGoal_.isNull())
				{
					ROS_ERROR("Cannot update current goal!");
				}
				else
				{
					ROS_INFO("Publishing next goal: Location %d (%d/%d)",
							path_[currentGoalIndex_], currentGoalIndex_+1, (int)path_.size());
					geometry_msgs::PoseStamped goalMsg;
					goalMsg.header = msg->header;
					rtabmap_ros::transformToPoseMsg(currentGoal_, goalMsg.pose);
					pubNextGoal_.publish(goalMsg);
				}
			}
		}
	}

private:
	std::string frameId_;
	std::string mapFrameId_;
	bool plan2d_;
	double goalError_;
	double graphChangeError_;

	tf::TransformListener tfListener_;

	ros::Subscriber goalTopic_;
	ros::Subscriber mapDataTopic_;

	ros::Publisher pubNextGoal_;
	ros::Publisher pubGoalReached_;
	ros::Publisher pubPath_;

	std::vector<int> path_;
	int currentGoalIndex_;
	rtabmap::Transform currentGoal_;
};


int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);
	ros::init(argc, argv, "planner");
	Planner planner;
	ros::spin();
	return 0;
}
