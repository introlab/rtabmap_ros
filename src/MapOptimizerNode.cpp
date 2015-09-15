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
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/thread.hpp>

using namespace rtabmap;

class MapOptimizer
{

public:
	MapOptimizer() :
		mapFrameId_("map"),
		odomFrameId_("odom"),
		iterations_(100),
		ignoreVariance_(false),
		globalOptimization_(true),
		optimizeFromLastNode_(false),
		mapToOdom_(rtabmap::Transform::getIdentity()),
		transformThread_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("iterations", iterations_, iterations_);
		pnh.param("ignore_variance", ignoreVariance_, ignoreVariance_);
		pnh.param("global_optimization", globalOptimization_, globalOptimization_);
		pnh.param("optimize_from_last_node", optimizeFromLastNode_, optimizeFromLastNode_);

		UASSERT(iterations_ > 0);

		double tfDelay = 0.05; // 20 Hz
		bool publishTf = true;
		pnh.param("publish_tf", publishTf, publishTf);
		pnh.param("tf_delay", tfDelay, tfDelay);

		mapDataTopic_ = nh.subscribe("mapData", 1, &MapOptimizer::mapDataReceivedCallback, this);
		mapDataPub_ = nh.advertise<rtabmap_ros::MapData>(nh.resolveName("mapData")+"_optimized", 1);
		mapGraphPub_ = nh.advertise<rtabmap_ros::MapGraph>(nh.resolveName("mapData")+"Graph_optimized", 1);

		if(publishTf)
		{
			ROS_INFO("map_optimizer will publish tf between frames \"%s\" and \"%s\"", mapFrameId_.c_str(), odomFrameId_.c_str());
			ROS_INFO("map_optimizer: map_frame_id = %s", mapFrameId_.c_str());
			ROS_INFO("map_optimizer: odom_frame_id = %s", odomFrameId_.c_str());
			ROS_INFO("map_optimizer: tf_delay = %f", tfDelay);
			transformThread_ = new boost::thread(boost::bind(&MapOptimizer::publishLoop, this, tfDelay));
		}
	}

	~MapOptimizer()
	{
		if(transformThread_)
		{
			transformThread_->join();
			delete transformThread_;
		}
	}

	void publishLoop(double tfDelay)
	{
		if(tfDelay == 0)
			return;
		ros::Rate r(1.0 / tfDelay);
		while(ros::ok())
		{
			mapToOdomMutex_.lock();
			ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfDelay);
			geometry_msgs::TransformStamped msg;
			msg.child_frame_id = odomFrameId_;
			msg.header.frame_id = mapFrameId_;
			msg.header.stamp = tfExpiration;
			rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);
			tfBroadcaster_.sendTransform(msg);
			mapToOdomMutex_.unlock();
			r.sleep();
		}
	}

	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		// save new poses and constraints
		// Assuming that nodes/constraints are all linked together
		UASSERT(msg->graph.posesId.size() == msg->graph.poses.size());

		bool dataChanged = false;

		std::multimap<int, Link> newConstraints;
		for(unsigned int i=0; i<msg->graph.links.size(); ++i)
		{
			Link link = rtabmap_ros::linkFromROS(msg->graph.links[i]);
			newConstraints.insert(std::make_pair(link.from(), link));

			bool edgeAlreadyAdded = false;
			for(std::multimap<int, Link>::iterator iter = cachedConstraints_.lower_bound(link.from());
					iter != cachedConstraints_.end() && iter->first == link.from();
					++iter)
			{
				if(iter->second.to() == link.to())
				{
					edgeAlreadyAdded = true;
					if(iter->second.transform() != link.transform())
					{
						dataChanged = true;
					}
				}
			}
			if(!edgeAlreadyAdded)
			{
				cachedConstraints_.insert(std::make_pair(link.from(), link));
			}
		}

		std::map<int, Transform> newPoses;
		// add new odometry poses
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			int id = msg->nodes[i].id;
			Transform pose = rtabmap_ros::transformFromPoseMsg(msg->nodes[i].pose);
			newPoses.insert(std::make_pair(id, pose));

			std::pair<std::map<int, Transform>::iterator, bool> p = cachedPoses_.insert(std::make_pair(id, pose));
			if(!p.second && pose != cachedPoses_.at(id))
			{
				dataChanged = true;
			}
		}

		if(dataChanged)
		{
			ROS_WARN("Graph data has changed! Reset cache...");
			cachedPoses_ = newPoses;
			cachedConstraints_ = newConstraints;
		}

		//match poses in the graph
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		if(globalOptimization_)
		{
			poses = cachedPoses_;
			constraints = cachedConstraints_;
		}
		else
		{
			constraints = newConstraints;
			for(unsigned int i=0; i<msg->graph.posesId.size(); ++i)
			{
				std::map<int, Transform>::iterator iter = cachedPoses_.find(msg->graph.posesId[i]);
				if(iter != cachedPoses_.end())
				{
					poses.insert(*iter);
				}
				else
				{
					ROS_ERROR("Odometry pose of node %d not found in cache!", msg->graph.posesId[i]);
					return;
				}
			}
		}
		// Optimize only if there is a subscriber
		if(mapDataPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers())
		{
			UTimer timer;
			std::map<int, Transform> optimizedPoses;
			Transform mapCorrection = Transform::getIdentity();
			std::multimap<int, rtabmap::Link> linksOut;
			if(poses.size() > 1 && constraints.size() > 0)
			{
				graph::TOROOptimizer optimizer(iterations_, false, ignoreVariance_);
				int fromId = optimizeFromLastNode_?poses.rbegin()->first:poses.begin()->first;
				std::map<int, rtabmap::Transform> posesOut;
				optimizer.getConnectedGraph(
						fromId,
						poses,
						constraints,
						posesOut,
						linksOut);
				optimizedPoses = optimizer.optimize(fromId, posesOut, linksOut);
				mapToOdomMutex_.lock();
				mapCorrection = optimizedPoses.at(posesOut.rbegin()->first) * posesOut.rbegin()->second.inverse();
				mapToOdom_ = mapCorrection;
				mapToOdomMutex_.unlock();
			}
			else if(poses.size() == 1 && constraints.size() == 0)
			{
				optimizedPoses = poses;
			}
			else if(poses.size() || constraints.size())
			{
				ROS_ERROR("map_optimizer: Poses=%d and edges=%d (poses must "
					   "not be null if there are edges, and edges must be null if poses <= 1)",
					  (int)poses.size(), (int)constraints.size());
			}

			rtabmap_ros::MapData outputDataMsg;
			rtabmap_ros::MapGraph outputGraphMsg;
			rtabmap_ros::mapGraphToROS(optimizedPoses,
					linksOut,
					mapCorrection,
					outputGraphMsg);

			if(mapGraphPub_.getNumSubscribers())
			{
				outputGraphMsg.header = msg->header;
				mapGraphPub_.publish(outputGraphMsg);
			}

			if(mapDataPub_.getNumSubscribers())
			{
				outputDataMsg.header = msg->header;
				outputDataMsg.graph = outputGraphMsg;
				outputDataMsg.nodes = msg->nodes;
				mapDataPub_.publish(outputDataMsg);
			}

			ROS_INFO("Time graph optimization = %f s", timer.ticks());
		}
	}

private:
	std::string mapFrameId_;
	std::string odomFrameId_;
	int iterations_;
	bool ignoreVariance_;
	bool globalOptimization_;
	bool optimizeFromLastNode_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Subscriber mapDataTopic_;

	ros::Publisher mapDataPub_;
	ros::Publisher mapGraphPub_;

	std::map<int, Transform> cachedPoses_;
	std::multimap<int, Link> cachedConstraints_;

	tf2_ros::TransformBroadcaster tfBroadcaster_;
	boost::thread* transformThread_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_optimizer");
	MapOptimizer optimizer;
	ros::spin();
	return 0;
}
