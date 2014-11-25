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
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

using namespace rtabmap;

class MapOptimizer
{

public:
	MapOptimizer() :
		mapFrameId_("map"),
		odomFrameId_("odom"),
		iterations_(100),
		globalOptimization_(true),
		optimizeFromLastNode_(false),
		mapToOdom_(tf::Transform::getIdentity()),
		transformThread_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("iterations", iterations_, iterations_);
		pnh.param("global_optimization", globalOptimization_, globalOptimization_);
		pnh.param("optimize_from_last_node", optimizeFromLastNode_, optimizeFromLastNode_);

		UASSERT(iterations_ > 0);

		double tfDelay = 0.05; // 20 Hz
		bool publishTf = true;
		pnh.param("publish_tf", publishTf, publishTf);
		pnh.param("tf_delay", tfDelay, tfDelay);

		mapDataTopic_ = nh.subscribe("mapData", 1, &MapOptimizer::mapDataReceivedCallback, this);
		mapDataPub_ = nh.advertise<rtabmap_ros::MapData>(nh.resolveName("mapData")+"_optimized", 1);

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
			tfBroadcaster_.sendTransform( tf::StampedTransform (mapToOdom_, tfExpiration, mapFrameId_, odomFrameId_));
			mapToOdomMutex_.unlock();
			r.sleep();
		}
	}

	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		// save new poses and constraints
		// Assuming that nodes/constraints are all linked together
		UASSERT(msg->poseIDs.size() == msg->poses.size());
		UASSERT(msg->mapIDs.size() == msg->poseIDs.size());
		UASSERT(msg->mapIDs.size() == msg->maps.size());
		UASSERT(msg->constraints.size() == msg->constraintFromIDs.size() &&
				msg->constraints.size() == msg->constraintToIDs.size() &&
				msg->constraints.size() == msg->constraintTypes.size());

		bool dataChanged = false;

		std::multimap<int, Link> newConstraints;
		for(unsigned int i=0; i<msg->constraints.size() && i<msg->constraints.size(); ++i)
		{
			Link link(msg->constraintFromIDs[i], msg->constraintToIDs[i], transformFromGeometryMsg(msg->constraints[i]), (Link::Type)msg->constraintTypes[i]);
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
		std::map<int, int> newMapIds;
		// add new odometry poses
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			int id = msg->nodes[i].id;
			Transform pose = transformFromPoseMsg(msg->nodes[i].pose);
			newPoses.insert(std::make_pair(id, pose));
			newMapIds.insert(std::make_pair(id, msg->nodes[i].mapId));

			std::pair<std::map<int, Transform>::iterator, bool> p = cachedPoses_.insert(std::make_pair(id, pose));
			if(!p.second && pose != cachedPoses_.at(id))
			{
				dataChanged = true;
			}
			else if(p.second)
			{
				cachedMapIds_.insert(std::make_pair(id, msg->nodes[i].mapId));
			}
		}

		if(dataChanged)
		{
			ROS_WARN("Graph data has changed! Reset cache...");
			cachedPoses_ = newPoses;
			cachedMapIds_ = newMapIds;
			cachedConstraints_ = newConstraints;
		}

		//match poses in the graph
		std::map<int, Transform> poses;
		std::map<int, int> mapIds;
		std::multimap<int, Link> constraints;
		if(globalOptimization_)
		{
			poses = cachedPoses_;
			mapIds = cachedMapIds_;
			constraints = cachedConstraints_;
		}
		else
		{
			constraints = newConstraints;
			for(unsigned int i=0; i<msg->poseIDs.size(); ++i)
			{
				std::map<int, Transform>::iterator iter = cachedPoses_.find(msg->poseIDs[i]);
				if(iter != cachedPoses_.end())
				{
					poses.insert(*iter);
					mapIds.insert(*cachedMapIds_.find(iter->first));
				}
				else
				{
					ROS_ERROR("Odometry pose of node %d not found in cache!", msg->poseIDs[i]);
					return;
				}
			}
		}

		// Optimize only if there is a subscriber
		if(mapDataPub_.getNumSubscribers())
		{
			UTimer timer;
			std::map<int, Transform> optimizedPoses;
			Transform mapCorrection = Transform::getIdentity();
			if(poses.size() > 1 && constraints.size() > 0)
			{
				if(optimizeFromLastNode_)
				{
					std::map<int, int> depthGraph = util3d::generateDepthGraph(constraints, poses.rbegin()->first);
					util3d::optimizeTOROGraph(depthGraph, poses, constraints, optimizedPoses, iterations_);
				}
				else
				{
					util3d::optimizeTOROGraph(poses, constraints, optimizedPoses, iterations_);
				}

				mapToOdomMutex_.lock();
				mapCorrection = optimizedPoses.at(poses.rbegin()->first) * poses.rbegin()->second.inverse();
				rtabmap::transformToTF(mapCorrection, mapToOdom_);
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
				mapIds.clear();
			}

			UASSERT(optimizedPoses.size() == mapIds.size());
			rtabmap_ros::MapData outputMsg = *msg;
			outputMsg.poseIDs.resize(optimizedPoses.size());
			outputMsg.poses.resize(optimizedPoses.size());
			outputMsg.mapIDs.resize(mapIds.size());
			outputMsg.maps.resize(mapIds.size());
			rtabmap::transformToGeometryMsg(mapCorrection, outputMsg.mapToOdom);
			int i=0;
			std::map<int, int>::iterator jter = mapIds.begin();
			for(std::map<int, Transform>::iterator iter = optimizedPoses.begin(); iter!=optimizedPoses.end(); ++iter, ++jter)
			{
				outputMsg.poseIDs[i] = iter->first;
				transformToPoseMsg(iter->second, outputMsg.poses[i]);
				outputMsg.mapIDs[i] = jter->first;
				outputMsg.maps[i] = jter->second;
				++i;
			}
			mapDataPub_.publish(outputMsg);

			ROS_INFO("Time graph optimization = %f s", timer.ticks());
		}
	}

private:
	std::string mapFrameId_;
	std::string odomFrameId_;
	int iterations_;
	bool globalOptimization_;
	bool optimizeFromLastNode_;

	tf::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Subscriber mapDataTopic_;

	ros::Publisher mapDataPub_;

	std::map<int, Transform> cachedPoses_;
	std::map<int, int> cachedMapIds_;
	std::multimap<int, Link> cachedConstraints_;

	tf::TransformBroadcaster tfBroadcaster_;
	boost::thread* transformThread_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_optimizer");
	MapOptimizer optimizer;
	ros::spin();
	return 0;
}
