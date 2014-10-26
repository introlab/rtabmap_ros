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
#include "rtabmap/MapData.h"
#include "rtabmap/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
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
		condChecked_(false),
		mapToOdom_(tf::Transform::getIdentity()),
		transformThread_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("iterations", iterations_, iterations_);
		pnh.param("global_optimization", globalOptimization_, globalOptimization_);

		UASSERT(iterations_ > 0);

		double tfDelay = 0.05; // 20 Hz
		pnh.param("tf_delay", tfDelay, tfDelay);

		// Verify that rtabmap is not sending optimized poses!
		std::string toroIterations;
		if(nh.getParam(Parameters::kRGBDToroIterations(), toroIterations))
		{
			if(std::atoi(toroIterations.c_str()) != 0)
			{
				ROS_WARN("map_optimizer: Parameter \"%s\" of rtabmap node is not 0 (value=%s), it should be 0 (optimization desactivated).",
						Parameters::kRGBDToroIterations().c_str(), toroIterations.c_str());
				exit(-1);
			}
		}

		mapDataTopic_ = nh.subscribe("mapData", 1, &MapOptimizer::mapDataReceivedCallback, this);
		mapDataPub_ = nh.advertise<rtabmap::MapData>(nh.resolveName("mapData")+"_optimized", 1);

		ROS_INFO("map_optimizer will publish tf between frames \"%s\" and \"%s\"", mapFrameId_.c_str(), odomFrameId_.c_str());
		ROS_INFO("map_optimizer: map_frame_id = %s", mapFrameId_.c_str());
		ROS_INFO("map_optimizer: odom_frame_id = %s", odomFrameId_.c_str());
		ROS_INFO("map_optimizer: tf_delay = %f", tfDelay);
		transformThread_ = new boost::thread(boost::bind(&MapOptimizer::publishLoop, this, tfDelay));
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

	void mapDataReceivedCallback(const rtabmap::MapDataConstPtr & msg)
	{
		if(globalOptimization_ && !condChecked_)
		{
			// Verify that rtabmap is not sending optimized poses!
			ros::NodeHandle nh;
			std::string toroIterations;
			if(nh.getParam(Parameters::kRGBDToroIterations(), toroIterations))
			{
				if(std::atoi(toroIterations.c_str()) != 0)
				{
					ROS_ERROR("map_optimizer: Parameter \"%s\" of rtabmap node is not 0 (value=%s), it should be 0 (optimization desactivated).",
							Parameters::kRGBDToroIterations().c_str(), toroIterations.c_str());
					exit(-1);
				}
			}
			else
			{
				ROS_ERROR("map_optimizer: Could not get parameter \"%s\" of rtabmap node, it should be 0 (optimization desactivated). Is rtabmap node started? and in the same namespace that map_assembler?", Parameters::kRGBDToroIterations().c_str());
				exit(-1);
			}
			condChecked_ = true;
		}
		else if(!globalOptimization_)
		{
			// optimize only local map
			poses_.clear();
			constraints_.clear();
			mapIds_.clear();
		}

		// save new poses and constraints
		// Assuming that nodes/constraints are all linked together
		UASSERT(msg->poseIDs.size() == msg->poses.size());
		UASSERT(msg->mapIDs.size() == msg->poseIDs.size());
		UASSERT(msg->mapIDs.size() == msg->maps.size());
		std::map<int, Transform> newPoses;
		std::map<int, int> newMapIds;
		for(unsigned int i=0; i<msg->poseIDs.size() && i<msg->poseIDs.size(); ++i)
		{
			newPoses.insert(std::make_pair(msg->poseIDs[i], transformFromPoseMsg(msg->poses[i])));
			newMapIds.insert(std::make_pair(msg->mapIDs[i], msg->maps[i]));
		}
		UASSERT(msg->constraints.size() == msg->constraintFromIDs.size() &&
				msg->constraints.size() == msg->constraintToIDs.size() &&
				msg->constraints.size() == msg->constraintTypes.size());
		std::multimap<int, Link> allNewConstraints;
		std::multimap<int, Link> filteredNewConstraints;
		bool constraintsChanged = false;
		for(unsigned int i=0; i<msg->constraints.size() && i<msg->constraints.size(); ++i)
		{
			Link link(msg->constraintFromIDs[i], msg->constraintToIDs[i], transformFromGeometryMsg(msg->constraints[i]), (Link::Type)msg->constraintTypes[i]);
			allNewConstraints.insert(std::make_pair(link.from(), link));
			bool edgeAlreadyAdded = false;
			for(std::multimap<int, Link>::iterator iter = constraints_.lower_bound(link.from());
					iter != constraints_.end() && iter->first == link.from();
					++iter)
			{
				if(iter->second.to() == link.to())
				{
					edgeAlreadyAdded = true;
					if(iter->second.transform() != link.transform())
					{
						constraintsChanged = true;
					}
				}
			}
			if(!edgeAlreadyAdded)
			{
				filteredNewConstraints.insert(std::make_pair(link.from(), link));
			}
		}

		//If a transform has changed, clear all.
		if(constraintsChanged)
		{
			UWARN("Some received constraints have changed from the cached "
					"constraints. RTAB-Map is restarted? If yes, ignore this "
					"warning. Clearing all cached constraints and restart with "
					"the new ones...");
			poses_ = newPoses;
			mapIds_ = newMapIds;
			constraints_ = allNewConstraints;
		}
		else
		{
			poses_.insert(newPoses.begin(), newPoses.end());
			mapIds_.insert(newMapIds.begin(), newMapIds.end());
			constraints_.insert(filteredNewConstraints.begin(), filteredNewConstraints.end());
		}

		// Optimize only if there is a subscriber
		if(mapDataPub_.getNumSubscribers())
		{
			std::map<int, Transform> optimizedPoses;
			std::map<int, int> mapIds = mapIds_;
			Transform mapCorrection = Transform::getIdentity();
			if(poses_.size() > 1 && constraints_.size() > 0)
			{
				Transform mapCorrectionToro;
				util3d::optimizeTOROGraph(poses_, constraints_, optimizedPoses, mapCorrectionToro, iterations_, true);

				mapToOdomMutex_.lock();
				mapCorrection = optimizedPoses.at(poses_.rbegin()->first) * poses_.rbegin()->second.inverse();
				rtabmap::transformToTF(mapCorrection, mapToOdom_);
				mapToOdomMutex_.unlock();
			}
			else if(poses_.size() == 1 && constraints_.size() == 0)
			{
				optimizedPoses = poses_;
			}
			else if(poses_.size() || constraints_.size())
			{
				ROS_ERROR("map_optimizer: Poses=%zu and edges=%zu (poses must "
					   "not be null if there are edges, and edges must be null if poses <= 1)",
					   poses_.size(), constraints_.size());
				mapIds.clear();
			}

			UASSERT(optimizedPoses.size() == mapIds.size());
			rtabmap::MapData outputMsg = *msg;
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
		}
	}

private:
	std::string mapFrameId_;
	std::string odomFrameId_;
	int iterations_;
	bool globalOptimization_;

	bool condChecked_;

	tf::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Subscriber mapDataTopic_;

	ros::Publisher mapDataPub_;

	std::map<int, Transform> poses_;
	std::multimap<int, Link> constraints_;
	std::map<int, int> mapIds_;

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
