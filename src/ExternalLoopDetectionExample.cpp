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
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/Info.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/AddLink.h>

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/utilite/UStl.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

/*
 * Test:
 * $ roslaunch rtabmap_ros demo_robot_mapping.launch
 * Disable internal loop closure detection, in rtabmapviz->Preferences:
 *    ->Vocabulary, set Max words to -1 (loop closure detection disabled)
 *    ->Proximity Detection, uncheck proximity detection by space
 * $ rosrun rtabmap_ros external_loop_detection_example
 * $ rosbag play --clock demo_mapping.bag
 */

typedef message_filters::sync_policies::ExactTime<rtabmap_ros::MapData, rtabmap_ros::Info> MyInfoMapSyncPolicy;

ros::ServiceClient addLinkSrv;

// This is used to keep in cache the old data of the map
std::map<int, rtabmap::SensorData> localData;

// In this example, we use rtabmap for our loop closure detector
rtabmap::Rtabmap loopClosureDetector;

rtabmap::RegistrationVis reg;

void mapDataCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg, const rtabmap_ros::InfoConstPtr & infoMsg)
{
	ROS_INFO("Received map data!");

	rtabmap::Statistics stats;
	rtabmap_ros::infoFromROS(*infoMsg, stats);

	bool smallMovement = (bool)uValue(stats.data(), rtabmap::Statistics::kMemorySmall_movement(), 0.0f);
	bool fastMovement = (bool)uValue(stats.data(), rtabmap::Statistics::kMemoryFast_movement(), 0.0f);

	if(smallMovement || fastMovement)
	{
		// The signature has been ignored from rtabmap, don't process it
		return;
	}

	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	std::map<int, rtabmap::Signature> signatures;
	rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);

	if(!signatures.empty() &&
		signatures.rbegin()->second.sensorData().isValid() &&
		localData.find(signatures.rbegin()->first) == localData.end())
	{
		int id = signatures.rbegin()->first;
		const rtabmap::SensorData & s =  signatures.rbegin()->second.sensorData();
		cv::Mat rgb;
		//rtabmap::LaserScan scan;
		s.uncompressDataConst(&rgb, 0/*, &scan*/);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::laserScanToPointCloud(scan, scan.localTransform());

		if(loopClosureDetector.process(rgb, id))
		{
			localData.insert(std::make_pair(id, s));

			if(loopClosureDetector.getLoopClosureId()>0)
			{
				int fromId = id;
				int toId = loopClosureDetector.getLoopClosureId();
				ROS_INFO("Detected loop closure between %d and %d", fromId, toId);
				//Compute transformation
				rtabmap::RegistrationInfo regInfo;
				rtabmap::SensorData tmpFrom = localData.at(fromId);
				rtabmap::SensorData tmpTo = localData.at(toId);
				tmpFrom.uncompressData();
				tmpTo.uncompressData();
				rtabmap::Transform t = reg.computeTransformation(tmpFrom, tmpTo, rtabmap::Transform(), &regInfo);

				if(!t.isNull())
				{
					rtabmap::Link link(fromId, toId, rtabmap::Link::kUserClosure, t, regInfo.covariance.inv());
					rtabmap_ros::AddLinkRequest req;
					rtabmap_ros::linkToROS(link, req.link);
					rtabmap_ros::AddLinkResponse res;
					if(!addLinkSrv.call(req, res))
					{
						ROS_ERROR("Failed to call %s service", addLinkSrv.getService().c_str());
					}
				}
				else
				{
					ROS_WARN("Could not compute transformation between %d and %d: %s", fromId, toId, regInfo.rejectedMsg.c_str());
				}
			}
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "external_loop_detection_example");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	rtabmap::ParametersMap params;
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemGenerateIds(), "false")); // use provided ids
	params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDEnabled(), "false")); // BOW-only mode
	loopClosureDetector.init(params, "");

	// service to add link
	addLinkSrv = nh.serviceClient<rtabmap_ros::AddLink>("/rtabmap/add_link");

	// subscription
	message_filters::Subscriber<rtabmap_ros::Info> infoTopic;
	message_filters::Subscriber<rtabmap_ros::MapData> mapDataTopic;
	infoTopic.subscribe(nh, "/rtabmap/info", 1);
	mapDataTopic.subscribe(nh, "/rtabmap/mapData", 1);
	message_filters::Synchronizer<MyInfoMapSyncPolicy> infoMapSync(
			MyInfoMapSyncPolicy(10),
			mapDataTopic,
			infoTopic);
	infoMapSync.registerCallback(&mapDataCallback);
	ROS_INFO("Subscribed to %s and %s", mapDataTopic.getTopic().c_str(), infoTopic.getTopic().c_str());

	ros::spin();

	return 0;
}
