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

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/util3d_transforms.h>

#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// A percentage value that represents the signal quality
// of the network. WLAN_SIGNAL_QUALITY is of type ULONG.
// This member contains a value between 0 and 100. A value
// of 0 implies an actual RSSI signal strength of -100 dbm.
// A value of 100 implies an actual RSSI signal strength of -50 dbm.
// You can calculate the RSSI signal strength value for wlanSignalQuality
// values between 1 and 99 using linear interpolation.
inline int dBm2Quality(int dBm)
{
	// dBm to Quality:
    if(dBm <= -100)
        return 0;
    else if(dBm >= -50)
    	return 100;
    else
    	return 2 * (dBm + 100);
}

ros::Publisher wifiSignalCloudPub;
std::map<double, int> wifiLevels;

void mapDataCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg)
{
	ROS_INFO("Received map data!");

	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	std::map<int, rtabmap::Signature> signatures;
	rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);

	std::map<double, int> nodeStamps; // <stamp, id>
	for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
	{
		cv::Mat data;
		iter->second.sensorData().uncompressDataConst(0 ,0, 0, &data);

		if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 2)
		{
			// format [int level, double stamp], see wifi_signal_pub_node.cpp
			int level = data.at<double>(0);
			double stamp = data.at<double>(1);
			wifiLevels.insert(std::make_pair(stamp, level));
		}
		else if(!data.empty())
		{
			ROS_ERROR("Wrong user data format for wifi signal.");
		}

		// Sort stamps by stamps->id
		nodeStamps.insert(std::make_pair(iter->second.getStamp(), iter->first));
	}

	if(wifiLevels.size() == 0)
	{
		ROS_WARN("No wifi signal detected yet in user data of map data");
	}

	//============================
	// Add WIFI symbols
	//============================

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledWifiSignals(new pcl::PointCloud<pcl::PointXYZRGB>);
	int id = 0;
	for(std::map<double, int>::iterator iter=wifiLevels.begin(); iter!=wifiLevels.end(); ++iter, ++id)
	{
		// The Wifi value may be taken between two nodes, interpolate its position.
		double stampWifi = iter->first;
		std::map<double, int>::iterator previousNode = nodeStamps.lower_bound(stampWifi); // lower bound of the stamp
		if(previousNode!=nodeStamps.end() && previousNode->first > stampWifi && previousNode != nodeStamps.begin())
		{
			--previousNode;
		}
		std::map<double, int>::iterator nextNode = nodeStamps.upper_bound(stampWifi); // upper bound of the stamp

		if(previousNode != nodeStamps.end() &&
		   nextNode != nodeStamps.end() &&
		   previousNode->second != nextNode->second &&
		   uContains(poses, previousNode->second) && uContains(poses, nextNode->second))
		{
			rtabmap::Transform poseA = poses.at(previousNode->second);
			rtabmap::Transform poseB = poses.at(nextNode->second);
			double stampA = previousNode->first;
			double stampB = nextNode->first;
			UASSERT(stampWifi>=stampA && stampWifi <=stampB);

			rtabmap::Transform v = poseA.inverse() * poseB;
			double ratio = (stampWifi-stampA)/(stampB-stampA);

			v.x()*=ratio;
			v.y()*=ratio;
			v.z()*=ratio;

			rtabmap::Transform wifiPose = (poseA*v).translation(); // rip off the rotation

			// Make a line with points
			int quality = dBm2Quality(iter->second)/10;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			for(int i=0; i<10; ++i)
			{
				// 2 cm between each points
				// the number of points depends on the dBm (which varies from -30 (near) to -80 (far))
				pcl::PointXYZRGB pt;
				pt.z = float(i+1)*0.02f;
				if(i<quality)
				{
					// green
					pt.g = 255;
					if(i<7)
					{
						// yellow
						pt.r = 255;
					}
				}
				else
				{
					// gray
					pt.r = pt.g = pt.b = 100;
				}
				cloud->push_back(pt);
			}
			pcl::PointXYZRGB anchor(255, 0, 0);
			cloud->push_back(anchor);

			cloud = rtabmap::util3d::transformPointCloud(cloud, wifiPose);

			if(assembledWifiSignals->size() == 0)
			{
				*assembledWifiSignals = *cloud;
			}
			else
			{
				*assembledWifiSignals += *cloud;
			}
		}
	}

	if(assembledWifiSignals->size())
	{
		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(*assembledWifiSignals, cloudMsg);
		cloudMsg.header = mapDataMsg->header;
		wifiSignalCloudPub.publish(cloudMsg);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wifi_signal_sub");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string interface = "wlan0";
	double rateHz = 0.5; // Hz
	std::string frameId = "base_link";

	wifiSignalCloudPub = nh.advertise<sensor_msgs::PointCloud2>("wifi_signals", 1);
	ros::Subscriber mapDataSub = nh.subscribe("/rtabmap/mapData", 1, mapDataCallback);

	ros::spin();

	return 0;
}
