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

bool hueSymbol = false;
int min_dbm = -100;
int max_dbm = -50;
bool autoScale = false;

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
    if(dBm <= min_dbm)
        return 0;
    else if(dBm >= max_dbm)
    	return 100;
    else
    {
    	return -(dBm-min_dbm)*100/(min_dbm-max_dbm);
    }
}

void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
{
	int i;
	float f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
		default:		// case 5:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

ros::Publisher wifiSignalCloudPub;
std::map<double, int> wifiLevels;
std::map<double, int> nodeStamps_;

void mapDataCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg)
{
	ROS_INFO("Received map data!");

	rtabmap::Transform mapToOdom;
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	std::map<int, rtabmap::Signature> signatures;
	rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);

	// handle the case where we can receive only latest data, or if all data are published
	for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
	{
		int id = iter->first;
		rtabmap::Signature & node = iter->second;

		nodeStamps_.insert(std::make_pair(node.getStamp(), node.id()));

		if(node.sensorData().envSensors().find(rtabmap::EnvSensor::kWifiSignalStrength) != node.sensorData().envSensors().end())
		{
			rtabmap::EnvSensor sensor = node.sensorData().envSensors().at(rtabmap::EnvSensor::kWifiSignalStrength);
			wifiLevels.insert(std::make_pair(sensor.stamp()>0.0?sensor.stamp():iter->second.getStamp(), sensor.value()));
		}
		else if(!node.sensorData().userDataCompressed().empty())
		{
			cv::Mat data;
			node.sensorData().uncompressDataConst(0 ,0, 0, &data);

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
		}
	}

	// for the logic below, we should keep only stamps for
	// nodes still in the graph (in case nodes are ignored when not moving)
	std::map<double, int> nodeStamps;
	for(std::map<double, int>::iterator iter=nodeStamps_.begin(); iter!=nodeStamps_.end(); ++iter)
	{
		std::map<int, rtabmap::Transform>::const_iterator jter = poses.find(iter->second);
		if(jter != poses.end())
		{
			nodeStamps.insert(*iter);
		}
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
	float min=0,max=0;
	for(std::map<double, int>::iterator iter=wifiLevels.begin(); iter!=wifiLevels.end(); ++iter, ++id)
	{
		if(min == 0.0f || min > iter->second)
		{
			min = iter->second;
		}
		if(max == 0.0f || max < iter->second)
		{
			max = iter->second;
		}
	}
	ROS_INFO("Min/Max dBm = %f %f", min, max);
	if(autoScale && min<0 && min < max)
	{
		min_dbm = min;
		max_dbm = max;
	}
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

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if(hueSymbol)
			{
				// scale between red -> yellow -> green
				int quality = dBm2Quality(iter->second)*120/100;
				float r,g,b;
				HSVtoRGB(&r,&g,&b,quality,1,1);
				pcl::PointXYZRGB anchor;
				anchor.r = r*255;
				anchor.g = g*255;
				anchor.b = b*255;
				cloud->push_back(anchor);
			}
			else
			{

				// Make a line with points
				int quality = dBm2Quality(iter->second)/10;
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
				pcl::PointXYZRGB anchor;
				anchor.r = 255;
				cloud->push_back(anchor);
			}

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

	pnh.param("hue_symbol", hueSymbol, hueSymbol);
	pnh.param("min", min_dbm, min_dbm);
	pnh.param("max", max_dbm, max_dbm);
	pnh.param("auto", autoScale, autoScale);

	wifiSignalCloudPub = nh.advertise<sensor_msgs::PointCloud2>("wifi_signals", 1);
	ros::Subscriber mapDataSub = nh.subscribe("/rtabmap/mapData", 1, mapDataCallback);

	ros::spin();

	return 0;
}
