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

#include <sys/socket.h>
#include <linux/wireless.h>
#include <sys/ioctl.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/UserData.h>

// Demo:
// $ roslaunch freenect_launch freenect.launch depth_registration:=true
// $ roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" user_data_async_topic:=/wifi_signal rtabmapviz:=false rviz:=true
// $ rosrun rtabmap_ros wifi_signal_pub interface:="wlan0"
// $ rosrun rtabmap_ros wifi_signal_sub
// In RVIZ add PointCloud2 "wifi_signals"

// A percentage value that represents the signal quality
// of the network. WLAN_SIGNAL_QUALITY is of type ULONG.
// This member contains a value between 0 and 100. A value
// of 0 implies an actual RSSI signal strength of -100 dbm.
// A value of 100 implies an actual RSSI signal strength of -50 dbm.
// You can calculate the RSSI signal strength value for wlanSignalQuality
// values between 1 and 99 using linear interpolation.
inline int quality2dBm(int quality)
{
	// Quality to dBm:
	if(quality <= 0)
		return -100;
	else if(quality >= 100)
		return -50;
	else
		return (quality / 2) - 100;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "wifi_signal_pub");

	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	std::string interface = "wlan0";
	double rateHz = 0.5; // Hz
	std::string frameId = "base_link";

	pnh.param("interface", interface, interface);
	pnh.param("rate", rateHz, rateHz);
	pnh.param("frame_id", frameId, frameId);

	ros::Rate rate(rateHz);

	ros::Publisher wifiPub = nh.advertise<rtabmap_ros::UserData>("wifi_signal", 1);

	while(ros::ok())
	{
		int dBm = 0;

		// Code inspired from http://blog.ajhodges.com/2011/10/using-ioctl-to-gather-wifi-information.html

		//have to use a socket for ioctl
		int sockfd;
		/* Any old socket will do, and a datagram socket is pretty cheap */
		if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
			ROS_ERROR("Could not create simple datagram socket");
			return -1;
		}

		struct iwreq req;
		struct iw_statistics stats;

		strncpy(req.ifr_name, interface.c_str(), IFNAMSIZ);

		//make room for the iw_statistics object
		req.u.data.pointer = (caddr_t) &stats;
		req.u.data.length = sizeof(stats);
		// clear updated flag
		req.u.data.flags = 1;

		//this will gather the signal strength
		if(ioctl(sockfd, SIOCGIWSTATS, &req) == -1)
		{
			//die with error, invalid interface
			ROS_ERROR("Invalid interface (\"%s\"). Tip: Try with sudo!", interface.c_str());
		}
		else if(((iw_statistics *)req.u.data.pointer)->qual.updated & IW_QUAL_DBM)
		{
			//signal is measured in dBm and is valid for us to use
			dBm = ((iw_statistics *)req.u.data.pointer)->qual.level - 256;
		}
		else
		{
			ROS_ERROR("Could not get signal level.");
		}

		close(sockfd);

		if(dBm != 0)
		{
			ros::Time stamp = ros::Time::now();

			// Create user data [level] with the value
			cv::Mat data(1, 2, CV_64FC1);
			data.at<double>(0) = double(dBm);

			// we should set stamp in data to be able to
			// retrieve it from rtabmap map data to get precise
			// position in the graph afterward
			data.at<double>(1) = stamp.toSec();

			rtabmap_ros::UserData dataMsg;
			dataMsg.header.frame_id = frameId;
			dataMsg.header.stamp = stamp;
			rtabmap_ros::userDataToROS(data, dataMsg, false);
			wifiPub.publish<rtabmap_ros::UserData>(dataMsg);
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
