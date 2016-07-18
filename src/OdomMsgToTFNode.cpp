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
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rtabmap_ros/MsgConversion.h>

class OdomMsgToTF
{

public:
	OdomMsgToTF() :
		frameId_(""),
		odomFrameId_("")
	{
		ros::NodeHandle pnh("~");
		pnh.param("frame_id", frameId_, frameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);

		ros::NodeHandle nh;
		odomTopic_ = nh.subscribe("odom", 1, &OdomMsgToTF::odomReceivedCallback, this);
	}

	virtual ~OdomMsgToTF(){}

	void odomReceivedCallback(const nav_msgs::OdometryConstPtr & msg)
	{
		if(frameId_.empty())
		{
			frameId_ = msg->child_frame_id;
		}
		if(odomFrameId_.empty())
		{
			odomFrameId_ = msg->header.frame_id;
		}
		geometry_msgs::TransformStamped t;
		rtabmap::Transform pose = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
		if(pose.isNull())
		{
			ROS_WARN("Odometry received is null! Cannot send tf...");
		}
		else
		{
			t.child_frame_id = frameId_;
			t.header.frame_id = odomFrameId_;
			t.header.stamp = msg->header.stamp;
			rtabmap_ros::transformToGeometryMsg(pose, t.transform);
			tfBroadcaster_.sendTransform(t);
		}
	}

private:
	std::string frameId_;
	std::string odomFrameId_;

	ros::Subscriber odomTopic_;
	tf2_ros::TransformBroadcaster tfBroadcaster_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_msg_to_tf");
	OdomMsgToTF odomToTf;
	ros::spin();
	return 0;
}
