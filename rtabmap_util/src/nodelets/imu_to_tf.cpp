/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_listener.h>

namespace rtabmap_util
{

class ImuToTF : public nodelet::Nodelet
{
public:
	ImuToTF() :
		fixedFrameId_("odom"),
		waitForTransformDuration_(0.1)
	{}

	virtual ~ImuToTF()
	{
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		pnh.param("fixed_frame_id", fixedFrameId_, fixedFrameId_);
		pnh.param("base_frame_id", baseFrameId_, baseFrameId_);
		pnh.param("wait_for_transform_duration", waitForTransformDuration_, waitForTransformDuration_);
		NODELET_INFO("fixed_frame_id: %s", fixedFrameId_.c_str());
		NODELET_INFO("base_frame_id: %s", baseFrameId_.c_str());

		sub_ = nh.subscribe<sensor_msgs::Imu>("imu/data", 1, &ImuToTF::imuCallback, this);
	}

	void imuCallback(const sensor_msgs::ImuConstPtr & msg)
	{
		tf::Quaternion q;
		tf::quaternionMsgToTF(msg->orientation, q);
		tf::StampedTransform st;
		st.setRotation(q);

		st.frame_id_ = fixedFrameId_;
		st.stamp_ = msg->header.stamp;

		if(!baseFrameId_.empty() &&
			baseFrameId_.compare(msg->header.frame_id) != 0)
		{
			try
			{
				std::string errorMsg;
				if(!tfListener_.waitForTransform(baseFrameId_, msg->header.frame_id, msg->header.stamp, ros::Duration(waitForTransformDuration_), ros::Duration(0.01), &errorMsg))
				{
					NODELET_ERROR("Could not get transform from %s to %s after %f seconds (for stamp=%f)! Error=\"%s\".",
							baseFrameId_.c_str(), msg->header.frame_id.c_str(), 0.1, msg->header.stamp.toSec(), errorMsg.c_str());
					return;
				}

				tf::StampedTransform tmp;
				tfListener_.lookupTransform(msg->header.frame_id, baseFrameId_, msg->header.stamp, tmp);
				tf::Transform t = tmp.inverse()*st*tmp;
				st.setRotation(t.getRotation());
				st.child_frame_id_ = baseFrameId_;
			}
			catch(tf::TransformException & ex)
			{
				NODELET_ERROR("(getting transform %s -> %s) %s", baseFrameId_.c_str(), msg->header.frame_id.c_str(), ex.what());
				return;
			}
		}
		else
		{
			st.child_frame_id_ = msg->header.frame_id;
		}
		st.setOrigin(tf::Vector3(0,0,0));

		pub_.sendTransform(st);
	}

private:
	ros::Subscriber sub_;
	tf::TransformBroadcaster pub_;
	std::string fixedFrameId_;
	std::string baseFrameId_;
	tf::TransformListener tfListener_;
	double waitForTransformDuration_;
};


PLUGINLIB_EXPORT_CLASS(rtabmap_util::ImuToTF, nodelet::Nodelet);
}

