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
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

namespace rtabmap
{

class DisparityToDepth : public nodelet::Nodelet
{
public:
	DisparityToDepth() {}

	virtual ~DisparityToDepth(){}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		image_transport::ImageTransport it(nh);
		pub_ = it.advertise("depth", 1);
		sub_ = nh.subscribe("disparity", 1, &DisparityToDepth::callback, this);
	}

	void callback(const stereo_msgs::DisparityImageConstPtr& disparityMsg)
	{
		if(disparityMsg->image.encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) !=0)
		{
			ROS_ERROR("Input type must be disparity=32FC1");
			return;
		}

		if(pub_.getNumSubscribers())
		{
			// sensor_msgs::image_encodings::TYPE_32FC1
			cv::Mat disparity(disparityMsg->image.height, disparityMsg->image.width, CV_32FC1, const_cast<uchar*>(disparityMsg->image.data.data()));

			cv::Mat depth = cv::Mat::zeros(disparity.rows, disparity.cols, CV_32F);
			for (int i = 0; i < disparity.rows; i++)
			{
				for (int j = 0; j < disparity.cols; j++)
				{
					float disparity_value = disparity.at<float>(i,j);
					if (disparity_value > disparityMsg->min_disparity && disparity_value < disparityMsg->max_disparity)
					{
						// baseline * focal / disparity
						depth.at<float>(i,j) = disparityMsg->T * disparityMsg->f / disparity_value;
					}
				}
			}

			// convert to ROS sensor_msg::Image
			cv_bridge::CvImage cvDepth(disparityMsg->header, sensor_msgs::image_encodings::TYPE_32FC1, depth);
			sensor_msgs::Image depthMsg;
			cvDepth.toImageMsg(depthMsg);

			//publish the message
			pub_.publish(depthMsg);
		}
}

private:
	image_transport::Publisher pub_;
	ros::Subscriber sub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap::DisparityToDepth, nodelet::Nodelet);
}
