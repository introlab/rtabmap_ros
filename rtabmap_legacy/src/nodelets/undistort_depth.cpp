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
#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/publisher.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "rtabmap/core/clams/discrete_depth_distortion_model.h"
#include "rtabmap/utilite/UConversion.h"

namespace rtabmap_legacy
{

class UndistortDepth : public nodelet::Nodelet
{
public:
	UndistortDepth()
	{}

	virtual ~UndistortDepth()
	{
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		std::string modelPath;
		pnh.param("model", modelPath, modelPath);

		if(modelPath.empty())
		{
			NODELET_ERROR("undistort_depth: \"model\" parameter should be set!");
		}

		model_.load(modelPath);
		if(!model_.isValid())
		{
			NODELET_ERROR("Loaded distortion model from \"%s\" is not valid!", modelPath.c_str());
		}
		else
		{
			image_transport::ImageTransport it(nh);
			sub_ = it.subscribe("depth", 1, &UndistortDepth::callback, this);
			pub_ = it.advertise(uFormat("%s_undistorted", nh.resolveName("depth").c_str()), 1);
		}
	}

	void callback(const sensor_msgs::ImageConstPtr& depth)
	{
		if(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)!=0 &&
		   depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)!=0 &&
		   depth->encoding.compare(sensor_msgs::image_encodings::MONO16)!=0)
		{
			NODELET_ERROR("Input type depth=32FC1,16UC1,MONO16");
			return;
		}

		if(pub_.getNumSubscribers())
		{
			if(depth->width == model_.getWidth() && depth->width == model_.getWidth())
			{
				cv_bridge::CvImagePtr imageDepthPtr = cv_bridge::toCvCopy(depth);
				model_.undistort(imageDepthPtr->image);
				pub_.publish(imageDepthPtr->toImageMsg());
			}
			else
			{
				NODELET_ERROR("Input depth image size (%dx%d) and distortion model "
						"size (%dx%d) don't match! Cannot undistort image.",
						depth->width, depth->height,
						model_.getWidth(), model_.getHeight());
			}
		}
	}

private:
	clams::DiscreteDepthDistortionModel model_;
	image_transport::Publisher pub_;
	image_transport::Subscriber sub_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_legacy::UndistortDepth, nodelet::Nodelet);
}

