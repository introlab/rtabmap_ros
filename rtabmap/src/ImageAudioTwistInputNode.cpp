/*
 * InputNode.cpp
 *
 *  Created on: 2012-05-27
 *      Author: mathieu
 */


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TwistStamped.h>
#include "rtabmap_audio/AudioFrameFreqSqrdMagn.h"
#include "rtabmap/Sensorimotor.h"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MsgConversion.h"

class ImageAudioTwistInput
{
public:
	ImageAudioTwistInput(ros::NodeHandle n) :
		n_(n),
		image_sub(n_, "image", 1),
		audio_sub(n_, "audioFrameFreqSqrdMagn", 1),
		twist_sub(n_, "cmd_vel", 1),
		sync(MySyncPolicy(10), image_sub, audio_sub, twist_sub)
	{
		sync.registerCallback(boost::bind(&ImageAudioTwistInput::callback, this, _1, _2, _3));
		sensorimotor_pub_ = n_.advertise<rtabmap::Sensorimotor>("sensorimotor",1);
	}

	void callback(
			const sensor_msgs::ImageConstPtr& image,
			const rtabmap_audio::AudioFrameFreqSqrdMagnConstPtr & audio,
			const geometry_msgs::TwistStampedConstPtr & twist)
	{
		if(!image->data.size())
		{
			ROS_ERROR("Image is empty...");
			return;
		}
		if(!audio->data.size())
		{
			ROS_ERROR("Audio is empty...");
			return;
		}

		// Create a sensorimotor msg
		cv::Mat data;
		rtabmap::SensorimotorPtr sm(new rtabmap::Sensorimotor());
		sm->header.stamp = ros::Time::now();
		sm->sensors.resize(2);

		//image
		cv_bridge::CvImageConstPtr img = cv_bridge::toCvShare(image);
		sm->sensors[0].type = rtabmap::Sensor::kTypeImage;
		fromCvMatToCvMatMsg(sm->sensors[0].matrix, img->image, false);

		//audio
		data = cv::Mat(audio->nChannels, audio->frameLength, CV_32F);
		memcpy(data.data, audio->data.data(), audio->data.size()*sizeof(float));
		sm->sensors[1].type = rtabmap::Sensor::kTypeAudioFreqSqrdMagn;
		fromCvMatToCvMatMsg(sm->sensors[1].matrix, data);

		//twist
		sm->actuators.resize(1);
		data = cv::Mat(1, 6, CV_32F);
		data.at<float>(0) = (float)twist->twist.linear.x;
		data.at<float>(1) = (float)twist->twist.linear.y;
		data.at<float>(2) = (float)twist->twist.linear.z;
		data.at<float>(3) = (float)twist->twist.angular.x;
		data.at<float>(4) = (float)twist->twist.angular.y;
		data.at<float>(5) = (float)twist->twist.angular.z;
		sm->actuators[0].type = rtabmap::Actuator::kTypeTwist;
		fromCvMatToCvMatMsg(sm->actuators[0].matrix, data);

		sensorimotor_pub_.publish(sm);
	}

private:
	ros::NodeHandle n_;

	//inputs
	message_filters::Subscriber<sensor_msgs::Image> image_sub;
	message_filters::Subscriber<rtabmap_audio::AudioFrameFreqSqrdMagn> audio_sub;
	message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub;

	//synchronization stuff
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, rtabmap_audio::AudioFrameFreqSqrdMagn, geometry_msgs::TwistStamped> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync;

	ros::Publisher sensorimotor_pub_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_audio_twist_input");
	ros::NodeHandle n;
	ImageAudioTwistInput iati(n);

	ros::spin();

	return 0;
}
