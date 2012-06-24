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
#include "rtabmap_audio/AudioFrameFreqSqrdMagn.h"
#include "rtabmap/Sensorimotor.h"
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MsgConversion.h"

class ImageAudioInput
{
public:
	ImageAudioInput(ros::NodeHandle n) :
		n_(n),
		image_sub(n_, "image", 1),
		audio_sub(n_, "audioFrameFreqSqrdMagn", 1),
		sync(MySyncPolicy(10), image_sub, audio_sub)
	{
		sync.registerCallback(boost::bind(&ImageAudioInput::imageAudioCallback, this, _1, _2));
		sensorimotor_pub_ = n_.advertise<rtabmap::Sensorimotor>("sensorimotor",1);
	}

	void imageAudioCallback(
			const sensor_msgs::ImageConstPtr& image,
			const rtabmap_audio::AudioFrameFreqSqrdMagnConstPtr & audio)
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
		rtabmap::SensorimotorPtr sm(new rtabmap::Sensorimotor());
		sm->header.stamp = ros::Time::now();
		sm->sensors.resize(2);

		//image
		cv_bridge::CvImageConstPtr img = cv_bridge::toCvShare(image);
		sm->sensors[0].type = rtabmap::Sensor::kTypeImage;
		fromCvMatToCvMatMsg(sm->sensors[0].matrix, img->image, false);

		//audio
		cv::Mat dataMat(audio->nChannels, audio->frameLength, CV_32F);
		memcpy(dataMat.data, audio->data.data(), audio->data.size()*sizeof(float));
		sm->sensors[1].type = rtabmap::Sensor::kTypeAudioFreqSqrdMagn;
		fromCvMatToCvMatMsg(sm->sensors[1].matrix, dataMat);

		sensorimotor_pub_.publish(sm);
	}

private:
	ros::NodeHandle n_;

	//inputs
	message_filters::Subscriber<sensor_msgs::Image> image_sub;
	message_filters::Subscriber<rtabmap_audio::AudioFrameFreqSqrdMagn > audio_sub;

	//synchronization stuff
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, rtabmap_audio::AudioFrameFreqSqrdMagn> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync;

	ros::Publisher sensorimotor_pub_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_audio_input");
	ros::NodeHandle n;
	ImageAudioInput iai(n);

	ros::spin();

	return 0;
}

