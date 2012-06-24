/*
 * AudioRecorderNode.cpp
 */

#include <ros/ros.h>

#include <utilite/ULogger.h>
#include <utilite/UFile.h>
#include <utilite/UEventsHandler.h>
#include <utilite/UEventsManager.h>
#include <utilite/UEvent.h>
#include <utilite/UThreadNode.h>

#include <std_msgs/Empty.h>

#include <rtabmap/core/Micro.h>

#include "rtabmap_audio/AudioFrame.h"
#include "rtabmap_audio/AudioFrameFreq.h"
#include "rtabmap_audio/AudioFrameFreqSqrdMagn.h"

#define DEFAULT_DEVICE_ID 0
#define DEFAULT_FILE_NAME ""
#define DEFAULT_FRAME_LENGTH 4800
#define DEFAULT_FS 48000
#define DEFAULT_SAMPLE_SIZE 2
#define DEFAULT_CHANNELS 1

class EndEvent : public UEvent
{
public:
	virtual std::string getClassName() const {return "EndEvent";}
};

class MicroWrapper : public UThreadNode
{
public:
	MicroWrapper(int deviceId, int frameLength, int fs, int sampleSize, int nChannels)
	{
		micro_ = new rtabmap::Micro(rtabmap::MicroEvent::kTypeFrame, deviceId, fs, frameLength, nChannels, sampleSize, 0);
		ros::NodeHandle nh("");
		audioFramePublisher_ = nh.advertise<rtabmap_audio::AudioFrame>("audioFrame", 1);
		audioFrameFreqPublisher_ = nh.advertise<rtabmap_audio::AudioFrameFreq>("audioFrameFreq", 1);
		audioFrameFreqSqrdMagnPublisher_ = nh.advertise<rtabmap_audio::AudioFrameFreqSqrdMagn>("audioFrameFreqSqrdMagn", 1);
	}

	MicroWrapper(const std::string & fileName, int frameLength)
	{
		micro_ = new rtabmap::Micro(rtabmap::MicroEvent::kTypeFrame, fileName, true, frameLength, 0);
		ros::NodeHandle nh("");
		audioFramePublisher_ = nh.advertise<rtabmap_audio::AudioFrame>("audioFrame", 1);
		audioFrameFreqPublisher_ = nh.advertise<rtabmap_audio::AudioFrameFreq>("audioFrameFreq", 1);
		audioFrameFreqSqrdMagnPublisher_ = nh.advertise<rtabmap_audio::AudioFrameFreqSqrdMagn>("audioFrameFreqSqrdMagn", 1);
	}

	virtual ~MicroWrapper()
	{
		this->join(true);
		micro_->join(true);
		delete micro_;
	}

	bool init()
	{
		if(micro_)
		{
			return micro_->init();
		}
		return false;
	}

	unsigned int freq() const
	{
		if(micro_)
		{
			return micro_->fs();
		}
		return 0;
	}

	unsigned int sampleSize() const
	{
		return micro_->bytesPerSample();
	}

	unsigned int nChannels() const
	{
		return micro_->channels();
	}

protected:
	virtual void mainLoopBegin()
	{
		micro_->startRecorder();
	}

	virtual void mainLoop()
	{
		if(!micro_)
		{
			UERROR("micro_ is not initialized");
			this->kill();
		}
		bool computeFFT = false;
		if(audioFrameFreqPublisher_.getNumSubscribers() || audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
		{
			computeFFT = true;
		}

		cv::Mat data;
		cv::Mat freq;
		if(!computeFFT)
		{
			data = micro_->getFrame();
		}
		else
		{
			data = micro_->getFrame(freq, false);
		}
		if(!data.empty())
		{
			ros::Time now = ros::Time::now();
			if(audioFramePublisher_.getNumSubscribers())
			{
				rtabmap_audio::AudioFramePtr msg(new rtabmap_audio::AudioFrame);
				msg->header.frame_id = "micro";
				msg->header.stamp = now;
				msg->data.resize(data.total()*data.elemSize());
				// Interleave the data
				for(unsigned int i=0; i<msg->data.size(); i+=data.elemSize()*data.rows)
				{
					for(int j=0; j<data.rows; ++j)
					{
						memcpy(msg->data.data()+i+j*data.elemSize(), data.data + (i/(data.elemSize()*data.rows))*data.elemSize() + j*data.cols*data.elemSize(), data.elemSize());
					}
				}
				msg->frameLength = data.cols;
				msg->fs = micro_->fs();
				msg->nChannels = data.rows;
				msg->sampleSize = data.elemSize();
				audioFramePublisher_.publish(msg);
			}
			if(audioFrameFreqPublisher_.getNumSubscribers())
			{
				rtabmap_audio::AudioFrameFreqPtr msg(new rtabmap_audio::AudioFrameFreq);
				msg->header.frame_id = "micro";
				msg->header.stamp = now;
				msg->data.resize(freq.total()*freq.elemSize());
				memcpy(msg->data.data(), freq.data, msg->data.size());
				msg->frameLength = freq.cols;
				msg->fs = micro_->fs();
				msg->nChannels = freq.rows;
				audioFrameFreqPublisher_.publish(msg);
			}
			if(audioFrameFreqSqrdMagnPublisher_.getNumSubscribers())
			{
				rtabmap_audio::AudioFrameFreqSqrdMagnPtr msg(new rtabmap_audio::AudioFrameFreqSqrdMagn);
				msg->header.frame_id = "micro";
				msg->header.stamp = now;

				//compute the squared magnitude
				cv::Mat sqrdMagn(freq.rows, freq.cols/2, CV_32F);
				//for each channels
				for(int i=0; i<sqrdMagn.rows; ++i)
				{
					cv::Mat rowFreq = freq.row(i);
					cv::Mat rowSqrdMagn = sqrdMagn.row(i);
					float re;
					float im;
					for(int j=0; j<rowSqrdMagn.cols; ++j)
					{
						re = rowFreq.at<float>(0, j*2);
						im = rowFreq.at<float>(0, j*2+1);
						rowSqrdMagn.at<float>(0, j) = re*re + im*im;
					}
				}

				msg->data.resize(sqrdMagn.cols * sqrdMagn.rows);
				memcpy(msg->data.data(), sqrdMagn.data, sqrdMagn.total() * sqrdMagn.elemSize());
				msg->frameLength = sqrdMagn.cols;
				msg->fs = micro_->fs();
				msg->nChannels = sqrdMagn.rows;

				audioFrameFreqSqrdMagnPublisher_.publish(msg);
			}
		}
		else
		{
			UEventsManager::post(new EndEvent());
			this->kill();
		}
	}

private:
	ros::Publisher audioFramePublisher_;
	ros::Publisher audioFrameFreqPublisher_;
	ros::Publisher audioFrameFreqSqrdMagnPublisher_;
	rtabmap::Micro * micro_;
};

class Handler: public UEventsHandler
{
public:
	Handler() {UEventsManager::addHandler(this);}
	virtual ~Handler() {UEventsManager::removeHandler(this);}
protected:
	virtual void handleEvent(UEvent * event)
	{
		if(event->getClassName().compare("EndEvent") == 0)
		{
			ROS_INFO("End of stream reached... shutting down!");
			ros::shutdown();
		}
	}
};

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	//ULogger::setLevel(ULogger::kDebug);
	ros::init(argc, argv, "audioRecorder");
	ros::NodeHandle nh("~");

	int deviceId = DEFAULT_DEVICE_ID;
	std::string fileName = DEFAULT_FILE_NAME;
	int frameLength = DEFAULT_FRAME_LENGTH;
	int fs = DEFAULT_FS;
	int sampleSize = DEFAULT_SAMPLE_SIZE;
	int nChannels = DEFAULT_CHANNELS;

	nh.param("device_id", deviceId, deviceId);
	nh.param("file_name", fileName, fileName);
	nh.param("frame_length", frameLength, frameLength);
	nh.param("fs", fs, fs);
	nh.param("sample_size", sampleSize, sampleSize);
	nh.param("channels", nChannels, nChannels);

	MicroWrapper * micro;
	if(fileName.size() && UFile::exists(fileName))
	{
		ROS_INFO("Recording from file %s", fileName.c_str());
		micro = new MicroWrapper(fileName, frameLength);
	}
	else
	{
		ROS_INFO("Recording from microphone %d", deviceId);
		micro = new MicroWrapper(deviceId, frameLength, fs, sampleSize, nChannels);
	}

	if(!micro->init())
	{
		ROS_ERROR("Cannot initiate the audio recorder.");
	}
	else
	{
		ROS_INFO("frame_length=%d", frameLength);
		ROS_INFO("fs=%d", micro->freq());
		ROS_INFO("sample_size=%d", micro->sampleSize());
		ROS_INFO("channels=%d", micro->nChannels());

		Handler h;
		// Start the mic
		micro->start();
		ROS_INFO("Audio recorder started...");
		ros::spin();
	}

	micro->join(true);
	delete micro;

	return 0;
}
