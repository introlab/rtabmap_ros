/*
 * AudioPlayerNode.cpp
 */

#include <fmod.hpp>
#include <fmod_errors.h>
#include <ros/ros.h>
#include <utilite/ULogger.h>
#include <utilite/UMath.h>
#include <utilite/UThreadNode.h>
#include <utilite/UPlot.h>
#include <utilite/USpectrogram.h>
#include "rtabmap_audio/AudioFrame.h"
#include "rtabmap_audio/AudioFrameFreqSqrdMagn.h"

#include <QApplication>
#include <signal.h>

#define DEFAULT_GUI_USED true
#define PLOT_SAMPLING_RATIO 64

// Ring buffer
unsigned int g_readPtr = 0;
unsigned int g_writePtr = 0;
std::vector<char> g_ringBuffer; // interleaved audio
unsigned int g_readingLooped = 0;
unsigned int g_writingLooped = 0;
bool g_warn = true;

// Fmod stuff
FMOD::System           *g_system = 0;
FMOD::Sound            *g_sound = 0;
FMOD::Channel          *g_channel = 0;
bool                    initialized = false;

// Display stuff
UPlot * g_plot = 0;
UPlotCurve * g_curveReceiving = 0;
UPlotCurve * g_curvePlaying = 0;
USpectrogram * g_spectrogram = 0;

void ERRCHECK(FMOD_RESULT result)
{
    if (result != FMOD_OK)
    {
    	ROS_ERROR("FMOD error! (%d) %s\n", result, FMOD_ErrorString(result));
        exit(-1);
    }
}

void updateCurve(UPlotCurve * curve, void * data, unsigned int dataSize, int channels, int sampleSize)
{
	if(!curve || !data || dataSize == 0)
	{
		return;
	}
	//ROS_INFO("channels=%d, bitsPerSample=%d", channels, sampleSize);
	int downSamplingFactor = PLOT_SAMPLING_RATIO*channels;
	QVector<int> v(dataSize/downSamplingFactor/sampleSize);
	//ROS_INFO("dataSize=%d downSamplingFactor = %d, v=%d", dataSize, downSamplingFactor, v.size());
	if(sampleSize == 1)
	{
		char * p = (char*)data;
		char min,max;
		for(int i=0; i<v.size(); ++i)
		{
			uMinMax(p + i*downSamplingFactor, downSamplingFactor, min, max);
			v[i] = abs(min) > abs(max) ? min : max;
		}
	}
	else if(sampleSize == 2)
	{
		short * p = (short*)data;
		short min,max;
		for(int i=0; i<v.size(); ++i)
		{
			uMinMax(p + i*downSamplingFactor, downSamplingFactor, min, max);
			v[i] = abs(min) > abs(max) ? min : max;
		}
	}
	else if(sampleSize == 4)
	{
		int * p = (int*)data;
		int min,max;
		for(int i=0; i<v.size(); ++i)
		{
			uMinMax(p + i*downSamplingFactor, downSamplingFactor, min, max);
			v[i] = abs(min) > abs(max) ? min : max;
		}
	}
	QMetaObject::invokeMethod(curve, "addValues", Q_ARG(QVector<int>, v));
}

FMOD_RESULT F_CALLBACK pcmreadcallback(FMOD_SOUND *sound, void *data, unsigned int datalen)
{
	//ROS_INFO("datalen=%d, g_readPtr=%d", datalen, g_readPtr);
    unsigned int  count;
    char * buffer = (char *)data;

    bool okToCpy = false;

    if(g_readPtr < g_writePtr)
    {
    	okToCpy = true;
    }
    else if(g_readingLooped < g_writingLooped)
    {
    	okToCpy = true;
    }

    if(okToCpy)
    {
    	g_warn = true;
    	unsigned int start = g_readPtr;
		for(count=0; count<datalen; ++count)
		{
			*(buffer++) = g_ringBuffer[g_readPtr++];
			g_readPtr %= g_ringBuffer.size();
		}
		if(g_readPtr < start)
		{
			++g_readingLooped;
		}

		if(g_plot && g_plot->isVisible() && g_curvePlaying)
		{
			int channels, bitsPerSample;
			FMOD_Sound_GetFormat(sound, 0, 0, &channels, &bitsPerSample);
			updateCurve(g_curvePlaying, &g_ringBuffer[start], datalen, channels, bitsPerSample/8);
		}
		//ROS_INFO("datalen=%d, writePtr=%d, readPtr=%d", datalen, g_writePtr, g_readPtr);
    }
    else if(g_warn)
    {
    	g_warn = false;
    	ROS_WARN("Empty buffer : stream down? (this warning is shown only one time)");

    	//fill data with zeros
    	for(unsigned int i=0; i<datalen; ++i)
		{
			*(buffer++) = 0;
		}
    }

    return FMOD_OK;
}


FMOD_RESULT F_CALLBACK pcmsetposcallback(FMOD_SOUND *sound, int subsound, unsigned int position, FMOD_TIMEUNIT postype)
{
    /*
        This is useful if the user calls Sound::setPosition and you want to seek your data accordingly.
    */
    return FMOD_OK;
}

/**
 * decoderBufferSize = in bytes
 */
bool initAudioPlayer(unsigned int decoderBufferSize, int fs, int channels, int bytesPerSample)
{
	ROS_INFO("Init player with fs=%d, channels=%d, bytesPerSample=%d", fs, channels, bytesPerSample);
	UASSERT(bytesPerSample == 1 || bytesPerSample == 2 || bytesPerSample == 4);
	FMOD_RESULT result;
	FMOD_CREATESOUNDEXINFO createsoundexinfo;
	unsigned int version;
	FMOD_MODE mode = FMOD_2D | FMOD_OPENUSER | FMOD_LOOP_NORMAL | FMOD_HARDWARE | FMOD_CREATESTREAM;
	/*
			Create a System object and initialize.
	*/
	result = FMOD::System_Create(&g_system);
	ERRCHECK(result);

	result = g_system->getVersion(&version);
	ERRCHECK(result);

	if (version < FMOD_VERSION)
	{
		printf("Error!  You are using an old version of FMOD %08x.  This program requires %08x\n", version, FMOD_VERSION);
		return false;
	}

	result = g_system->init(32, FMOD_INIT_NORMAL, 0);
	ERRCHECK(result);

	 memset(&createsoundexinfo, 0, sizeof(FMOD_CREATESOUNDEXINFO));
	createsoundexinfo.cbsize            = sizeof(FMOD_CREATESOUNDEXINFO);              /* required. */
	createsoundexinfo.length            = decoderBufferSize;     		   /* Length of PCM data in bytes of whole song (for Sound::getLength) */
	createsoundexinfo.numchannels       = channels;                                    /* Number of channels in the sound. */
	createsoundexinfo.defaultfrequency  = fs;                                       /* Default playback rate of sound. */
	createsoundexinfo.decodebuffersize  = (decoderBufferSize / bytesPerSample) / channels;                  /* Chunk size of stream update in samples.  This will be the amount of data passed to the user callback. */
	if(bytesPerSample == 1)
	{
		createsoundexinfo.format            = FMOD_SOUND_FORMAT_PCM8;               /* Data format of sound. */
	}
	else if(bytesPerSample == 2)
	{
		createsoundexinfo.format            = FMOD_SOUND_FORMAT_PCM16;               /* Data format of sound. */
	}
	else if(bytesPerSample == 4)
	{
		createsoundexinfo.format            = FMOD_SOUND_FORMAT_PCM32;                 /* Data format of sound. */
	}
	else
	{
		ROS_ERROR("Sample size format (%d) must be 1, 2 or 4", bytesPerSample);
		return false;
	}
	createsoundexinfo.pcmreadcallback   = pcmreadcallback;                             /* User callback for reading. */
	createsoundexinfo.pcmsetposcallback = pcmsetposcallback;                           /* User callback for seeking. */

	result = g_system->createSound(0, mode, &createsoundexinfo, &g_sound);
	ERRCHECK(result);

	/*
		Play the sound.
	*/

	result = g_system->playSound(FMOD_CHANNEL_FREE, g_sound, 0, &g_channel);
	ERRCHECK(result);

	return true;
}

void frameReceivedCallback(const rtabmap_audio::AudioFramePtr & msg)
{
	if(msg->data.size())
	{
		if(!g_ringBuffer.size())
		{
			g_ringBuffer = std::vector<char>(msg->data.size() * 5 * msg->nChannels * msg->sampleSize, 0);
		}

		//ROS_INFO("Received audio frame size=(%d), writePtr=%d", msg->data.size(), g_writePtr);
		unsigned int start = g_writePtr;
		if(g_writePtr + msg->data.size() <= g_ringBuffer.size())
		{
			memcpy(g_ringBuffer.data()+g_writePtr, msg->data.data(), msg->data.size());
			g_writePtr += msg->data.size();
			g_writePtr %= g_ringBuffer.size();
		}
		else
		{
			unsigned int size2 = (g_writePtr + msg->data.size()) - g_ringBuffer.size();
			unsigned int size1 = msg->data.size() - size2;
			memcpy(g_ringBuffer.data()+g_writePtr, msg->data.data(), size1);
			memcpy(g_ringBuffer.data(), msg->data.data() + size1, size2);
			g_writePtr = size2;
		}
		if(g_writePtr < start)
		{
			++g_writingLooped;
		}

		if(!initialized)
		{
			//wait for second frame
			if(g_writePtr > msg->data.size() * 2)
			{
				initialized = initAudioPlayer(msg->data.size(), msg->fs, msg->nChannels, msg->sampleSize);
				if(!initialized)
				{
					ROS_ERROR("Cannot initialize the audio player...");
					exit(-1);
				}
			}
			else if(g_plot)
			{
				if(g_plot->isVisible() && ((msg->data.size() / msg->nChannels) / msg->sampleSize) % PLOT_SAMPLING_RATIO != 0)
				{
					g_plot->setVisible(false);
					ROS_WARN("Frame length (%d) must be a multiple of %d to show audio plot...", ((msg->data.size() / msg->nChannels) / msg->sampleSize), PLOT_SAMPLING_RATIO);
				}
				else if(g_curveReceiving && g_curvePlaying)
				{
					QMetaObject::invokeMethod(g_curveReceiving, "setXIncrement", Q_ARG(float, float(PLOT_SAMPLING_RATIO)/float(msg->fs)));
					QMetaObject::invokeMethod(g_curvePlaying, "setXIncrement", Q_ARG(float, float(PLOT_SAMPLING_RATIO)/float(msg->fs)));
				}
			}
		}
		if(g_plot && g_plot->isVisible() && g_curveReceiving)
		{
			updateCurve(g_curveReceiving, msg->data.data(), msg->data.size(), msg->nChannels, msg->sampleSize);
		}
	}
}

void frameFreqSqrdMagnReceivedCallback(const rtabmap_audio::AudioFrameFreqSqrdMagnPtr & msg)
{
	if(msg->data.size() && msg->nChannels && g_spectrogram && g_spectrogram->isVisible())
	{
		QMetaObject::invokeMethod(g_spectrogram, "setSamplingRate", Q_ARG(int, msg->fs));
		std::vector<float> data(msg->data.size()/msg->nChannels);
		memcpy(data.data(), msg->data.data(), data.size()*sizeof(float)); // Just copy the first channel TODO support more channels...
		QMetaObject::invokeMethod(g_spectrogram, "push", Q_ARG(std::vector<float>, data));
	}
}

void my_handler(int s){
	QApplication::closeAllWindows();
}

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kDebug);
	ros::init(argc, argv, "audioPlayer");

	ros::NodeHandle nh("~");

	//Parameter
	bool guiUsed = DEFAULT_GUI_USED;
	nh.param("gui_used", guiUsed, guiUsed);
	ROS_INFO("gui_used=%s", guiUsed?"true":"false");

	nh = ros::NodeHandle("");
	ros::Subscriber audioSubs = nh.subscribe("audioFrame", 1, frameReceivedCallback);
	ros::Subscriber audioFreqSqrdMagnSubs;

	if(guiUsed)
	{
		QApplication app(argc, argv);

		qRegisterMetaType<QVector<int> >("QVector<int>");
		qRegisterMetaType<std::vector<float> >("std::vector<float>");

		g_plot = new UPlot();
		g_plot->keepAllData(false);
		g_plot->setMaxVisibleItems(1024);
		g_plot->setXLabel("Time (s)");
		g_curveReceiving = g_plot->addCurve("Receiving"); // debugging purpose, to see what is received
		g_curvePlaying = g_plot->addCurve("Playing");
		g_plot->setMinimumSize(600, 400);

		g_spectrogram = new USpectrogram();
		g_spectrogram->setMinimumSize(640, 480);

		g_spectrogram->show();
		g_plot->show();

		audioFreqSqrdMagnSubs = nh.subscribe("audioFrameFreqSqrdMagn", 1, frameFreqSqrdMagnReceivedCallback);

		// Catch ctrl-c to close the gui
		// (Place this after QApplication's constructor)
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = my_handler;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);

		ros::AsyncSpinner spinner(4); // Use 4 threads
		spinner.start();
		app.exec();
		spinner.stop();
	}
	else
	{
		ros::spin();
	}

	uSleep(100); // make sure all subscribers have terminated
	/*
		Shut down
	*/
	FMOD_RESULT result;
	if(g_sound)
	{
		result = g_sound->release();
		ERRCHECK(result);
	}
	if(g_system)
	{
		result = g_system->close();
		ERRCHECK(result);
		result = g_system->release();
		ERRCHECK(result);
	}

	if(g_plot)
	{
		delete g_plot;
	}
	if(g_spectrogram)
	{
		delete g_spectrogram;
	}

	return 0;
}
