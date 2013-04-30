/*
 * CoreWrapper.h
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#ifndef COREWRAPPER_H_
#define COREWRAPPER_H_


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <rtabmap/core/Statistics.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <rtabmap/core/Parameters.h>

namespace rtabmap
{
	class Rtabmap;
}

class CoreWrapper
{
public:
	CoreWrapper(bool deleteDbOnStart = false);
	virtual ~CoreWrapper();

private:
	void imageReceivedCallback(const sensor_msgs::ImageConstPtr & msg);
	void twistCallback(const geometry_msgs::TwistConstPtr & msg);
	void parametersUpdatedCallback(const std_msgs::EmptyConstPtr & msg);

	bool resetMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool dumpMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool dumpPredictionCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
	bool deleteMemoryCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

	rtabmap::ParametersMap loadNodeParameters(const std::string & configFile,
											  const std::string & workingDirectory);
	void saveNodeParameters(const std::string & configFile);

	void publishStats(const rtabmap::Statistics & stats);

private:
	rtabmap::Rtabmap * rtabmap_;
	image_transport::Subscriber imageTopic_;
	ros::Subscriber audioFrameFreqSqrdMagnTopic_;
	ros::Subscriber twistTopic_;
	ros::Subscriber parametersUpdatedTopic_;
	ros::Publisher infoPub_;
	ros::Publisher infoPubEx_;
	ros::Publisher parametersLoadedPub_;
	std::string configPath_;

	ros::ServiceServer resetMemorySrv_;
	ros::ServiceServer dumpMemorySrv_;
	ros::ServiceServer deleteMemorySrv_;
	ros::ServiceServer dumpPredictionSrv_;
};

#endif /* COREWRAPPER_H_ */
