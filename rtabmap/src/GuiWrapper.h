/*
 * GuiWrapper.h
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
 */

#ifndef GUIWRAPPER_H_
#define GUIWRAPPER_H_

#include <ros/ros.h>
#include "rtabmap/RtabmapInfo.h"
#include "rtabmap/RtabmapInfoEx.h"
#include "utilite/UEventsHandler.h"
#include <geometry_msgs/TwistStamped.h>

namespace rtabmap
{
	class MainWindow;
}

class QApplication;

class GuiWrapper : public UEventsHandler
{
public:
	GuiWrapper(int & argc, char** argv);
	virtual ~GuiWrapper();

	int exec();

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	void infoReceivedCallback(const rtabmap::RtabmapInfoConstPtr & infoMsg);
	void velocityReceivedCallback(const geometry_msgs::TwistStampedConstPtr & msg);

private:
	ros::Subscriber infoTopic_;
	ros::Subscriber velocity_sub_;
	QApplication * app_;
	rtabmap::MainWindow * mainWindow_;

	ros::ServiceClient resetMemoryClient_;
	ros::ServiceClient dumpMemoryClient_;
	ros::ServiceClient changeCameraImgRateClient_;
	ros::ServiceClient deleteMemoryClient_;
	ros::ServiceClient dumpPredictionClient_;

	ros::Publisher parametersUpdatedPub_;
};

#endif /* GUIWRAPPER_H_ */
