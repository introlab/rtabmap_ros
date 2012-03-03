/*
 * CoreNode.cpp
 *
 *  Created on: 2 févr. 2010
 *      Author: labm2414
 */

#include "GuiWrapper.h"
#include "utilite/ULogger.h"

#include <QApplication>
#include <rtabmap/gui/MainWindow.h>
#include <signal.h>

void my_handler(int s){
	QApplication::closeAllWindows();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rtabmap_gui");

	GuiWrapper gui(argc, argv);

	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// Here start the ROS events loop
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	ROS_INFO("Node started.");
	// Now wait for application to finish
	int r = gui.exec();// MUST be called by the Main Thread

	spinner.stop();

	ROS_INFO("All done! Closing...");
	return r;
}
