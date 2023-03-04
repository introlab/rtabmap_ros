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

#include "rtabmap_viz/GuiWrapper.h"
#include "rtabmap/utilite/ULogger.h"

#include <QApplication>
#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/utilite/ULogger.h>
#include <signal.h>

QApplication * app = 0;
ros::AsyncSpinner * spinner = 0;

void my_handler(int s){
	ROS_INFO("rtabmapviz: ctrl-c catched! Exiting Qt app...");
	ros::shutdown();
	app->exit(-1);
}

int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	ros::init(argc, argv, "rtabmap_viz");

	app = new QApplication(argc, argv);
	app->connect( app, SIGNAL( lastWindowClosed() ), app, SLOT( quit() ) );

	rtabmap_viz::GuiWrapper * gui = new rtabmap_viz::GuiWrapper(argc, argv);

	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	// Here start the ROS events loop
	spinner = new ros::AsyncSpinner(1); // Use 1 thread
	spinner->start();

	ROS_INFO("rtabmap_viz started.");
	// Now wait for application to finish
	int r = app->exec();// MUST be called by the Main Thread

	ROS_INFO("rtabmap_viz stopping spinner...");
	delete spinner;

	ROS_INFO("rtabmap_viz deleting qt stuff...");
	delete gui;
	delete app;
	ROS_INFO("rtabmap_viz: All done! Closing...");
	return r;
}
