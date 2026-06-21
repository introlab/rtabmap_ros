/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap_viz/rgbd_image_viewer.hpp"
#include "rtabmap/utilite/ULogger.h"

#include <QApplication>
#include <rtabmap/gui/CameraViewer.h>
#include <rtabmap/utilite/ULogger.h>
#include <signal.h>

QApplication * app = 0;

void my_handler(int){
	app->exit(-1);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	app = new QApplication(argc, argv);
	app->connect( app, SIGNAL( lastWindowClosed() ), app, SLOT( quit() ) );

	int r;
	{
		auto node = std::make_shared<rclcpp::Node>("rgbd_image_viewer");
		rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argc, argv, true);
		rtabmap_viz::RGBDImageViewer viewer(node, parameters);
    	viewer.show();

		// Catch ctrl-c to close the gui
		// (Place this after QApplication's constructor)
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = my_handler;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);

		// Here start the ROS events loop
		rclcpp::executors::SingleThreadedExecutor executor; //Use 1 thread
		executor.add_node(node);
		auto spin_executor = [&executor]() {
			executor.spin();
		  };

		// Launch executer
		std::thread execution_thread(spin_executor);

		// Now wait for application to finish
		r = app->exec();// MUST be called by the Main Thread

		rclcpp::shutdown();
		execution_thread.join();
	}
	delete app;

	return r;
}
