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

#ifndef RGBDIMAGEVIEWER_H_
#define RGBDIMAGEVIEWER_H_

#include <rtabmap_viz/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <QMainWindow>
#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UEventsSender.h>

namespace rtabmap
{
	class CameraViewer;
}

class QComboBox;
class QSpinBox;
class QLabel;

namespace rtabmap_viz {

class RGBDImageViewer : public QMainWindow, public UEventsSender
{
    Q_OBJECT

public:
	RTABMAP_VIZ_PUBLIC
	explicit RGBDImageViewer(std::shared_ptr<rclcpp::Node> & node, const rtabmap::ParametersMap & parameters);
	virtual ~RGBDImageViewer();

private Q_SLOTS:
    void updateTopicList();
    void topicSelected(const QString & topicName);

private:
	void callback(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr msg);

private:
  QComboBox * topicComboBox_;
  QComboBox * frameComboBox_;
  QSpinBox * spinBox_;
  QLabel * warningLabel_;
	rtabmap::CameraViewer * cameraView_;
	rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImageSub_;
  
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  std::mutex mutex_;
};

}

#endif /* RGBDIMAGEVIEWER_H_ */
