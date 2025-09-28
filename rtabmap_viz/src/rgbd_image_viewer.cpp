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

#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/gui/CameraViewer.h>
#include <rtabmap/core/SensorEvent.h>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>

namespace rtabmap_viz {

RGBDImageViewer::RGBDImageViewer(std::shared_ptr<rclcpp::Node> & node, const rtabmap::ParametersMap & parameters) :
    node_(node)
{
    this->setWindowTitle("rgbd_image_viewer");

    topicComboBox_ = new QComboBox(this);
    topicComboBox_->setSizeAdjustPolicy(QComboBox::SizeAdjustPolicy::AdjustToContents);
    topicComboBox_->setToolTip("Available rtabmap_msgs::RGBDImage topics");
    frameComboBox_ = new QComboBox(this);
    frameComboBox_->setSizeAdjustPolicy(QComboBox::SizeAdjustPolicy::AdjustToContents);
    frameComboBox_->setToolTip("Base frame of the point cloud");
    spinBox_ = new QSpinBox(this);
    spinBox_->setMinimum(0);
    spinBox_->setMaximum(1000);
    spinBox_->setValue(10);
    spinBox_->setSuffix(" ms");
    spinBox_->setToolTip("Maximum time to wait for TF to transform in base frame (0 means latest available)");
    warningLabel_ = new QLabel(this);
    warningLabel_->setStyleSheet("QLabel { color : red; }"); 
	cameraView_ = new rtabmap::CameraViewer(this, parameters);
    cameraView_->registerToEventsManager();
    QPushButton * refreshButton = new QPushButton(this);
    refreshButton->setIcon(style()->standardIcon(QStyle::SP_BrowserReload));
    refreshButton->setToolTip("Refresh topics and frames");

    connect(topicComboBox_, SIGNAL(currentTextChanged(const QString &)), this, SLOT(topicSelected(const QString &)));
    connect(cameraView_, SIGNAL(finished(int)), this, SLOT(close()));
    connect(refreshButton, SIGNAL(clicked()), this, SLOT(updateTopicList()));

    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);

    QHBoxLayout *hlayout = new QHBoxLayout();
    hlayout->addWidget(topicComboBox_);
    hlayout->addWidget(frameComboBox_);
    hlayout->addWidget(spinBox_);
    hlayout->addWidget(refreshButton);
    hlayout->addWidget(warningLabel_);
    hlayout->addStretch();

    layout->addLayout(hlayout);
    layout->addWidget(cameraView_);

    this->setCentralWidget(centralWidget);

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    updateTopicList();
}

RGBDImageViewer::~RGBDImageViewer()
{
}

void RGBDImageViewer::updateTopicList() {
    std::map<std::string, std::vector<std::string>> topicNames = node_->get_topic_names_and_types();
    topicComboBox_->clear();
    for(auto topic: topicNames) {
        for(auto type: topic.second) {
            if(type == "rtabmap_msgs/msg/RGBDImage") {
                topicComboBox_->addItem(topic.first.c_str());
            }
        }
    }
    std::vector<std::string> frames = tfBuffer_->getAllFrameNames();
    frameComboBox_->clear();
    frameComboBox_->addItem("<camera>");
    for(auto & frame: frames) {
        frameComboBox_->addItem(frame.c_str());
    }
}

void RGBDImageViewer::topicSelected(const QString & topicName) {
    rgbdImageSub_.reset();
    if(!topicName.isEmpty()) {
        rgbdImageSub_ = node_->create_subscription<rtabmap_msgs::msg::RGBDImage>(topicName.toStdString(), rclcpp::QoS(1), std::bind(&RGBDImageViewer::callback, this, std::placeholders::_1));
    }
}

void RGBDImageViewer::callback(
		const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr msg)
{
    bool warned = false;
    rtabmap::SensorData data = rtabmap_conversions::rgbdImageFromROS(msg);
    if(!frameComboBox_->currentText().isEmpty() && (!data.cameraModels().empty() || !data.stereoCameraModels().empty())) {
        rtabmap::Transform localTransform;
        if(frameComboBox_->currentText().compare("<camera>") == 0) {
            localTransform = rtabmap::CameraModel::opticalRotation();
        }
        else {
            localTransform = rtabmap_conversions::getTransform(
                frameComboBox_->currentText().toStdString(),
                msg->header.frame_id,
                msg->header.stamp,
                *tfBuffer_,
                double(spinBox_->value()) / 1000.0);
        }
        if(localTransform.isNull())
        {
            QString log = QString("Could not get TF between \"%1\" and \"%2\" frames for stamp %3 after waiting %4 ms.")
                .arg(frameComboBox_->currentText())
                .arg(msg->header.frame_id.c_str())
                .arg(QString::number(rclcpp::Time(msg->header.stamp).seconds(), 'f', 3))
                .arg(spinBox_->value());
            warningLabel_->setToolTip(log);
            QMetaObject::invokeMethod(warningLabel_, "setText", Q_ARG(QString, log));
            warned = true;
        }
        
        if(!data.cameraModels().empty()) {
            rtabmap::CameraModel model = data.cameraModels()[0];
            model.setLocalTransform(localTransform);
            data.setCameraModel(model);
        }
        else {
            rtabmap::StereoCameraModel model = data.stereoCameraModels()[0];
            model.setLocalTransform(localTransform);
            data.setStereoCameraModel(model);
        }
    }
    if(!warned) {
        warningLabel_->setToolTip("");
        QMetaObject::invokeMethod(warningLabel_, "clear");
    }

    this->post(new rtabmap::SensorEvent(data));
}

}
