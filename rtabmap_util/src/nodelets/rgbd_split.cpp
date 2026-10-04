/*
Copyright (c) 2010-2022, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap_util/rgbd_split.hpp>
#include <rtabmap/core/Compression.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <sensor_msgs/image_encodings.hpp>
#include <algorithm>
#include <cstring>

#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace rtabmap_util
{

RGBDSplit::RGBDSplit(const rclcpp::NodeOptions & options) :
	Node("rgbd_split", options),
	stereo_(false),
	compressedImageFormat_("jpeg"),
	compressedDepthFormat_("png"),
	compressedDepthMax_(10.0),
	compressedDepthQuantization_(100.0)
{
	int qos = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
	qos = this->declare_parameter("qos", qos);
	// Each side can be set independently so the node can bridge a producer and a
	// consumer that don't agree on reliability. Both default to qos.
	int qosSub = this->declare_parameter("qos_sub", qos);
	int qosPub = this->declare_parameter("qos_pub", qos);
	int queueSub = this->declare_parameter("queue_sub", 5);
	int queuePub = this->declare_parameter("queue_pub", 1);
	// A stereo RGBDImage carries the right image in the depth slot, so name the outputs
	// left/right instead of rgb/depth to say what they really are.
	stereo_ = this->declare_parameter("stereo", false);
	// Republish compressed images on the compressed and compressedDepth topics without
	// decompressing them. Depth can be set apart, e.g., to use the jpeg_quality parameter
	// of the compressed plugin while still passing the depth through.
	bool compressedPassthrough = this->declare_parameter("compressed_passthrough", true);
	bool compressedDepthPassthrough = this->declare_parameter("compressed_depth_passthrough", compressedPassthrough);

	RCLCPP_INFO(this->get_logger(), "%s: qos         = %d", get_name(), qos);
	RCLCPP_INFO(this->get_logger(), "%s: queue_sub   = %d", get_name(), queueSub);
	RCLCPP_INFO(this->get_logger(), "%s: queue_pub   = %d", get_name(), queuePub);
	RCLCPP_INFO(this->get_logger(), "%s: stereo      = %s", get_name(), stereo_?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: compressed_passthrough = %s", get_name(), compressedPassthrough?"true":"false");
	RCLCPP_INFO(this->get_logger(), "%s: compressed_depth_passthrough = %s", get_name(), compressedDepthPassthrough?"true":"false");

	UASSERT_MSG(queueSub >= 1 && queuePub >= 1,
			uFormat("queue_sub (%d) and queue_pub (%d) must be at least 1", queueSub, queuePub).c_str());

	rgbdImageSub_ = create_subscription<rtabmap_msgs::msg::RGBDImage>("rgbd_image", rclcpp::QoS(queueSub).reliability((rmw_qos_reliability_policy_t)qosSub), std::bind(&RGBDSplit::callback, this, std::placeholders::_1));

	const std::string base = rgbdImageSub_->get_topic_name();
	const std::string firstName = stereo_?"/left":"/rgb";
	const std::string secondName = stereo_?"/right":"/depth";
	const rclcpp::QoS pubQos = rclcpp::QoS(queuePub).reliability((rmw_qos_reliability_policy_t)qosPub);

	const std::string rgbTopic = base + firstName + "/image";
	const std::string depthTopic = base + secondName + "/image";

	// Parameter namespace image_transport gives to a topic (see
	// image_transport::Publisher), e.g., "rgbd_image.depth.image".
	const auto paramBaseOf = [this](const std::string & topic) {
		std::string paramBase = topic.substr(std::min(topic.size(), this->get_effective_namespace().size()));
		std::replace(paramBase.begin(), paramBase.end(), '/', '.');
		if(!paramBase.empty() && paramBase.front() == '.')
		{
			paramBase = paramBase.substr(1);
		}
		return paramBase;
	};
	// Disables the image_transport @p transport plugin of @p topic, as we publish on its
	// topic instead. image_transport reads the list of plugins from this parameter when
	// the publisher is created: the plugin is removed from it even if set by the user,
	// otherwise both would publish on the same topic.
	const auto removePlugin = [this, &paramBaseOf](const std::string & topic, const std::string & transport, const std::string & passthroughParam) {
		const std::string name = paramBaseOf(topic) + ".enable_pub_plugins";
		std::vector<std::string> plugins;
		for(const std::string & loadable : image_transport::getLoadableTransports())
		{
			if(loadable != transport)
			{
				plugins.push_back(loadable);
			}
		}
		plugins = this->declare_parameter(name, plugins);
		const auto plugin = std::find(plugins.begin(), plugins.end(), transport);
		if(plugin != plugins.end())
		{
			RCLCPP_WARN(this->get_logger(), "%s: %s is removed from %s, as %s is true: the node "
					"publishes that topic itself. Set %s to false to use the plugin.",
					get_name(), transport.c_str(), name.c_str(), passthroughParam.c_str(), passthroughParam.c_str());
			plugins.erase(plugin);
			this->set_parameter(rclcpp::Parameter(name, plugins));
		}
	};

	if(compressedPassthrough)
	{
		const std::string kCompressed = "image_transport/compressed";
		// Same parameter as compressed_image_transport's publisher ("jpeg_quality" and
		// "png_level" are not supported), read from the color topic.
		const std::string cBase = paramBaseOf(rgbTopic) + ".compressed.";
		compressedImageFormat_ = this->declare_parameter(cBase + "format", compressedImageFormat_);
		if(compressedImageFormat_ != "jpeg" && compressedImageFormat_ != "png")
		{
			RCLCPP_ERROR(this->get_logger(), "%sformat should be \"jpeg\" or \"png\" (\"%s\"), using \"jpeg\".",
					cBase.c_str(), compressedImageFormat_.c_str());
			compressedImageFormat_ = "jpeg";
		}
		removePlugin(rgbTopic, kCompressed, "compressed_passthrough");
		compressedRgbPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(rgbTopic + "/compressed", pubQos);
		if(stereo_)
		{
			removePlugin(depthTopic, kCompressed, "compressed_passthrough");
			compressedRightPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(depthTopic + "/compressed", pubQos);
		}
	}

	if(!stereo_ && compressedDepthPassthrough)
	{
		removePlugin(depthTopic, "image_transport/compressedDepth", "compressed_depth_passthrough");
		// Same parameters as compressed_depth_image_transport's publisher
		// ("png_level" is not supported).
		const std::string cdBase = paramBaseOf(depthTopic) + ".compressedDepth.";
		compressedDepthFormat_ = this->declare_parameter(cdBase + "format", compressedDepthFormat_);
		compressedDepthMax_ = this->declare_parameter(cdBase + "depth_max", compressedDepthMax_);
		compressedDepthQuantization_ = this->declare_parameter(cdBase + "depth_quantization", compressedDepthQuantization_);
#ifdef PRE_ROS_JAZZY
		if(compressedDepthFormat_ == "rvl")
		{
			RCLCPP_ERROR(this->get_logger(), "%sformat \"rvl\" cannot be decoded by compressed_depth_image_transport "
					"before ROS Jazzy, using \"png\".", cdBase.c_str());
			compressedDepthFormat_ = "png";
		}
#endif
		if(compressedDepthFormat_ != "png" && compressedDepthFormat_ != "rvl")
		{
			RCLCPP_ERROR(this->get_logger(), "%sformat should be \"png\" or \"rvl\" (\"%s\"), using \"png\".",
					cdBase.c_str(), compressedDepthFormat_.c_str());
			compressedDepthFormat_ = "png";
		}
		if(compressedDepthMax_ <= 0.0 || compressedDepthQuantization_ <= 0.0)
		{
			RCLCPP_ERROR(this->get_logger(), "%sdepth_max (%f) and %sdepth_quantization (%f) should be positive, using 10 and 100.",
					cdBase.c_str(), compressedDepthMax_, cdBase.c_str(), compressedDepthQuantization_);
			compressedDepthMax_ = 10.0;
			compressedDepthQuantization_ = 100.0;
		}
		compressedDepthPub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(depthTopic + "/compressedDepth", pubQos);
	}

#ifdef PRE_ROS_LYRICAL
	rgbPub_ = image_transport::create_publisher(this, rgbTopic, pubQos.get_rmw_qos_profile());
	depthPub_ = image_transport::create_publisher(this, depthTopic, pubQos.get_rmw_qos_profile());
#else
	rgbPub_ = image_transport::create_publisher(*this, rgbTopic, pubQos);
	depthPub_ = image_transport::create_publisher(*this, depthTopic, pubQos);
#endif
	rgbInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(base + firstName + "/camera_info", pubQos);
	depthInfoPub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(base + secondName + "/camera_info", pubQos);

	// Resolved names: the outputs are derived from the input topic, so a remapping of
	// "rgbd_image" moves all four with it. Print them so it is clear what to subscribe to.
	RCLCPP_INFO(this->get_logger(), "%s: subscribed to:\n   %s", get_name(), rgbdImageSub_->get_topic_name());
	RCLCPP_INFO(this->get_logger(), "%s: publishing:\n   %s,\n   %s,\n   %s,\n   %s",
			get_name(),
			rgbPub_.getTopic().c_str(),
			rgbInfoPub_->get_topic_name(),
			depthPub_.getTopic().c_str(),
			depthInfoPub_->get_topic_name());
}


void RGBDSplit::publishImage(
		const sensor_msgs::msg::Image & raw,
		const sensor_msgs::msg::CompressedImage & compressed,
		const std_msgs::msg::Header & defaultHeader,
		const rtabmap_msgs::msg::RGBDImage::SharedPtr & input,
		const image_transport::Publisher & rawPub,
		const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & compressedPub,
		sensor_msgs::msg::Image & outputImage) const
{
	const bool rawSubscribed = rawPub.getNumSubscribers() > 0;
	const bool compressedSubscribed = compressedPub && compressedPub->get_subscription_count() > 0;

	// compressed without decompression, when compressed_image_transport can read it
	sensor_msgs::msg::CompressedImage outputCompressed;
	bool compressedReady = false;
	if(compressedSubscribed && raw.data.empty() && !compressed.data.empty())
	{
		const std::string format = rtabmap_conversions::compressedImageTransportFormat(compressed.data);
		if(!format.empty())
		{
			outputCompressed = compressed;
			outputCompressed.format = format; // with the encoding, which older producers did not set
			compressedReady = true;
		}
	}

	outputImage.header = defaultHeader;
	cv_bridge::CvImageConstPtr image;
	if(!raw.data.empty())
	{
		if(rawSubscribed)
		{
			// already raw, just copy pointer
			outputImage = raw;
		}
		else
		{
			outputImage.header = raw.header;
		}
		if(compressedSubscribed)
		{
			image = cv_bridge::toCvShare(raw, input);
		}
	}
	else if(!compressed.data.empty() && (rawSubscribed || !compressedReady))
	{
		cv_bridge::CvImagePtr decoded = cv_bridge::toCvCopy(compressed);
		decoded->toImageMsg(outputImage);
		image = decoded;
	}
	else if(!compressed.data.empty())
	{
		outputImage.header = compressed.header;
	}
	if(outputImage.header.frame_id.empty())
	{
		outputImage.header = defaultHeader;
	}

	if(rawSubscribed)
	{
		rawPub.publish(outputImage);
	}
	if(compressedSubscribed)
	{
		if(!compressedReady && image)
		{
			compressedReady = rtabmap_conversions::toCompressedImageMsg(*image, compressedImageFormat_, outputCompressed);
		}
		if(compressedReady)
		{
			outputCompressed.header = outputImage.header;
			compressedPub->publish(outputCompressed);
		}
	}
}

void RGBDSplit::callback(const rtabmap_msgs::msg::RGBDImage::SharedPtr input) const
{
	if(rgbPub_.getNumSubscribers() || (compressedRgbPub_ && compressedRgbPub_->get_subscription_count()))
	{
		sensor_msgs::msg::Image outputImage;
		const sensor_msgs::msg::CameraInfo outputCameraInfo = input->rgb_camera_info;
		publishImage(input->rgb, input->rgb_compressed, input->header, input, rgbPub_, compressedRgbPub_, outputImage);
		rgbInfoPub_->publish(outputCameraInfo);
	}

	const bool rawDepthSubscribed = depthPub_.getNumSubscribers() > 0;
	const bool compressedDepthSubscribed = compressedDepthPub_ && compressedDepthPub_->get_subscription_count() > 0;
	const bool compressedRightSubscribed = compressedRightPub_ && compressedRightPub_->get_subscription_count() > 0;
	if(rawDepthSubscribed || compressedDepthSubscribed || compressedRightSubscribed)
	{
		sensor_msgs::msg::Image outputImage;
		sensor_msgs::msg::CameraInfo outputCameraInfo;
		outputCameraInfo = input->depth_camera_info;

		cv::Mat depth;
		sensor_msgs::msg::CompressedImage outputCompressedDepth;
		bool compressedDepthReady = false;
		if(compressedRightPub_)
		{
			// Right image of a stereo pair
			publishImage(input->depth, input->depth_compressed, input->header, input, depthPub_, compressedRightPub_, outputImage);
		}
		else
		{
			// compressedDepth without decompression, when the depth is already compressed
			// in a format compressed_depth_image_transport can read.
			if(compressedDepthSubscribed && input->depth.data.empty() && !input->depth_compressed.data.empty())
			{
				const bool rosFormat = input->depth_compressed.format.find("compressedDepth") != std::string::npos;
#ifdef PRE_ROS_JAZZY
				const bool rvlSupported = false;
#else
				const bool rvlSupported = true;
#endif
				if(rosFormat && (rvlSupported || input->depth_compressed.format.find(" rvl") == std::string::npos))
				{
					outputCompressedDepth = input->depth_compressed;
					compressedDepthReady = true;
				}
				else
				{
					// RVL is re-compressed as PNG before Jazzy. Legacy 32FC1 format (4 channels
					// PNG) and right images have no compressedDepth equivalent, they are
					// re-compressed below.
					const cv::Mat bytes = rosFormat ?
							rtabmap_conversions::compressedDepthTransportToRtabmap(input->depth_compressed) :
							rtabmap_conversions::compressedMatFromBytes(input->depth_compressed.data, false);
					outputCompressedDepth.header = input->depth_compressed.header;
					compressedDepthReady = rtabmap_conversions::rtabmapToCompressedDepthTransport(bytes, outputCompressedDepth, false);
				}
			}

				if(!input->depth.data.empty())
			{
				if(rawDepthSubscribed)
				{
					// already raw, just copy pointer
					outputImage = input->depth;
				}
				else
				{
					outputImage.header = input->depth.header;
				}
				if(compressedDepthSubscribed)
				{
					depth = cv_bridge::toCvShare(input->depth, input)->image;
				}
			}
			else if(!input->depth_compressed.data.empty() && (rawDepthSubscribed || !compressedDepthReady))
			{
#ifdef CV_BRIDGE_HYDRO
				ROS_ERROR("Unsupported compressed image copy, please upgrade at least to ROS Indigo to use this.");
#else
				cv_bridge::CvImagePtr cvImg = rtabmap_conversions::uncompressDepthImage(input->depth_compressed);
				if(!cvImg->image.empty())
				{
					cvImg->toImageMsg(outputImage);
					depth = cvImg->image;
				}
#endif
			}
			else
			{
				outputImage.header = input->depth_compressed.header;
			}
		}
		if(outputCameraInfo.header.frame_id.empty()) {
			if(outputImage.header.frame_id.empty()) {
				outputCameraInfo.header = input->header;
			}
			else {
				outputCameraInfo.header = outputImage.header;
			}
		}
		if(outputImage.header.frame_id.empty()) {
			if(outputCameraInfo.header.frame_id.empty()) {
				outputImage.header = input->header;
			}
			else {
				outputImage.header = outputCameraInfo.header;
			}
		}
		// The "depth" slot of an RGBDImage holds either a depth image or the right image
		// of a stereo pair, and "stereo" decides which name it goes out under. Warn when
		// the two disagree: the topic name would be lying to every consumer downstream.
		// Both directions only warn and keep forwarding -- publishing a right image on
		// the depth topic is what this node has always done, and setups rely on it.
		if(!outputImage.data.empty())
		{
			const bool isDepth =
					outputImage.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
					outputImage.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
					outputImage.encoding == sensor_msgs::image_encodings::MONO16;
			if(stereo_ && isDepth)
			{
				RCLCPP_WARN_ONCE(this->get_logger(),
						"Parameter \"stereo\" is true, so the second half is published as \"%s\", "
						"but the received image is a depth image (encoding=\"%s\"), not the right "
						"image of a stereo pair. Set \"stereo\" to false to publish it as depth. "
						"(This warning is printed only once)",
						depthPub_.getTopic().c_str(), outputImage.encoding.c_str());
			}
			else if(!stereo_ && !isDepth)
			{
				RCLCPP_WARN_ONCE(this->get_logger(),
						"Parameter \"stereo\" is false, so the second half is published as \"%s\", "
						"but the received image is not a depth image (encoding=\"%s\"): it looks "
						"like the right image of a stereo pair. Set \"stereo\" to true to publish "
						"it under a name that says so. (This warning is printed only once)",
						depthPub_.getTopic().c_str(), outputImage.encoding.c_str());
			}
		}

		if(!compressedRightPub_) // else already published
		{
			if(rawDepthSubscribed)
			{
				depthPub_.publish(outputImage);
			}
			if(compressedDepthSubscribed)
			{
				if(!compressedDepthReady && (depth.type() == CV_32FC1 || depth.type() == CV_16UC1))
				{
					const std::string format = depth.type() == CV_32FC1 ?
							uFormat(".%s:%g:%g", compressedDepthFormat_.c_str(), compressedDepthMax_, compressedDepthQuantization_) :
							"." + compressedDepthFormat_;
					// Same as compressed_depth_image_transport: inverse depth for 32FC1
					compressedDepthReady = rtabmap_conversions::compressDepthImage(depth, format, outputCompressedDepth);
				}
				else if(!compressedDepthReady && !depth.empty())
				{
					RCLCPP_WARN_ONCE(this->get_logger(), "Cannot publish \"%s\" as compressedDepth, it is "
							"not a depth image (type=%d). (This warning is printed only once)",
							compressedDepthPub_->get_topic_name(), depth.type());
				}
				if(compressedDepthReady)
				{
					outputCompressedDepth.header = outputImage.header;
					compressedDepthPub_->publish(outputCompressedDepth);
				}
			}
		}
		depthInfoPub_->publish(outputCameraInfo);
	}
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rtabmap_util::RGBDSplit)

