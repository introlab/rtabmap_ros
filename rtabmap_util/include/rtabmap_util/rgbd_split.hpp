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

#include <rtabmap_util/visibility.h>
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <image_transport/image_transport.hpp>

#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

namespace rtabmap_util
{

class RGBDSplit : public rclcpp::Node
{
public:
	RTABMAP_UTIL_PUBLIC
	explicit RGBDSplit(const rclcpp::NodeOptions & options);

	virtual ~RGBDSplit() {}

	void callback(const rtabmap_msgs::msg::RGBDImage::SharedPtr input) const;

private:
	/// True when the outputs are named left/right rather than rgb/depth.
	bool stereo_;

	rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdImageSub_;

	image_transport::Publisher rgbPub_;
	image_transport::Publisher depthPub_;
	/// Replace the "compressed" image_transport plugin of rgbPub_, and of depthPub_ with
	/// "stereo" (right image), so that images already compressed in the RGBDImage are
	/// republished without being decompressed. Null if "compressed_passthrough" is false.
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressedRgbPub_;
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressedRightPub_;
	/// Format used when an image has to be compressed: "jpeg" or "png".
	std::string compressedImageFormat_;
	/// Same for the "compressedDepth" plugin of depthPub_, without "stereo". Null if
	/// "compressed_depth_passthrough" is false.
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressedDepthPub_;
	/// Format used when depth has to be compressed (raw input, or compressed in a format
	/// that compressed_depth_image_transport cannot read), same parameters as the plugin.
	std::string compressedDepthFormat_;
	double compressedDepthMax_;
	double compressedDepthQuantization_;

	/// Publishes the image (color, left or right) of @p raw or @p compressed on @p rawPub
	/// and on @p compressedPub, without decompressing it when possible.
	void publishImage(
			const sensor_msgs::msg::Image & raw,
			const sensor_msgs::msg::CompressedImage & compressed,
			const std_msgs::msg::Header & defaultHeader,
			const rtabmap_msgs::msg::RGBDImage::SharedPtr & input,
			const image_transport::Publisher & rawPub,
			const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr & compressedPub,
			sensor_msgs::msg::Image & outputImage) const;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbInfoPub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthInfoPub_;
};

}

