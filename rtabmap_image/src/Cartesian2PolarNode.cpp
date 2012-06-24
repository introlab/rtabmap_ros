/*
 * RGB2IndexedNode.cpp
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <dynamic_reconfigure/server.h>
#include <rtabmap_image/xy2polarConfig.h>

int dp_rays = 128;
int dp_rings = 64;

image_transport::Publisher rosPublisherPolar;
image_transport::Publisher rosPublisherPolarReconstructed;

void callback(rtabmap_image::xy2polarConfig &config, uint32_t level)
{
	dp_rays = config.rays;
	dp_rings = config.rings;
}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(!rosPublisherPolar.getNumSubscribers() && !rosPublisherPolarReconstructed.getNumSubscribers())
	{
		return;
	}

	if(msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		if(ptr->image.depth() == CV_8U && ptr->image.channels() == 3)
		{
			int radiusX = ptr->image.cols/2;
			int radiusY = ptr->image.rows/2;
			int radius = radiusX<radiusY?radiusX:radiusX;
			float M = dp_rings/std::log(radius);
			cv::Mat polar(dp_rays, dp_rings, CV_8UC3);
			IplImage iplPolar = polar;
			IplImage iplImage = ptr->image;
			cvLogPolar( &iplImage, &iplPolar, cvPoint2D32f(radiusX, radiusY), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );

			if(rosPublisherPolar.getNumSubscribers())
			{
				cv_bridge::CvImage img;
				img.header.stamp = ros::Time::now();
				img.header.frame_id = msg->header.frame_id;
				img.encoding = ptr->encoding;
				img.image = polar;
				rosPublisherPolar.publish(img.toImageMsg());
			}

			if(rosPublisherPolarReconstructed.getNumSubscribers())
			{
				cv::Mat reconstructed(ptr->image.rows, ptr->image.cols, ptr->image.type());
				IplImage iplReconstructed = reconstructed;
				cvLogPolar( &iplPolar, &iplReconstructed, cvPoint2D32f(radiusX, radiusY), double(M), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS+CV_WARP_INVERSE_MAP );

				cv_bridge::CvImage img;
				img.header.stamp = ros::Time::now();
				img.header.frame_id = msg->header.frame_id;
				img.encoding = ptr->encoding;
				img.image = reconstructed;
				rosPublisherPolarReconstructed.publish(img.toImageMsg());
			}
		}
	}
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "xy2polar");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);
	rosPublisherPolar = it.advertise("image_polar", 1);
	rosPublisherPolarReconstructed = it.advertise("image_polar_reconstructed", 1);

	dynamic_reconfigure::Server<rtabmap_image::xy2polarConfig> server;
	dynamic_reconfigure::Server<rtabmap_image::xy2polarConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
