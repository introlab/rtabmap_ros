/*
 * RGB2IndexedNode.cpp
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <rtabmap/core/ColorTable.h>
#include <dynamic_reconfigure/server.h>
#include <rtabmap_image/rgb2indConfig.h>

image_transport::Publisher rosPublisher;
rtabmap::ColorTable colorTable(rtabmap::ColorTable::kSize1024);

void callback(rtabmap_image::rgb2indConfig &config, uint32_t level)
{
	if(config.color_table_size == 8)
	{
		colorTable = rtabmap::ColorTable(rtabmap::ColorTable::kSize65536);
	}
	else
	{
		colorTable = rtabmap::ColorTable(1<<(config.color_table_size+3));
	}
}

void imgReceivedCallback(const sensor_msgs::ImageConstPtr & msg)
{
	if(rosPublisher.getNumSubscribers() && msg->data.size())
	{
		cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(msg);

		if(ptr->image.depth() == CV_8U && ptr->image.channels() == 3)
		{
			cv::Mat ind = ptr->image.clone();
			unsigned char * imageData = (unsigned char *)ind.data;
			int widthStep = ind.cols * ind.elemSize();
			for(int i=0; i<ind.rows; ++i)
			{
				for(int j=0; j<ind.cols; ++j)
				{
					unsigned char & b = imageData[i*widthStep+j*3+0];
					unsigned char & g = imageData[i*widthStep+j*3+1];
					unsigned char & r = imageData[i*widthStep+j*3+2];
					int index = (int)colorTable.getIndex(r, g, b);
					colorTable.getRgb(index, r, g , b);
				}
			}
			cv_bridge::CvImage img;
			img.header.stamp = ros::Time::now();
			img.header.frame_id = ptr->header.frame_id;
			img.encoding = ptr->encoding;
			img.image = ind;
			rosPublisher.publish(img.toImageMsg());
		}
		else
		{
			ROS_WARN("Image format should be 8bits - 3 channels");
		}
	}
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "rgb2ind");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber image_sub = it.subscribe("image", 1, imgReceivedCallback);
	rosPublisher = it.advertise("image_indexed", 1);

	dynamic_reconfigure::Server<rtabmap_image::rgb2indConfig> server;
	dynamic_reconfigure::Server<rtabmap_image::rgb2indConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
