
#include <ros/ros.h>

#include "rtabmap/utilite/ULogger.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "rtabmap/MsgConversion.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

#include <rtabmap/core/util3d.h>

#include <QtGui/QApplication>

#include "rtabmap/gui/DataRecorder.h"

using namespace rtabmap;


class DataRecorderWrapper
{
public:
	DataRecorderWrapper() :
		fileName_("output.db"),
		frameId_("base_link"),
		depthScanSync_(0),
		depthSync_(0),
		scanSync_(0),
		depthImageSync_(0)
	{
		ros::NodeHandle pnh("~");

		bool subscribeOdometry = false;
		bool subscribeLaserScan = false;
		bool subscribeDepth = false;
		int queueSize = 10;
		pnh.param("subscribe_odometry", subscribeOdometry, subscribeOdometry);
		pnh.param("subscribe_depth", subscribeDepth, subscribeDepth);
		pnh.param("subscribe_laserScan", subscribeLaserScan, subscribeLaserScan);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("output_file_name", fileName_, fileName_);
		pnh.param("frame_id", frameId_, frameId_);

		setupCallbacks(subscribeOdometry, subscribeDepth, subscribeLaserScan, queueSize);
	}
	bool init()
	{
		return recorder_.init(fileName_.c_str());
	}

	virtual ~DataRecorderWrapper()
	{
		if(depthScanSync_)
			delete depthScanSync_;
		if(depthSync_)
			delete depthSync_;
		if(scanSync_)
			delete scanSync_;
		if(depthImageSync_)
			delete depthImageSync_;
	}

private:
	void setupCallbacks(
			bool subscribeOdom,
			bool subscribeDepth,
			bool subscribeLaserScan,
			int queueSize)
	{
		ros::NodeHandle nh; // public
		ros::NodeHandle pnh("~"); // private
		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		if(subscribeOdom && subscribeDepth && subscribeLaserScan)
		{
			ROS_INFO("Registering Depth+LaserScan callback...");
			imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
			imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
			cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
			odomSub_.subscribe(nh, "odom", 1);
			scanSub_.subscribe(nh, "scan", 1);
			depthScanSync_ = new message_filters::Synchronizer<MyDepthScanSyncPolicy>(MyDepthScanSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_, scanSub_);
			depthScanSync_->registerCallback(boost::bind(&DataRecorderWrapper::depthScanCallback, this, _1, _2, _3, _4, _5));
		}
		else if(subscribeOdom && subscribeDepth && !subscribeLaserScan)
		{
			ROS_INFO("Registering Depth callback...");
			imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
			imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
			cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
			odomSub_.subscribe(nh, "odom", 1);
			depthSync_ = new message_filters::Synchronizer<MyDepthSyncPolicy>(MyDepthSyncPolicy(queueSize), imageSub_, odomSub_, imageDepthSub_, cameraInfoSub_);
			depthSync_->registerCallback(boost::bind(&DataRecorderWrapper::depthCallback, this, _1, _2, _3, _4));
		}
		else if(subscribeOdom && !subscribeDepth && subscribeLaserScan)
		{
			ROS_INFO("Registering LaserScan callback...");
			imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
			odomSub_.subscribe(nh, "odom", 1);
			scanSub_.subscribe(nh, "scan", 1);
			scanSync_ = new message_filters::Synchronizer<MyScanSyncPolicy>(MyScanSyncPolicy(queueSize), imageSub_, odomSub_, scanSub_);
			scanSync_->registerCallback(boost::bind(&DataRecorderWrapper::scanCallback, this, _1, _2, _3));
		}
		else if(!subscribeOdom && subscribeDepth)
		{
			ROS_INFO("Registering to depth without odometry callback...");
			imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
			imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
			cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
			depthImageSync_ = new message_filters::Synchronizer<MyDepthImageSyncPolicy>(MyDepthImageSyncPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
			depthImageSync_->registerCallback(boost::bind(&DataRecorderWrapper::depthImageCallback, this, _1, _2, _3));
		}
		else
		{
			ROS_INFO("Registering default callback...");
			defaultSub_ = rgb_it.subscribe("image", 1, &DataRecorderWrapper::defaultCallback, this);
		}
	}

	void defaultCallback(const sensor_msgs::ImageConstPtr & imageMsg)
	{
		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		rtabmap::Image image(
			ptrImage->image.clone(),
			cv::Mat(),
			cv::Mat(),
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			Transform(),
			Transform());
		recorder_.addData(image);
	}

	void depthImageCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		float depthFx = cameraInfoMsg->K[0];
		float depthFy = cameraInfoMsg->K[4];
		float depthCx = cameraInfoMsg->K[2];
		float depthCy = cameraInfoMsg->K[5];

		cv::Mat depth16;
		if(ptrDepth->image.type() != CV_16UC1)
		{
			if(ptrDepth->image.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(ptrDepth->image);
				static bool shown = false;
				if(!shown)
				{
					ROS_WARN("Use depth image with \"unsigned short\" type to "
							 "avoid conversion. This message is only printed once...");
					shown = true;
				}
			}
			else
			{
				ROS_ERROR("Depth image must be of type \"unsigned short\"!");
				return;
			}
		}
		else
		{
			depth16 = ptrDepth->image;
		}

		rtabmap::Image image(
			ptrImage->image.clone(),
			depth16,
			cv::Mat(),
			depthFx,
			depthFy,
			depthCx,
			depthCy,
			Transform(),
			localTransform);
		recorder_.addData(image);
	}

	void depthCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		Transform odom = transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		float depthFx = cameraInfoMsg->K[0];
		float depthFy = cameraInfoMsg->K[4];
		float depthCx = cameraInfoMsg->K[2];
		float depthCy = cameraInfoMsg->K[5];

		cv::Mat depth16;
		if(ptrDepth->image.type() != CV_16UC1)
		{
			if(ptrDepth->image.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(ptrDepth->image);
				static bool shown = false;
				if(!shown)
				{
					ROS_WARN("Use depth image with \"unsigned short\" type to "
							 "avoid conversion. This message is only printed once...");
					shown = true;
				}
			}
			else
			{
				ROS_ERROR("Depth image must be of type \"unsigned short\"!");
				return;
			}
		}
		else
		{
			depth16 = ptrDepth->image;
		}

		rtabmap::Image image(
			ptrImage->image.clone(),
			depth16,
			cv::Mat(),
			depthFx,
			depthFy,
			depthCx,
			depthCy,
			odom,
			localTransform);
		recorder_.addData(image);
	}

	void scanCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg)
	{
		// TF ready?
		try
		{
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ> pclScan;
		pcl::fromROSMsg(scanOut, pclScan);
		cv::Mat scan = util3d::depth2DFromPointCloud(pclScan);

		Transform odom = transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");

		rtabmap::Image image(
			ptrImage->image.clone(),
			cv::Mat(),
			scan,
			0.0f,
			0.0f,
			0.0f,
			0.0f,
			odom,
			Transform());
		recorder_.addData(image);

	}

	void depthScanCallback(
			const sensor_msgs::ImageConstPtr& imageMsg,
			const nav_msgs::OdometryConstPtr & odomMsg,
			const sensor_msgs::ImageConstPtr& depthMsg,
			const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
			const sensor_msgs::LaserScanConstPtr& scanMsg)
	{
		// TF ready?
		Transform localTransform;
		try
		{
			tf::StampedTransform tmp;
			tfListener_.lookupTransform(frameId_, scanMsg->header.frame_id, scanMsg->header.stamp, tmp);
			tfListener_.lookupTransform(frameId_, depthMsg->header.frame_id, depthMsg->header.stamp, tmp);
			localTransform = transformFromTF(tmp);
		}
		catch(tf::TransformException & ex)
		{
			ROS_WARN("%s",ex.what());
			return;
		}

		//transform in frameId_ frame
		sensor_msgs::PointCloud2 scanOut;
		laser_geometry::LaserProjection projection;
		projection.transformLaserScanToPointCloud(frameId_, *scanMsg, scanOut, tfListener_);
		pcl::PointCloud<pcl::PointXYZ> pclScan;
		pcl::fromROSMsg(scanOut, pclScan);
		cv::Mat scan = util3d::depth2DFromPointCloud(pclScan);

		Transform odom = transformFromPoseMsg(odomMsg->pose.pose);

		cv_bridge::CvImageConstPtr ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
		cv_bridge::CvImageConstPtr ptrDepth = cv_bridge::toCvShare(depthMsg);

		float depthFx = cameraInfoMsg->K[0];
		float depthFy = cameraInfoMsg->K[4];
		float depthCx = cameraInfoMsg->K[2];
		float depthCy = cameraInfoMsg->K[5];

		cv::Mat depth16;
		if(ptrDepth->image.type() != CV_16UC1)
		{
			if(ptrDepth->image.type() == CV_32FC1)
			{
				//convert to 16 bits
				depth16 = util3d::cvtDepthFromFloat(ptrDepth->image);
				static bool shown = false;
				if(!shown)
				{
					ROS_WARN("Use depth image with \"unsigned short\" type to "
							 "avoid conversion. This message is only printed once...");
					shown = true;
				}
			}
			else
			{
				ROS_ERROR("Depth image must be of type \"unsigned short\"!");
				return;
			}
		}
		else
		{
			depth16 = ptrDepth->image;
		}

		rtabmap::Image image(
			ptrImage->image.clone(),
			depth16,
			scan,
			depthFx,
			depthFy,
			depthCx,
			depthCy,
			odom,
			localTransform);
		recorder_.addData(image);
	}

private:
	DataRecorder recorder_;
	std::string fileName_;
	std::string frameId_;

	image_transport::Subscriber defaultSub_;
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo,
			sensor_msgs::LaserScan> MyDepthScanSyncPolicy;
	message_filters::Synchronizer<MyDepthScanSyncPolicy> * depthScanSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthSyncPolicy;
	message_filters::Synchronizer<MyDepthSyncPolicy> * depthSync_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			nav_msgs::Odometry,
			sensor_msgs::LaserScan> MyScanSyncPolicy;
	message_filters::Synchronizer<MyScanSyncPolicy> * scanSync_;

	//without odom
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::Image,
			sensor_msgs::Image,
			sensor_msgs::CameraInfo> MyDepthImageSyncPolicy;
	message_filters::Synchronizer<MyDepthImageSyncPolicy> * depthImageSync_;

	tf::TransformListener tfListener_;
};

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	ros::init(argc, argv, "data_recorder");

	QApplication app(argc, argv);

	DataRecorderWrapper recorder;

	if(recorder.init())
	{
		ros::spin();
	}
	else
	{
		ROS_ERROR("Cannot initialize the recorder! Make sure the parameter output_file_name is set!");
	}
}
