/*
 * GuiWrapper.cpp
 *
 *  Created on: 4 févr. 2010
 *      Author: labm2414
 */

#include "GuiWrapper.h"
#include <QtGui/QApplication>
#include <QtCore/QDir>

#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/gui/MainWindow.h>
#include <rtabmap/core/RtabmapEvent.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/util3d.h>

#include "rtabmap/MsgConversion.h"

#include "PreferencesDialogROS.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

using namespace rtabmap;

class MapAssembler
{

public:
	MapAssembler() :
		cloudDecimation_(4),
		cloudMaxDepth_(4.0),
		cloudVoxelSize_(0.02),
		scanVoxelSize_(0.01)
	{
		ros::NodeHandle pnh("~");
		pnh.param("cloud_decimation", cloudDecimation_, cloudDecimation_);
		pnh.param("cloud_max_depth", cloudMaxDepth_, cloudMaxDepth_);
		pnh.param("cloud_voxel_size", cloudVoxelSize_, cloudVoxelSize_);
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);

		ros::NodeHandle nh;
		infoExTopic_ = nh.subscribe("infoEx", 1, &MapAssembler::infoExReceivedCallback, this);
		mapDataTopic_ = nh.subscribe("mapData", 1, &MapAssembler::mapDataReceivedCallback, this);

		assembledMapClouds_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_clouds", 1);
		assembledMapScans_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_scans", 1);
	}

	~MapAssembler()
	{
	}

	void infoExReceivedCallback(const rtabmap::InfoExConstPtr & msg)
	{
		for(unsigned int i=0; i<msg->data.localTransformIDs.size() && i<msg->data.localTransforms.size(); ++i)
		{
			int id = msg->data.localTransformIDs[i];
			if(!uContains(rgbClouds_, id))
			{
				rtabmap::Transform localTransform = transformFromGeometryMsg(msg->data.localTransforms[i]);
				if(!localTransform.isNull())
				{
					cv::Mat image, depth;
					float depthConstant = 0.0f;

					for(unsigned int i=0; i<msg->data.imageIDs.size() && i<msg->data.images.size(); ++i)
					{
						if(msg->data.imageIDs[i] == id)
						{
							image = util3d::uncompressImage(msg->data.images[i].bytes);
							break;
						}
					}
					for(unsigned int i=0; i<msg->data.depthIDs.size() && i<msg->data.depths.size(); ++i)
					{
						if(msg->data.depthIDs[i] == id)
						{
							depth = util3d::uncompressImage(msg->data.depths[i].bytes);
							break;
						}
					}
					for(unsigned int i=0; i<msg->data.depthConstantIDs.size() && i<msg->data.depthConstants.size(); ++i)
					{
						if(msg->data.depthConstantIDs[i] == id)
						{
							depthConstant = msg->data.depthConstants[i];
							break;
						}
					}

					if(!image.empty() && !depth.empty() && depthConstant > 0.0f)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(image, depth, depthConstant, cloudDecimation_);

						if(cloudMaxDepth_ > 0)
						{
							cloud = util3d::passThrough(cloud, "z", 0, cloudMaxDepth_);
						}
						if(cloudVoxelSize_ > 0)
						{
							cloud = util3d::voxelize(cloud, cloudVoxelSize_);
						}

						cloud = util3d::transformPointCloud(cloud, localTransform);

						rgbClouds_.insert(std::make_pair(id, cloud));
					}
				}
			}
		}


		for(unsigned int i=0; i<msg->data.depth2DIDs.size() && i<msg->data.depth2Ds.size(); ++i)
		{
			if(!uContains(scans_, msg->data.depth2DIDs[i]))
			{
				cv::Mat depth2d = util3d::uncompressData(msg->data.depth2Ds[i].bytes);
				if(!depth2d.empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::depth2DToPointCloud(depth2d);
					if(scanVoxelSize_ > 0)
					{
						cloud = util3d::voxelize(cloud, scanVoxelSize_);
					}

					scans_.insert(std::make_pair(msg->data.depth2DIDs[i], cloud));
				}
			}
		}


		if(assembledMapClouds_.getNumSubscribers())
		{
			// generate the assembled cloud!
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

			for(unsigned int i=0; i<msg->data.poseIDs.size() && i<msg->data.poses.size(); ++i)
			{
				Transform pose = transformFromPoseMsg(msg->data.poses[i]);

				std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter = rgbClouds_.find(msg->data.poseIDs[i]);
				if(iter != rgbClouds_.end())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(iter->second, pose);
					*assembledCloud+=*transformed;
				}
			}

			if(assembledCloud->size())
			{
				if(cloudVoxelSize_ > 0)
				{
					assembledCloud = util3d::voxelize(assembledCloud,cloudVoxelSize_);
				}

				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledCloud, *cloudMsg);
				cloudMsg->header.stamp = ros::Time::now();
				cloudMsg->header.frame_id = msg->header.frame_id;
				assembledMapClouds_.publish(cloudMsg);
			}
		}

		if(assembledMapScans_.getNumSubscribers())
		{
			// generate the assembled scan!
			pcl::PointCloud<pcl::PointXYZ>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZ>);

			for(unsigned int i=0; i<msg->data.poseIDs.size() && i<msg->data.poses.size(); ++i)
			{
				Transform pose = transformFromPoseMsg(msg->data.poses[i]);

				std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = scans_.find(msg->data.poseIDs[i]);
				if(iter != scans_.end())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = util3d::transformPointCloud(iter->second, pose);
					*assembledCloud+=*transformed;
				}
			}

			if(assembledCloud->size())
			{
				if(scanVoxelSize_ > 0)
				{
					assembledCloud = util3d::voxelize(assembledCloud, scanVoxelSize_);
				}

				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledCloud, *cloudMsg);
				cloudMsg->header.stamp = ros::Time::now();
				cloudMsg->header.frame_id = msg->header.frame_id;
				assembledMapScans_.publish(cloudMsg);
			}
		}
	}

	void mapDataReceivedCallback(const rtabmap::MapDataConstPtr & msg)
	{
		std::map<int, std::vector<unsigned char> > images;
		std::map<int, std::vector<unsigned char> > depths;
		std::map<int, std::vector<unsigned char> > depths2d;
		std::map<int, float> depthConstants;
		std::map<int, Transform> localTransforms;
		std::map<int, Transform> poses;

		if(msg->imageIDs.size() != msg->images.size())
		{
			ROS_WARN("rtabmapviz: receiving map... images and IDs are not the same size (%d vs %d)!",
					(int)msg->images.size(), (int)msg->imageIDs.size());
		}

		if(msg->depthIDs.size() != msg->depths.size())
		{
			ROS_WARN("rtabmapviz: receiving map... depths and IDs are not the same size (%d vs %d)!",
					(int)msg->depths.size(), (int)msg->depthIDs.size());
		}

		if(msg->depthConstantIDs.size() != msg->depthConstants.size())
		{
			ROS_WARN("rtabmapviz: receiving map... depthConstants and IDs are not the same size (%d vs %d)!",
					(int)msg->depthConstants.size(), (int)msg->depthConstantIDs.size());
		}

		if(msg->depth2DIDs.size() != msg->depth2Ds.size())
		{
			ROS_WARN("rtabmapviz: receiving map... depths2D and depth2DIDs are not the same size (%d vs %d)!",
					(int)msg->depth2Ds.size(), (int)msg->depth2DIDs.size());
		}

		// fill maps
		for(unsigned int i=0; i<msg->imageIDs.size() && i < msg->images.size(); ++i)
		{
			if(!uContains(rgbClouds_, msg->imageIDs[i]))
			{
				images.insert(std::make_pair(msg->imageIDs[i], msg->images[i].bytes));
			}
		}

		for(unsigned int i=0; i<msg->depthIDs.size() && i < msg->depths.size(); ++i)
		{
			if(!uContains(rgbClouds_, msg->depthIDs[i]))
			{
				depths.insert(std::make_pair(msg->depthIDs[i], msg->depths[i].bytes));
			}
		}

		for(unsigned int i=0; i<msg->depthConstantIDs.size() && i < msg->depthConstants.size(); ++i)
		{
			if(!uContains(rgbClouds_, msg->depthConstantIDs[i]))
			{
				depthConstants.insert(std::make_pair(msg->depthConstantIDs[i], msg->depthConstants[i]));
			}
		}

		for(unsigned int i=0; i<msg->localTransformIDs.size() && i < msg->localTransforms.size(); ++i)
		{
			if(!uContains(rgbClouds_, msg->localTransformIDs[i]))
			{
				Transform t = transformFromGeometryMsg(msg->localTransforms[i]);
				localTransforms.insert(std::make_pair(msg->localTransformIDs[i], t));
			}
		}

		for(unsigned int i=0; i<msg->depth2DIDs.size() && i < msg->depth2Ds.size(); ++i)
		{
			if(!uContains(scans_, msg->depth2DIDs[i]))
			{
				cv::Mat depth2d = util3d::uncompressData(msg->depth2Ds[i].bytes);
				scans_.insert(std::make_pair(msg->depth2DIDs[i], util3d::depth2DToPointCloud(depth2d)));
			}
		}

		// create clouds
		for(std::map<int, std::vector<unsigned char> >::iterator iter = images.begin(); iter!=images.end(); ++iter)
		{
			if(uContains(depths, iter->first) && uContains(depthConstants, iter->first) && uContains(localTransforms, iter->first))
			{
				cv::Mat image = util3d::uncompressImage(iter->second);
				cv::Mat depth = util3d::uncompressImage(depths.at(iter->first));
				float depthConstant = depthConstants.at(iter->first);
				rtabmap::Transform localTransform = localTransforms.at(iter->first);
				rgbClouds_.insert(std::make_pair(iter->first, util3d::cloudFromDepthRGB(image, depth, depthConstant, cloudDecimation_)));
			}
		}
	}

private:
	int cloudDecimation_;
	double cloudMaxDepth_;
	double cloudVoxelSize_;
	double scanVoxelSize_;

	ros::Subscriber infoExTopic_;
	ros::Subscriber mapDataTopic_;

	ros::Publisher assembledMapClouds_;
	ros::Publisher assembledMapScans_;

	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > rgbClouds_;
	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_assembler");
	MapAssembler assembler;
	ros::spin();
	return 0;
}
