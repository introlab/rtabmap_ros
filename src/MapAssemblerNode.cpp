/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <ros/ros.h>
#include "rtabmap/MapData.h"
#include "rtabmap/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

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
		mapDataTopic_ = nh.subscribe("mapData", 1, &MapAssembler::mapDataReceivedCallback, this);

		assembledMapClouds_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_clouds", 1);
		assembledMapScans_ = nh.advertise<sensor_msgs::PointCloud2>("assembled_scans", 1);
	}

	~MapAssembler()
	{
	}

	void mapDataReceivedCallback(const rtabmap::MapDataConstPtr & msg)
	{
		for(unsigned int i=0; i<msg->localTransformIDs.size() && i<msg->localTransforms.size(); ++i)
		{
			int id = msg->localTransformIDs[i];
			if(!uContains(rgbClouds_, id))
			{
				rtabmap::Transform localTransform = transformFromGeometryMsg(msg->localTransforms[i]);
				if(!localTransform.isNull())
				{
					cv::Mat image, depth;
					float depthFx = 0.0f;
					float depthFy = 0.0f;
					float depthCx = 0.0f;
					float depthCy = 0.0f;

					for(unsigned int i=0; i<msg->imageIDs.size() && i<msg->images.size(); ++i)
					{
						if(msg->imageIDs[i] == id)
						{
							image = util3d::uncompressImage(msg->images[i].bytes);
							break;
						}
					}
					for(unsigned int i=0; i<msg->depthIDs.size() && i<msg->depths.size(); ++i)
					{
						if(msg->depthIDs[i] == id)
						{
							depth = util3d::uncompressImage(msg->depths[i].bytes);
							break;
						}
					}
					for(unsigned int i=0; i<msg->depthFxIDs.size() && i<msg->depthFxs.size(); ++i)
					{
						if(msg->depthFxIDs[i] == id)
						{
							depthFx = msg->depthFxs[i];
							break;
						}
					}
					for(unsigned int i=0; i<msg->depthFyIDs.size() && i<msg->depthFys.size(); ++i)
					{
						if(msg->depthFyIDs[i] == id)
						{
							depthFy = msg->depthFys[i];
							break;
						}
					}
					for(unsigned int i=0; i<msg->depthCxIDs.size() && i<msg->depthCxs.size(); ++i)
					{
						if(msg->depthCxIDs[i] == id)
						{
							depthCx = msg->depthCxs[i];
							break;
						}
					}
					for(unsigned int i=0; i<msg->depthCyIDs.size() && i<msg->depthCys.size(); ++i)
					{
						if(msg->depthCyIDs[i] == id)
						{
							depthCy = msg->depthCys[i];
							break;
						}
					}

					if(!image.empty() && !depth.empty() && depthFx > 0.0f && depthFy > 0.0f && depthCx >= 0.0f && depthCy >= 0.0f)
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
						if(depth.type() == CV_8UC1)
						{
							cloud = util3d::cloudFromStereoImages(image, depth, depthCx, depthCy, depthFx, depthFy, cloudDecimation_);
						}
						else
						{
							cloud = util3d::cloudFromDepthRGB(image, depth, depthCx, depthCy, depthFx, depthFy, cloudDecimation_);
						}

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


		for(unsigned int i=0; i<msg->depth2DIDs.size() && i<msg->depth2Ds.size(); ++i)
		{
			if(!uContains(scans_, msg->depth2DIDs[i]))
			{
				cv::Mat depth2d = util3d::uncompressData(msg->depth2Ds[i].bytes);
				if(!depth2d.empty())
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::depth2DToPointCloud(depth2d);
					if(scanVoxelSize_ > 0)
					{
						cloud = util3d::voxelize(cloud, scanVoxelSize_);
					}

					scans_.insert(std::make_pair(msg->depth2DIDs[i], cloud));
				}
			}
		}


		if(assembledMapClouds_.getNumSubscribers())
		{
			// generate the assembled cloud!
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

			for(unsigned int i=0; i<msg->poseIDs.size() && i<msg->poses.size(); ++i)
			{
				Transform pose = transformFromPoseMsg(msg->poses[i]);

				std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter = rgbClouds_.find(msg->poseIDs[i]);
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

			for(unsigned int i=0; i<msg->poseIDs.size() && i<msg->poses.size(); ++i)
			{
				Transform pose = transformFromPoseMsg(msg->poses[i]);

				std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = scans_.find(msg->poseIDs[i]);
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

private:
	int cloudDecimation_;
	double cloudMaxDepth_;
	double cloudVoxelSize_;
	double scanVoxelSize_;

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
