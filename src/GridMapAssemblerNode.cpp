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
#include <nav_msgs/GetMap.h>

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
#include <pcl/common/common.h>
#include <laser_geometry/laser_geometry.h>

using namespace rtabmap;

class GridMapAssembler
{

public:
	GridMapAssembler() :
		scanVoxelSize_(0.01)
	{
		ros::NodeHandle pnh("~");
		pnh.param("scan_voxel_size", scanVoxelSize_, scanVoxelSize_);

		ros::NodeHandle nh;
		infoExTopic_ = nh.subscribe("infoEx", 1, &GridMapAssembler::infoExReceivedCallback, this);
		mapDataTopic_ = nh.subscribe("mapData", 1, &GridMapAssembler::mapDataReceivedCallback, this);

		gridMap_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
		getMapService_ = nh.advertiseService("get_map", &GridMapAssembler::getMapCallback, this);
	}

	~GridMapAssembler()
	{
	}

	void infoExReceivedCallback(const rtabmap::InfoExConstPtr & msg)
	{
		for(unsigned int i=0; i<msg->data.depth2DIDs.size() && i<msg->data.depth2Ds.size(); ++i)
		{
			if(!uContains(scans_, msg->data.depth2DIDs[i]))
			{
				cv::Mat depth2d = util3d::uncompressData(msg->data.depth2Ds[i].bytes);
				scans_.insert(std::make_pair(msg->data.depth2DIDs[i], util3d::depth2DToPointCloud(depth2d)));
			}
		}

		std::map<int, Transform> poses;
		for(unsigned int i=0; i<msg->data.poseIDs.size() && i<msg->data.poses.size(); ++i)
		{
			poses.insert(std::make_pair(msg->data.poseIDs[i], transformFromPoseMsg(msg->data.poses[i])));
		}

		if(gridMap_.getNumSubscribers())
		{
			float delta = 0.05; //m

			// create the map
			float xMin=0.0f, yMin=0.0f;
			cv::Mat pixels = util3d::create2DMap(poses, scans_, delta, xMin, yMin);

			if(!pixels.empty())
			{
				//init
				map_.info.resolution = delta;
				map_.info.origin.position.x = 0.0;
				map_.info.origin.position.y = 0.0;
				map_.info.origin.position.z = 0.0;
				map_.info.origin.orientation.x = 0.0;
				map_.info.origin.orientation.y = 0.0;
				map_.info.origin.orientation.z = 0.0;
				map_.info.origin.orientation.w = 1.0;

				map_.info.width = pixels.cols;
				map_.info.height = pixels.rows;
				map_.info.origin.position.x = xMin;
				map_.info.origin.position.y = yMin;
				map_.data.resize(map_.info.width * map_.info.height);

				memcpy(map_.data.data(), pixels.data, map_.info.width * map_.info.height);

				map_.header.frame_id = msg->header.frame_id;
				map_.header.stamp = ros::Time::now();

				gridMap_.publish(map_);
			}
		}
	}

	void mapDataReceivedCallback(const rtabmap::MapDataConstPtr & msg)
	{
		std::map<int, std::vector<unsigned char> > depths2d;

		if(msg->depth2DIDs.size() != msg->depth2Ds.size())
		{
			ROS_WARN("grid_map_assembler: receiving map... depths2D and depth2DIDs are not the same size (%d vs %d)!",
					(int)msg->depth2Ds.size(), (int)msg->depth2DIDs.size());
		}

		// fill maps
		for(unsigned int i=0; i<msg->depth2DIDs.size() && i < msg->depth2Ds.size(); ++i)
		{
			if(!uContains(scans_, msg->depth2DIDs[i]))
			{
				cv::Mat depth2d = util3d::uncompressData(msg->depth2Ds[i].bytes);
				scans_.insert(std::make_pair(msg->depth2DIDs[i], util3d::depth2DToPointCloud(depth2d)));
			}
		}
	}

	bool getMapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res)
	{
		if(map_.data.size())
		{
			res.map = map_;
			return true;
		}
		return false;
	}

private:
	double scanVoxelSize_;

	ros::Subscriber infoExTopic_;
	ros::Subscriber mapDataTopic_;

	ros::Publisher gridMap_;

	ros::ServiceServer getMapService_;

	std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans_;

	nav_msgs::OccupancyGrid map_;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "grid_map_assembler");
	GridMapAssembler assembler;
	ros::spin();
	return 0;
}
