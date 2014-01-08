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
			cv::Mat pixels = create2DMap(poses, delta, xMin, yMin);

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

	cv::Mat create2DMap(const std::map<int, Transform> & poses, float delta, float & xMin, float & yMin)
	{
		std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr > scans;

		pcl::PointCloud<pcl::PointXYZ> minMax;
		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			if(uContains(scans_, iter->first))
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::transformPointCloud(scans_.at(iter->first), iter->second);
				pcl::PointXYZ min, max;
				pcl::getMinMax3D(*cloud, min, max);
				minMax.push_back(min);
				minMax.push_back(max);
				minMax.push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
				scans.insert(std::make_pair(iter->first, cloud));
			}
		}

		cv::Mat map;
		if(minMax.size())
		{
			//Get map size
			pcl::PointXYZ min, max;
			pcl::getMinMax3D(minMax, min, max);

			xMin = min.x-1.0f;
			yMin = min.y-1.0f;
			float xMax = max.x+1.0f;
			float yMax = max.y+1.0f;

			map = cv::Mat::ones((yMax - yMin) / delta, (xMax - xMin) / delta, CV_8S)*-1;
			for(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator iter = scans.begin(); iter!=scans.end(); ++iter)
			{
				for(unsigned int i=0; i<iter->second->size(); ++i)
				{
					const Transform & pose = poses.at(iter->first);
					cv::Point2i start((pose.x()-xMin)/delta + 0.5f, (pose.y()-yMin)/delta + 0.5f);
					cv::Point2i end((iter->second->points[i].x-xMin)/delta + 0.5f, (iter->second->points[i].y-yMin)/delta + 0.5f);

					rayTrace(start, end, map); // trace free space

					map.at<char>(end.y, end.x) = 100; // obstacle
				}
			}
		}
		return map;
	}

	void rayTrace(const cv::Point2i & start, const cv::Point2i & end, cv::Mat & grid)
	{
		UASSERT_MSG(start.x >= 0 && start.x < grid.cols, uFormat("start.x=%d grid.cols=%d", start.x, grid.cols).c_str());
		UASSERT_MSG(start.y >= 0 && start.y < grid.rows, uFormat("start.y=%d grid.rows=%d", start.y, grid.rows).c_str());
		UASSERT_MSG(end.x >= 0 && end.x < grid.cols, uFormat("end.x=%d grid.cols=%d", end.x, grid.cols).c_str());
		UASSERT_MSG(end.y >= 0 && end.y < grid.rows, uFormat("end.x=%d grid.cols=%d", end.y, grid.rows).c_str());

		cv::Point2i ptA, ptB;
		if(start.x > end.x)
		{
			ptA = end;
			ptB = start;
		}
		else
		{
			ptA = start;
			ptB = end;
		}

		float slope = float(ptB.y - ptA.y)/float(ptB.x - ptA.x);
		float b = ptA.y - slope*ptA.x;


		//ROS_WARN("start=%d,%d end=%d,%d", ptA.x, ptA.y, ptB.x, ptB.y);

		//ROS_WARN("y = %f*x + %f", slope, b);

		for(int x=ptA.x; x<ptB.x; ++x)
		{
			float lowerbound = float(x)*slope + b;
			float upperbound = float(x+1)*slope + b;

			if(lowerbound > upperbound)
			{
				float tmp = lowerbound;
				lowerbound = upperbound;
				upperbound = tmp;
			}

			//ROS_WARN("lowerbound=%f upperbound=%f", lowerbound, upperbound);
			UASSERT_MSG(lowerbound >= 0 && lowerbound < grid.rows, uFormat("lowerbound=%f grid.cols=%d x=%d slope=%f b=%f", lowerbound, grid.cols, x, slope, b).c_str());
			UASSERT_MSG(upperbound >= 0 && upperbound < grid.rows, uFormat("upperbound=%f grid.cols=%d x+1=%d slope=%f b=%f", upperbound, grid.cols, x+1, slope, b).c_str());
			for(int y = lowerbound; y<=(int)upperbound; ++y)
			{
				if(grid.at<char>(y, x) == -1)
				{
					grid.at<char>(y, x) = 0; // free space
				}
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
