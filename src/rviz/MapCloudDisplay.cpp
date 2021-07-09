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

#include "MapCloudDisplay.h"

#include <QApplication>
#include <QMessageBox>
#include <QTimer>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rclcpp/clock.hpp"

#include "rviz_common/display.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/properties/int_property.hpp>
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/srv/get_map.hpp>


namespace rtabmap_ros
{


CloudInfo::CloudInfo() :
		manager_(nullptr),
		pose_(rtabmap::Transform::getIdentity()),
		id_(0),
		scene_node_(nullptr)
{}

CloudInfo::~CloudInfo()
{
	clear();
}

void CloudInfo::clear()
{
	if ( scene_node_ )
	{
		manager_->destroySceneNode( scene_node_ );
		scene_node_=nullptr;
	}
}

const std::string MapCloudDisplay::message_status_name_ = "Message";  // NOLINT allow std::string

MapCloudDisplay::MapCloudDisplay()
  : auto_size_(false),
    new_xyz_transformer_(false),
    new_color_transformer_(false),
    needs_retransform_(false),
	transformer_factory_(std::make_unique<rviz_default_plugins::PointCloudTransformerFactory>())
{
	//QIcon icon;
	//this->setIcon(icon);

	style_property_ = new rviz_common::properties::EnumProperty( "Style", "Flat Squares",
										"Rendering mode to use, in order of computational complexity.",
										this, SLOT( updateStyle() ), this );
	style_property_->addOption( "Points", rviz_rendering::PointCloud::RM_POINTS );
	style_property_->addOption( "Squares", rviz_rendering::PointCloud::RM_SQUARES );
	style_property_->addOption( "Flat Squares", rviz_rendering::PointCloud::RM_FLAT_SQUARES );
	style_property_->addOption( "Spheres", rviz_rendering::PointCloud::RM_SPHERES );
	style_property_->addOption( "Boxes", rviz_rendering::PointCloud::RM_BOXES );

	point_world_size_property_ = new rviz_common::properties::FloatProperty( "Size (m)", 0.01,
												  "Point size in meters.",
												  this, SLOT( updateBillboardSize() ), this );
	point_world_size_property_->setMin( 0.0001 );

	point_pixel_size_property_ = new rviz_common::properties::FloatProperty( "Size (Pixels)", 3,
												  "Point size in pixels.",
												  this, SLOT( updateBillboardSize() ), this );
	point_pixel_size_property_->setMin( 1 );

	alpha_property_ = new rviz_common::properties::FloatProperty( "Alpha", 1.0,
										 "Amount of transparency to apply to the points.  Note that this is experimental and does not always look correct.",
										 this, SLOT( updateAlpha() ), this );
	alpha_property_->setMin( 0 );
	alpha_property_->setMax( 1 );

	xyz_transformer_property_ = new rviz_common::properties::EnumProperty( "Position Transformer", "",
												  "Set the transformer to use to set the position of the points.",
												  this, SLOT( updateXyzTransformer() ), this );
	connect( xyz_transformer_property_, SIGNAL( requestOptions( rviz_common::properties::EnumProperty* )),
			 this, SLOT( setXyzTransformerOptions( rviz_common::properties::EnumProperty* )));

	color_transformer_property_ = new rviz_common::properties::EnumProperty( "Color Transformer", "",
													"Set the transformer to use to set the color of the points.",
													this, SLOT( updateColorTransformer() ), this );
	connect( color_transformer_property_, SIGNAL( requestOptions( rviz_common::properties::EnumProperty* )),
			 this, SLOT( setColorTransformerOptions( rviz_common::properties::EnumProperty* )));

	cloud_from_scan_ = new rviz_common::properties::BoolProperty( "Cloud from scan", false,
										 "Create the cloud from laser scans instead of the RGB-D/Stereo images.",
										 this, SLOT( updateCloudParameters() ), this );

	cloud_decimation_ = new rviz_common::properties::IntProperty( "Cloud decimation", 4,
										 "Decimation of the input RGB and depth images before creating the cloud.",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_decimation_->setMin( 1 );
	cloud_decimation_->setMax( 16 );

	cloud_max_depth_ = new rviz_common::properties::FloatProperty( "Cloud max depth (m)", 4.0f,
										 "Maximum depth of the generated clouds.",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_max_depth_->setMin( 0.0f );
	cloud_max_depth_->setMax( 999.0f );

	cloud_min_depth_ = new rviz_common::properties::FloatProperty( "Cloud min depth (m)", 0.0f,
											 "Minimum depth of the generated clouds.",
											 this, SLOT( updateCloudParameters() ), this );
	cloud_min_depth_->setMin( 0.0f );
	cloud_min_depth_->setMax( 999.0f );

	cloud_voxel_size_ = new rviz_common::properties::FloatProperty( "Cloud voxel size (m)", 0.01f,
										 "Voxel size of the generated clouds.",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_voxel_size_->setMin( 0.0f );
	cloud_voxel_size_->setMax( 1.0f );

	cloud_filter_floor_height_ = new rviz_common::properties::FloatProperty( "Filter floor (m)", 0.0f,
										 "Filter the floor up to maximum height set here "
										 "(only appropriate for 2D mapping).",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_filter_floor_height_->setMin( 0.0f );
	cloud_filter_floor_height_->setMax( 999.0f );

	cloud_filter_ceiling_height_ = new rviz_common::properties::FloatProperty( "Filter ceiling (m)", 0.0f,
										 "Filter the ceiling at the specified height set here "
										 "(only appropriate for 2D mapping).",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_filter_ceiling_height_->setMin( 0.0f );
	cloud_filter_ceiling_height_->setMax( 999.0f );

	node_filtering_radius_ = new rviz_common::properties::FloatProperty( "Node filtering radius (m)", 0.0f,
										 "(Disabled=0) Only keep one node in the specified radius.",
										 this, SLOT( updateCloudParameters() ), this );
	node_filtering_radius_->setMin( 0.0f );
	node_filtering_radius_->setMax( 10.0f );

	node_filtering_angle_ = new rviz_common::properties::FloatProperty( "Node filtering angle (degrees)", 30.0f,
										 "(Disabled=0) Only keep one node in the specified angle in the filtering radius.",
										 this, SLOT( updateCloudParameters() ), this );
	node_filtering_angle_->setMin( 0.0f );
	node_filtering_angle_->setMax( 359.0f );

	download_map_ = new rviz_common::properties::BoolProperty( "Download map", false,
										 "Download the optimized global map using rtabmap/GetMap service. This will force to re-create all clouds.",
										 this, SLOT( downloadMap() ), this );

	download_graph_ = new rviz_common::properties::BoolProperty( "Download graph", false,
											 "Download the optimized global graph (without cloud data) using rtabmap/GetMap service.",
											 this, SLOT( downloadGraph() ), this );
}


void MapCloudDisplay::onInitialize()
{
    MFDClass::onInitialize();

	loadTransformers();

	updateStyle();
	updateBillboardSize();
	updateAlpha();
}

void MapCloudDisplay::loadTransformers()
{
	auto plugins = transformer_factory_->getDeclaredPlugins();
	  for (auto const & plugin : plugins) {
	    auto plugin_name_std = plugin.name.toStdString();
	    if (transformers_.count(plugin_name_std) > 0) {
	      RVIZ_COMMON_LOG_ERROR_STREAM("Transformer type " << plugin_name_std << " is already loaded.");
	      continue;
	    }

	    std::shared_ptr<rviz_default_plugins::PointCloudTransformer> trans(transformer_factory_->make(plugin.id));
	    loadTransformer(trans, plugin_name_std, plugin.id.toStdString());
	  }
}

void MapCloudDisplay::loadTransformer(
  std::shared_ptr<rviz_default_plugins::PointCloudTransformer> trans,
  std::string name,
  const std::string & lookup_name)
{
  trans->init();
  connect(trans.get(), SIGNAL(needRetransform()), this, SLOT(causeRetransform()));

  TransformerInfo info;
  info.transformer = trans;
  info.readable_name = name;
  info.lookup_name = lookup_name;

  info.transformer->createProperties(
    this, rviz_default_plugins::PointCloudTransformer::Support_XYZ, info.xyz_props);
  setPropertiesHidden(info.xyz_props, true);

  info.transformer->createProperties(
    this, rviz_default_plugins::PointCloudTransformer::Support_Color, info.color_props);
  setPropertiesHidden(info.color_props, true);

  transformers_[name] = info;
}

void MapCloudDisplay::processMessage( const rtabmap_ros::msg::MapData::ConstSharedPtr msg )
{
	processMapData(*msg);

	this->emitTimeSignal(msg->header.stamp);
}

void MapCloudDisplay::processMapData(const rtabmap_ros::msg::MapData& map)
{
	std::map<int, rtabmap::Transform> poses;
	for(unsigned int i=0; i<map.graph.poses_id.size() && i<map.graph.poses.size(); ++i)
	{
		poses.insert(std::make_pair(map.graph.poses_id[i], rtabmap_ros::transformFromPoseMsg(map.graph.poses[i])));
	}

	// Add new clouds...
	bool fromDepth = !cloud_from_scan_->getBool();
	for(unsigned int i=0; i<map.nodes.size() && i<map.nodes.size(); ++i)
	{
		int id = map.nodes[i].id;

		// Always refresh the cloud if there are data
		rtabmap::Signature s = rtabmap_ros::nodeDataFromROS(map.nodes[i]);
		if((fromDepth &&
			!s.sensorData().imageCompressed().empty() &&
		    !s.sensorData().depthOrRightCompressed().empty() &&
		    (s.sensorData().cameraModels().size() || s.sensorData().stereoCameraModel().isValidForProjection())) ||
		   (!fromDepth && !s.sensorData().laserScanCompressed().isEmpty()))
		{
			cv::Mat image, depth;
			rtabmap::LaserScan scan;

			s.sensorData().uncompressData(fromDepth?&image:0, fromDepth?&depth:0, !fromDepth?&scan:0);

			sensor_msgs::msg::PointCloud2::SharedPtr cloudMsg(new sensor_msgs::msg::PointCloud2);
			if(fromDepth && !s.sensorData().imageRaw().empty() && !s.sensorData().depthOrRightRaw().empty())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				pcl::IndicesPtr validIndices(new std::vector<int>);

				cloud = rtabmap::util3d::cloudRGBFromSensorData(
						s.sensorData(),
						cloud_decimation_->getInt(),
						cloud_max_depth_->getFloat(),
						cloud_min_depth_->getFloat(),
						validIndices.get());

				if(!cloud->empty())
				{
					if(cloud_voxel_size_->getFloat())
					{
						cloud = rtabmap::util3d::voxelize(cloud, validIndices, cloud_voxel_size_->getFloat());
					}

					if(cloud_filter_floor_height_->getFloat() > 0.0f || cloud_filter_ceiling_height_->getFloat() > 0.0f)
					{
						// convert in /odom frame
						cloud = rtabmap::util3d::transformPointCloud(cloud, s.getPose());
						cloud = rtabmap::util3d::passThrough(cloud, "z",
								cloud_filter_floor_height_->getFloat()>0.0f?cloud_filter_floor_height_->getFloat():-999.0f,
								cloud_filter_ceiling_height_->getFloat()>0.0f && (cloud_filter_floor_height_->getFloat()<=0.0f || cloud_filter_ceiling_height_->getFloat()>cloud_filter_floor_height_->getFloat())?cloud_filter_ceiling_height_->getFloat():999.0f);
						// convert back in /base_link frame
						cloud = rtabmap::util3d::transformPointCloud(cloud, s.getPose().inverse());
					}

					if(!cloud->empty())
					{
						pcl::toROSMsg(*cloud, *cloudMsg);
					}
				}
			}
			else if(!fromDepth && !scan.isEmpty())
			{
				scan = rtabmap::util3d::commonFiltering(
						scan,
						1,
						cloud_min_depth_->getFloat(),
						cloud_max_depth_->getFloat(),
						cloud_voxel_size_->getFloat());
				pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
				cloud = rtabmap::util3d::laserScanToPointCloudI(scan, scan.localTransform());
				if(cloud_filter_floor_height_->getFloat() > 0.0f || cloud_filter_ceiling_height_->getFloat() > 0.0f)
				{
					// convert in /odom frame
					cloud = rtabmap::util3d::transformPointCloud(cloud, s.getPose());
					cloud = rtabmap::util3d::passThrough(cloud, "z",
							cloud_filter_floor_height_->getFloat()>0.0f?cloud_filter_floor_height_->getFloat():-999.0f,
							cloud_filter_ceiling_height_->getFloat()>0.0f && (cloud_filter_floor_height_->getFloat()<=0.0f || cloud_filter_ceiling_height_->getFloat()>cloud_filter_floor_height_->getFloat())?cloud_filter_ceiling_height_->getFloat():999.0f);
					// convert back in /base_link frame
					cloud = rtabmap::util3d::transformPointCloud(cloud, s.getPose().inverse());
				}

				if(!cloud->empty())
				{
					pcl::toROSMsg(*cloud, *cloudMsg);
				}
			}

			if(!cloudMsg->data.empty())
			{
				cloudMsg->header = map.header;
				CloudInfoPtr info(new CloudInfo);
				info->message_ = cloudMsg;
				info->pose_ = rtabmap::Transform::getIdentity();
				info->id_ = id;

				if (transformCloud(info, true))
				{
					std::unique_lock<std::mutex> lock(new_clouds_mutex_);
					new_cloud_infos_.erase(id);
					new_cloud_infos_.insert(std::make_pair(id, info));
				}
			}
		}
	}

	// Update graph
	if(node_filtering_angle_->getFloat() > 0.0f && node_filtering_radius_->getFloat() > 0.0f)
	{
		poses = rtabmap::graph::radiusPosesFiltering(poses,
				node_filtering_radius_->getFloat(),
				node_filtering_angle_->getFloat()*CV_PI/180.0);
	}

	{
		std::unique_lock<std::mutex> lock(current_map_mutex_);
		current_map_ = poses;
	}
}

void MapCloudDisplay::setPropertiesHidden( const QList<rviz_common::properties::Property*>& props, bool hide )
{
  for (auto prop : props) {
	prop->setHidden(hide);
  }
}

void MapCloudDisplay::updateTransformers( const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud )
{
  std::string xyz_name = xyz_transformer_property_->getStdString();
  std::string color_name = color_transformer_property_->getStdString();

  xyz_transformer_property_->clearOptions();
  color_transformer_property_->clearOptions();

  // Get the channels that we could potentially render
  typedef std::set<std::pair<uint8_t, std::string>> S_string;
  S_string valid_xyz, valid_color;
  bool cur_xyz_valid = false;
  bool cur_color_valid = false;
  bool has_rgb_transformer = false;
  for (auto transformer : transformers_) {
    const std::string & name = transformer.first;
    const std::shared_ptr<rviz_default_plugins::PointCloudTransformer> & trans = transformer.second.transformer;
    uint32_t mask = trans->supports(cloud);
    if (mask & rviz_default_plugins::PointCloudTransformer::Support_XYZ) {
      valid_xyz.insert(std::make_pair(trans->score(cloud), name));
      if (name == xyz_name) {
        cur_xyz_valid = true;
      }
      xyz_transformer_property_->addOptionStd(name);
    }

    if (mask & rviz_default_plugins::PointCloudTransformer::Support_Color) {
      valid_color.insert(std::make_pair(trans->score(cloud), name));
      if (name == color_name) {
        cur_color_valid = true;
      }
      if (name == "RGB8") {
        has_rgb_transformer = true;
      }
      color_transformer_property_->addOptionStd(name);
    }
  }

  if (!cur_xyz_valid) {
    if (!valid_xyz.empty()) {
      xyz_transformer_property_->setStringStd(valid_xyz.rbegin()->second);
    }
  }

  if (!cur_color_valid) {
    if (!valid_color.empty()) {
      if (has_rgb_transformer) {
        color_transformer_property_->setStringStd("RGB8");
      } else {
        color_transformer_property_->setStringStd(valid_color.rbegin()->second);
      }
    }
  }
}

void MapCloudDisplay::updateAlpha()
{
	for (auto const & cloud_info : cloud_infos_) {
	    bool per_point_alpha = rviz_default_plugins::findChannelIndex(cloud_info.second->message_, "rgba") != -1;
	    cloud_info.second->cloud_->setAlpha(alpha_property_->getFloat(), per_point_alpha);
	  }
}

void MapCloudDisplay::updateStyle()
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    point_world_size_property_->hide();
    point_pixel_size_property_->show();
  } else {
    point_world_size_property_->show();
    point_pixel_size_property_->hide();
  }
  for (auto const & cloud_info : cloud_infos_) {
    cloud_info.second->cloud_->setRenderMode(mode);
  }
  updateBillboardSize();
}

void MapCloudDisplay::updateBillboardSize()
{
  auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());
  float size;
  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
    size = point_pixel_size_property_->getFloat();
  } else {
    size = point_world_size_property_->getFloat();
  }
  for (auto & cloud_info : cloud_infos_) {
    cloud_info.second->cloud_->setDimensions(size, size, size);
  }
  context_->queueRender();
}

void MapCloudDisplay::updateCloudParameters()
{
	// do nothing... only take effect on next generated clouds
}

void MapCloudDisplay::downloadMap()
{
	if(download_map_->getBool())
	{
		QMessageBox * messageBox = new QMessageBox(
				QMessageBox::NoIcon,
				tr("Calling \"%1\" service...").arg("get_map_data"),
				tr("Downloading the map... please wait (rviz could become gray!)"),
				QMessageBox::NoButton);
		messageBox->setAttribute(Qt::WA_DeleteOnClose, true);
		messageBox->show();
		QApplication::processEvents();
		uSleep(100); // hack make sure the text in the QMessageBox is shown...
		QApplication::processEvents();

		auto node = rclcpp::Node::make_shared(rviz_ros_node_.lock()->get_node_name());
		 auto client = node->create_client<rtabmap_ros::srv::GetMap>("get_map_data");
		if(client->wait_for_service(std::chrono::seconds(2)))
		{
			auto request = std::make_shared<rtabmap_ros::srv::GetMap::Request>();
			request->global = true;
			request->optimized = true;
			request->graph_only = false;
			auto result_future = client->async_send_request(request);
			if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_ERROR(node->get_logger(), "Service \"get_map_data\" failed to get the data.");
			}
			else
			{
				auto result = result_future.get();

				messageBox->setText(tr("Creating all clouds (%1 poses and %2 clouds downloaded)...")
						.arg(result->data.graph.poses.size()).arg(result->data.nodes.size()));
				QApplication::processEvents();
				this->reset();
				processMapData(result->data);
				messageBox->setText(tr("Creating all clouds (%1 poses and %2 clouds downloaded)... done!")
						.arg(result->data.graph.poses.size()).arg(result->data.nodes.size()));

				QTimer::singleShot(1000, messageBox, SLOT(close()));
			}
		}
		else
		{
			std::string msg = uFormat("MapCloudDisplay: Can't call \"get_map_data\" service. "
					  "Tip: if rtabmap node is not in rtabmap namespace, you can remap the service "
					  "to \"get_map_data\" in the launch "
					  "file like: <remap from=\"get_map_data\" to=\"/rtabmap/get_map_data\"/>.");
			RVIZ_COMMON_LOG_ERROR(msg);
			messageBox->setText(msg.c_str());
		}

		download_map_->blockSignals(true);
		download_map_->setBool(false);
		download_map_->blockSignals(false);
	}
	else
	{
		// just stay true if double-clicked on DownloadMap property, let the
		// first process above finishes
		download_map_->blockSignals(true);
		download_map_->setBool(true);
		download_map_->blockSignals(false);
	}
}

void MapCloudDisplay::downloadGraph()
{
	if(download_graph_->getBool())
	{
		QMessageBox * messageBox = new QMessageBox(
				QMessageBox::NoIcon,
				tr("Calling \"%1\" service...").arg("get_map_data"),
				tr("Downloading the map... please wait (rviz could become gray!)"),
				QMessageBox::NoButton);
		messageBox->setAttribute(Qt::WA_DeleteOnClose, true);
		messageBox->show();
		QApplication::processEvents();
		uSleep(100); // hack make sure the text in the QMessageBox is shown...
		QApplication::processEvents();

		auto node = rclcpp::Node::make_shared(rviz_ros_node_.lock()->get_node_name());
		 auto client = node->create_client<rtabmap_ros::srv::GetMap>("get_map_data");
		if(client->wait_for_service(std::chrono::seconds(2)))
		{
			auto request = std::make_shared<rtabmap_ros::srv::GetMap::Request>();
			request->global = true;
			request->optimized = true;
			request->graph_only = true;
			auto result_future = client->async_send_request(request);
			if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
			{
				RCLCPP_ERROR(node->get_logger(), "Service \"get_map_data\" failed to get the data.");
			}
			else
			{
				auto result = result_future.get();

				messageBox->setText(tr("Updating the map (%1 nodes downloaded)...").arg(result->data.graph.poses.size()));
				QApplication::processEvents();
				processMapData(result->data);
				messageBox->setText(tr("Updating the map (%1 nodes downloaded)... done!").arg(result->data.graph.poses.size()));

				QTimer::singleShot(1000, messageBox, SLOT(close()));
			}
		}
		else
		{
			std::string msg = uFormat("MapCloudDisplay: Can't call \"get_map_data\" service. "
					  "Tip: if rtabmap node is not in rtabmap namespace, you can remap the service "
					  "to \"get_map_data\" in the launch "
					  "file like: <remap from=\"get_map_data\" to=\"rtabmap/get_map_data\"/>");
			RVIZ_COMMON_LOG_ERROR(msg);
			messageBox->setText(msg.c_str());
		}

		download_graph_->blockSignals(true);
		download_graph_->setBool(false);
		download_graph_->blockSignals(false);
	}
	else
	{
		// just stay true if double-clicked on DownloadGraph property, let the
		// first process above finishes
		download_graph_->blockSignals(true);
		download_graph_->setBool(true);
		download_graph_->blockSignals(false);
	}
}

void MapCloudDisplay::causeRetransform()
{
  needs_retransform_ = true;
}

void MapCloudDisplay::update( float, float )
{
	auto mode = static_cast<rviz_rendering::PointCloud::RenderMode>(style_property_->getOptionInt());

	if (needs_retransform_)
	{
		retransform();
		needs_retransform_ = false;
	}

	{
		std::unique_lock<std::mutex> lock(new_clouds_mutex_);
		if( !new_cloud_infos_.empty() )
		{
			float size;
			  if (mode == rviz_rendering::PointCloud::RM_POINTS) {
			    size = point_pixel_size_property_->getFloat();
			  } else {
			    size = point_world_size_property_->getFloat();
			  }

			auto it = new_cloud_infos_.begin();
			auto end = new_cloud_infos_.end();
			for (; it != end; ++it)
			{
				CloudInfoPtr cloud_info = it->second;

				bool per_point_alpha = rviz_default_plugins::findChannelIndex(cloud_info->message_, "rgba") != -1;

				cloud_info->cloud_.reset( new rviz_rendering::PointCloud() );
				cloud_info->cloud_->addPoints(cloud_info->transformed_points_.begin(), cloud_info->transformed_points_.end());
				cloud_info->cloud_->setRenderMode( mode );
				cloud_info->cloud_->setAlpha( alpha_property_->getFloat(), per_point_alpha);
				cloud_info->cloud_->setDimensions( size, size, size );
				cloud_info->cloud_->setAutoSize(auto_size_);

				cloud_info->manager_ = context_->getSceneManager();

				cloud_info->scene_node_ = scene_node_->createChildSceneNode();

				cloud_info->scene_node_->attachObject( cloud_info->cloud_.get() );
				cloud_info->scene_node_->setVisible(false);

				cloud_infos_.erase(it->first);
				cloud_infos_.insert(*it);
			}

			new_cloud_infos_.clear();
		}
	}

	{
		std::unique_lock<std::recursive_mutex> lock( transformers_mutex_ );

		if( new_xyz_transformer_ || new_color_transformer_ )
		{
			for (auto transformer : transformers_) {
			      const std::string & name = transformer.first;
			      TransformerInfo & info = transformer.second;

			      setPropertiesHidden(info.xyz_props, name != xyz_transformer_property_->getStdString());
			      setPropertiesHidden(info.color_props,
			        name != color_transformer_property_->getStdString());

			      if (name == xyz_transformer_property_->getStdString() ||
			        name == color_transformer_property_->getStdString())
			      {
			        info.transformer->hideUnusedProperties();
			      }
			    }
		}

		new_xyz_transformer_ = false;
		new_color_transformer_ = false;
	}

	int totalPoints = 0;
	int totalNodesShown = 0;
	{
		// update poses
		std::unique_lock<std::mutex> lock(current_map_mutex_);
		if(!current_map_.empty())
		{
			for (std::map<int, rtabmap::Transform>::iterator it=current_map_.begin(); it != current_map_.end(); ++it)
			{
				std::map<int, CloudInfoPtr>::iterator cloudInfoIt = cloud_infos_.find(it->first);
				if(cloudInfoIt != cloud_infos_.end())
				{
					totalPoints += cloudInfoIt->second->transformed_points_.size();
					cloudInfoIt->second->pose_ = it->second;
					Ogre::Vector3 framePosition;
					Ogre::Quaternion frameOrientation;
					if (context_->getFrameManager()->getTransform(cloudInfoIt->second->message_->header.frame_id, framePosition, frameOrientation))
					{
						// Multiply frame with pose
						Ogre::Matrix4 frameTransform;
						frameTransform.makeTransform( framePosition, Ogre::Vector3(1,1,1), frameOrientation);
						const rtabmap::Transform & p = cloudInfoIt->second->pose_;
						Ogre::Matrix4 pose(p[0], p[1], p[2], p[3],
										 p[4], p[5], p[6], p[7],
										 p[8], p[9], p[10], p[11],
										 0, 0, 0, 1);
						frameTransform = frameTransform * pose;
						Ogre::Vector3 posePosition = frameTransform.getTrans();
						Ogre::Quaternion poseOrientation = frameTransform.extractQuaternion();
						poseOrientation.normalise();

						cloudInfoIt->second->scene_node_->setPosition(posePosition);
						cloudInfoIt->second->scene_node_->setOrientation(poseOrientation);
						cloudInfoIt->second->scene_node_->setVisible(true);
						++totalNodesShown;
					}
					else
					{
						RVIZ_COMMON_LOG_ERROR(uFormat("MapCloudDisplay: Could not update pose of node %d", it->first));
					}

				}
			}
			//hide not used clouds
			for(std::map<int, CloudInfoPtr>::iterator iter = cloud_infos_.begin(); iter!=cloud_infos_.end(); ++iter)
			{
				if(current_map_.find(iter->first) == current_map_.end())
				{
					iter->second->scene_node_->setVisible(false);
				}
			}
		}
	}

	this->setStatusStd(rviz_common::properties::StatusProperty::Ok, "Points", tr("%1").arg(totalPoints).toStdString());
	this->setStatusStd(rviz_common::properties::StatusProperty::Ok, "Nodes", tr("%1 shown of %2").arg(totalNodesShown).arg(cloud_infos_.size()).toStdString());
}

void MapCloudDisplay::reset()
{
	{
		std::unique_lock<std::mutex> lock(new_clouds_mutex_);
		cloud_infos_.clear();
		new_cloud_infos_.clear();
	}
	{
		std::unique_lock<std::mutex> lock(current_map_mutex_);
		current_map_.clear();
	}
}

void MapCloudDisplay::updateXyzTransformer()
{
	std::unique_lock<std::recursive_mutex> lock( transformers_mutex_ );
	if( transformers_.count( xyz_transformer_property_->getStdString() ) == 0 )
	{
		return;
	}
	new_xyz_transformer_ = true;
	causeRetransform();
}

void MapCloudDisplay::updateColorTransformer()
{
	std::unique_lock<std::recursive_mutex> lock( transformers_mutex_ );
	if( transformers_.count( color_transformer_property_->getStdString() ) == 0 )
	{
		return;
	}
	new_color_transformer_ = true;
	causeRetransform();
}

void MapCloudDisplay::setXyzTransformerOptions( rviz_common::properties::EnumProperty* prop )
{
	fillTransformerOptions( prop, rviz_default_plugins::PointCloudTransformer::Support_XYZ );
}

void MapCloudDisplay::setColorTransformerOptions( rviz_common::properties::EnumProperty* prop )
{
	fillTransformerOptions( prop, rviz_default_plugins::PointCloudTransformer::Support_Color );
}

void MapCloudDisplay::fillTransformerOptions(
  rviz_common::properties::EnumProperty * prop,
  uint32_t mask)
{
  prop->clearOptions();

  if (cloud_infos_.empty()) {
    return;
  }

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg = cloud_infos_.begin()->second->message_;

  for (auto transformer : transformers_) {
    const std::shared_ptr<rviz_default_plugins::PointCloudTransformer> & trans = transformer.second.transformer;
    if ((trans->supports(msg) & mask) == mask) {
      prop->addOption(QString::fromStdString(transformer.first));
    }
  }
}

std::shared_ptr<rviz_default_plugins::PointCloudTransformer> MapCloudDisplay::getXYZTransformer(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(xyz_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const std::shared_ptr<rviz_default_plugins::PointCloudTransformer> & trans = it->second.transformer;
    if (trans->supports(cloud) & rviz_default_plugins::PointCloudTransformer::Support_XYZ) {
      return trans;
    }
  }

  return std::shared_ptr<rviz_default_plugins::PointCloudTransformer>();
}

std::shared_ptr<rviz_default_plugins::PointCloudTransformer> MapCloudDisplay::getColorTransformer(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(color_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const std::shared_ptr<rviz_default_plugins::PointCloudTransformer> & trans = it->second.transformer;
    if (trans->supports(cloud) & rviz_default_plugins::PointCloudTransformer::Support_Color) {
      return trans;
    }
  }

  return std::shared_ptr<rviz_default_plugins::PointCloudTransformer>();
}


void MapCloudDisplay::retransform()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  for (auto const & cloud_info : cloud_infos_) {
    transformCloud(cloud_info.second, false);
    cloud_info.second->cloud_->clear();
    cloud_info.second->cloud_->addPoints(
      cloud_info.second->transformed_points_.begin(), cloud_info.second->transformed_points_.end());
  }
}

bool MapCloudDisplay::transformCloud(const CloudInfoPtr& cloud_info, bool update_transformers)
{
	this->deleteStatusStd(message_status_name_);

	rviz_default_plugins::V_PointCloudPoint& cloud_points = cloud_info->transformed_points_;
	cloud_points.clear();

	size_t size = cloud_info->message_->width * cloud_info->message_->height;
	rviz_rendering::PointCloud::Point default_pt;
	default_pt.color = Ogre::ColourValue(1, 1, 1);
	default_pt.position = Ogre::Vector3::ZERO;
	cloud_points.resize(size, default_pt);

	{
		std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
		if( update_transformers )
		{
			updateTransformers( cloud_info->message_ );
		}
		std::shared_ptr<rviz_default_plugins::PointCloudTransformer> xyz_trans = getXYZTransformer(cloud_info->message_);
		std::shared_ptr<rviz_default_plugins::PointCloudTransformer> color_trans = getColorTransformer(cloud_info->message_);

		if (cloud_info->message_->data.size() !=
		    cloud_info->message_->width * cloud_info->message_->height * cloud_info->message_->point_step)
		  {
		    std::string status = "PointCloud contained not enough or too much data";
		    this->setStatusStd(
		      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
		    return false;
		  }

		if (!xyz_trans)
		{
			std::stringstream ss;
			ss << "No position transformer available for cloud";
			this->setStatusStd(rviz_common::properties::StatusProperty::Error, message_status_name_, ss.str());
			return false;
		}

		if (!color_trans)
		{
			std::stringstream ss;
			ss << "No color transformer available for cloud";
			this->setStatusStd(rviz_common::properties::StatusProperty::Error, message_status_name_, ss.str());
			return false;
		}

		xyz_trans->transform(cloud_info->message_, rviz_default_plugins::PointCloudTransformer::Support_XYZ, Ogre::Matrix4::IDENTITY, cloud_points);
		color_trans->transform(cloud_info->message_, rviz_default_plugins::PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, cloud_points);
	}

	for (auto & cloud_point : cloud_points) {
	    if (!rviz_common::validateFloats(cloud_point.position)) {
	      cloud_point.position.x = 999999.0f;
	      cloud_point.position.y = 999999.0f;
	      cloud_point.position.z = 999999.0f;
	    }
	  }

	return true;
}

} // namespace rtabmap

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rtabmap_ros::MapCloudDisplay, rviz_common::Display)
