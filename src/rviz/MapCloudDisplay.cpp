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

#include <QApplication>
#include <QMessageBox>
#include <QTimer>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>

#include <tf/transform_listener.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/validate_floats.h>
#include <rviz/properties/int_property.h>
#include "rviz/properties/bool_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"

#include <pcl_conversions/pcl_conversions.h>

#include "MapCloudDisplay.h"
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/GetMap.h>


namespace rtabmap_ros
{


MapCloudDisplay::CloudInfo::CloudInfo() :
		manager_(0),
		pose_(rtabmap::Transform::getIdentity()),
		id_(0),
		scene_node_(0)
{}

MapCloudDisplay::CloudInfo::~CloudInfo()
{
	clear();
}

void MapCloudDisplay::CloudInfo::clear()
{
	if ( scene_node_ )
	{
		manager_->destroySceneNode( scene_node_ );
		scene_node_=0;
	}
}

MapCloudDisplay::MapCloudDisplay()
  : spinner_(1, &cbqueue_),
    new_xyz_transformer_(false),
    new_color_transformer_(false),
    needs_retransform_(false),
    transformer_class_loader_(NULL)
{
	//QIcon icon;
	//this->setIcon(icon);

	style_property_ = new rviz::EnumProperty( "Style", "Flat Squares",
										"Rendering mode to use, in order of computational complexity.",
										this, SLOT( updateStyle() ), this );
	style_property_->addOption( "Points", rviz::PointCloud::RM_POINTS );
	style_property_->addOption( "Squares", rviz::PointCloud::RM_SQUARES );
	style_property_->addOption( "Flat Squares", rviz::PointCloud::RM_FLAT_SQUARES );
	style_property_->addOption( "Spheres", rviz::PointCloud::RM_SPHERES );
	style_property_->addOption( "Boxes", rviz::PointCloud::RM_BOXES );

	point_world_size_property_ = new rviz::FloatProperty( "Size (m)", 0.01,
												  "Point size in meters.",
												  this, SLOT( updateBillboardSize() ), this );
	point_world_size_property_->setMin( 0.0001 );

	point_pixel_size_property_ = new rviz::FloatProperty( "Size (Pixels)", 3,
												  "Point size in pixels.",
												  this, SLOT( updateBillboardSize() ), this );
	point_pixel_size_property_->setMin( 1 );

	alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
										 "Amount of transparency to apply to the points.  Note that this is experimental and does not always look correct.",
										 this, SLOT( updateAlpha() ), this );
	alpha_property_->setMin( 0 );
	alpha_property_->setMax( 1 );

	xyz_transformer_property_ = new rviz::EnumProperty( "Position Transformer", "",
												  "Set the transformer to use to set the position of the points.",
												  this, SLOT( updateXyzTransformer() ), this );
	connect( xyz_transformer_property_, SIGNAL( requestOptions( EnumProperty* )),
			 this, SLOT( setXyzTransformerOptions( EnumProperty* )));

	color_transformer_property_ = new rviz::EnumProperty( "Color Transformer", "",
													"Set the transformer to use to set the color of the points.",
													this, SLOT( updateColorTransformer() ), this );
	connect( color_transformer_property_, SIGNAL( requestOptions( EnumProperty* )),
			 this, SLOT( setColorTransformerOptions( EnumProperty* )));

	cloud_decimation_ = new rviz::IntProperty( "Cloud decimation", 4,
										 "Decimation of the input RGB and depth images before creating the cloud.",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_decimation_->setMin( 1 );
	cloud_decimation_->setMax( 16 );

	cloud_max_depth_ = new rviz::FloatProperty( "Cloud max depth (m)", 4.0f,
										 "Maximum depth of the generated clouds.",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_max_depth_->setMin( 0.0f );
	cloud_max_depth_->setMax( 999.0f );

	cloud_min_depth_ = new rviz::FloatProperty( "Cloud min depth (m)", 0.0f,
											 "Minimum depth of the generated clouds.",
											 this, SLOT( updateCloudParameters() ), this );
	cloud_min_depth_->setMin( 0.0f );
	cloud_min_depth_->setMax( 999.0f );

	cloud_voxel_size_ = new rviz::FloatProperty( "Cloud voxel size (m)", 0.01f,
										 "Voxel size of the generated clouds.",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_voxel_size_->setMin( 0.0f );
	cloud_voxel_size_->setMax( 1.0f );

	cloud_filter_floor_height_ = new rviz::FloatProperty( "Filter floor (m)", 0.0f,
										 "Filter the floor up to maximum height set here "
										 "(only appropriate for 2D mapping).",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_filter_floor_height_->setMin( 0.0f );
	cloud_filter_floor_height_->setMax( 999.0f );

	cloud_filter_ceiling_height_ = new rviz::FloatProperty( "Filter ceiling (m)", 0.0f,
										 "Filter the ceiling at the specified height set here "
										 "(only appropriate for 2D mapping).",
										 this, SLOT( updateCloudParameters() ), this );
	cloud_filter_ceiling_height_->setMin( 0.0f );
	cloud_filter_ceiling_height_->setMax( 999.0f );

	node_filtering_radius_ = new rviz::FloatProperty( "Node filtering radius (m)", 0.0f,
										 "(Disabled=0) Only keep one node in the specified radius.",
										 this, SLOT( updateCloudParameters() ), this );
	node_filtering_radius_->setMin( 0.0f );
	node_filtering_radius_->setMax( 10.0f );

	node_filtering_angle_ = new rviz::FloatProperty( "Node filtering angle (degrees)", 30.0f,
										 "(Disabled=0) Only keep one node in the specified angle in the filtering radius.",
										 this, SLOT( updateCloudParameters() ), this );
	node_filtering_angle_->setMin( 0.0f );
	node_filtering_angle_->setMax( 359.0f );

	download_map_ = new rviz::BoolProperty( "Download map", false,
										 "Download the optimized global map using rtabmap/GetMap service. This will force to re-create all clouds.",
										 this, SLOT( downloadMap() ), this );

	download_graph_ = new rviz::BoolProperty( "Download graph", false,
											 "Download the optimized global graph (without cloud data) using rtabmap/GetMap service.",
											 this, SLOT( downloadGraph() ), this );

	// PointCloudCommon sets up a callback queue with a thread for each
	// instance.  Use that for processing incoming messages.
	update_nh_.setCallbackQueue( &cbqueue_ );
}

MapCloudDisplay::~MapCloudDisplay()
{
	if ( transformer_class_loader_ )
	{
		delete transformer_class_loader_;
	}

	spinner_.stop();
}

void MapCloudDisplay::loadTransformers()
{
	std::vector<std::string> classes = transformer_class_loader_->getDeclaredClasses();
	std::vector<std::string>::iterator ci;

	for( ci = classes.begin(); ci != classes.end(); ci++ )
	{
		const std::string& lookup_name = *ci;
		std::string name = transformer_class_loader_->getName( lookup_name );

		if( transformers_.count( name ) > 0 )
		{
			ROS_ERROR( "Transformer type [%s] is already loaded.", name.c_str() );
			continue;
		}

		rviz::PointCloudTransformerPtr trans( transformer_class_loader_->createUnmanagedInstance( lookup_name ));
		trans->init();
		connect( trans.get(), SIGNAL( needRetransform() ), this, SLOT( causeRetransform() ));

		TransformerInfo info;
		info.transformer = trans;
		info.readable_name = name;
		info.lookup_name = lookup_name;

		info.transformer->createProperties( this, rviz::PointCloudTransformer::Support_XYZ, info.xyz_props );
		setPropertiesHidden( info.xyz_props, true );

		info.transformer->createProperties( this, rviz::PointCloudTransformer::Support_Color, info.color_props );
		setPropertiesHidden( info.color_props, true );

		transformers_[ name ] = info;
	}
}

void MapCloudDisplay::onInitialize()
{
	MFDClass::onInitialize();

	transformer_class_loader_ = new pluginlib::ClassLoader<rviz::PointCloudTransformer>( "rviz", "rviz::PointCloudTransformer" );
	loadTransformers();

	updateStyle();
	updateBillboardSize();
	updateAlpha();

	spinner_.start();
}

void MapCloudDisplay::processMessage( const rtabmap_ros::MapDataConstPtr& msg )
{
	processMapData(*msg);

	this->emitTimeSignal(msg->header.stamp);
}

void MapCloudDisplay::processMapData(const rtabmap_ros::MapData& map)
{
	std::map<int, rtabmap::Transform> poses;
	for(unsigned int i=0; i<map.graph.posesId.size() && i<map.graph.poses.size(); ++i)
	{
		poses.insert(std::make_pair(map.graph.posesId[i], rtabmap_ros::transformFromPoseMsg(map.graph.poses[i])));
	}

	// Add new clouds...
	for(unsigned int i=0; i<map.nodes.size() && i<map.nodes.size(); ++i)
	{
		int id = map.nodes[i].id;

		// Always refresh the cloud if there are data
		rtabmap::Signature s = rtabmap_ros::nodeDataFromROS(map.nodes[i]);
		if(!s.sensorData().imageCompressed().empty() &&
		   !s.sensorData().depthOrRightCompressed().empty() &&
		   (s.sensorData().cameraModels().size() || s.sensorData().stereoCameraModel().isValidForProjection()))
		{
			cv::Mat image, depth;
			s.sensorData().uncompressData(&image, &depth, 0);


			if(!s.sensorData().imageRaw().empty() && !s.sensorData().depthOrRightRaw().empty())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
				pcl::IndicesPtr validIndices(new std::vector<int>);
				cloud = rtabmap::util3d::cloudRGBFromSensorData(
						s.sensorData(),
						cloud_decimation_->getInt(),
						cloud_max_depth_->getFloat(),
						cloud_min_depth_->getFloat(),
						validIndices.get());

				if(cloud_voxel_size_->getFloat())
				{
					cloud = rtabmap::util3d::voxelize(cloud, validIndices, cloud_voxel_size_->getFloat());
				}

				if(cloud->size())
				{
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

					sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
					pcl::toROSMsg(*cloud, *cloudMsg);
					cloudMsg->header = map.header;

					CloudInfoPtr info(new CloudInfo);
					info->message_ = cloudMsg;
					info->pose_ = rtabmap::Transform::getIdentity();
					info->id_ = id;

					if (transformCloud(info, true))
					{
						boost::mutex::scoped_lock lock(new_clouds_mutex_);
						new_cloud_infos_.erase(id);
						new_cloud_infos_.insert(std::make_pair(id, info));
					}
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
		boost::mutex::scoped_lock lock(current_map_mutex_);
		current_map_ = poses;
	}
}

void MapCloudDisplay::setPropertiesHidden( const QList<Property*>& props, bool hide )
{
	for( int i = 0; i < props.size(); i++ )
	{
		props[ i ]->setHidden( hide );
	}
}

void MapCloudDisplay::updateTransformers( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
	std::string xyz_name = xyz_transformer_property_->getStdString();
	std::string color_name = color_transformer_property_->getStdString();

	xyz_transformer_property_->clearOptions();
	color_transformer_property_->clearOptions();

	// Get the channels that we could potentially render
	typedef std::set<std::pair<uint8_t, std::string> > S_string;
	S_string valid_xyz, valid_color;
	bool cur_xyz_valid = false;
	bool cur_color_valid = false;
	bool has_rgb_transformer = false;
	M_TransformerInfo::iterator trans_it = transformers_.begin();
	M_TransformerInfo::iterator trans_end = transformers_.end();
	for(;trans_it != trans_end; ++trans_it)
	{
		const std::string& name = trans_it->first;
		const rviz::PointCloudTransformerPtr& trans = trans_it->second.transformer;
		uint32_t mask = trans->supports(cloud);
		if (mask & rviz::PointCloudTransformer::Support_XYZ)
		{
			valid_xyz.insert(std::make_pair(trans->score(cloud), name));
			if (name == xyz_name)
			{
				cur_xyz_valid = true;
			}
			xyz_transformer_property_->addOptionStd( name );
		}

		if (mask & rviz::PointCloudTransformer::Support_Color)
		{
			valid_color.insert(std::make_pair(trans->score(cloud), name));
			if (name == color_name)
			{
				cur_color_valid = true;
			}
			if (name == "RGB8")
			{
				has_rgb_transformer = true;
			}
			color_transformer_property_->addOptionStd( name );
		}
	}

	if( !cur_xyz_valid )
	{
		if( !valid_xyz.empty() )
		{
			xyz_transformer_property_->setStringStd( valid_xyz.rbegin()->second );
		}
	}

	if( !cur_color_valid )
	{
		if( !valid_color.empty() )
		{
			if (has_rgb_transformer)
			{
				color_transformer_property_->setStringStd( "RGB8" );
			}
			else
			{
				color_transformer_property_->setStringStd( valid_color.rbegin()->second );
			}
		}
	}
}

void MapCloudDisplay::updateAlpha()
{
	for( std::map<int, CloudInfoPtr>::iterator it = cloud_infos_.begin(); it != cloud_infos_.end(); ++it )
	{
		it->second->cloud_->setAlpha( alpha_property_->getFloat() );
	}
}

void MapCloudDisplay::updateStyle()
{
	rviz::PointCloud::RenderMode mode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();
	if( mode == rviz::PointCloud::RM_POINTS )
	{
		point_world_size_property_->hide();
		point_pixel_size_property_->show();
	}
	else
	{
		point_world_size_property_->show();
		point_pixel_size_property_->hide();
	}
	for( std::map<int, CloudInfoPtr>::iterator it = cloud_infos_.begin(); it != cloud_infos_.end(); ++it )
	{
		it->second->cloud_->setRenderMode( mode );
	}
	updateBillboardSize();
}

void MapCloudDisplay::updateBillboardSize()
{
	rviz::PointCloud::RenderMode mode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();
	float size;
	if( mode == rviz::PointCloud::RM_POINTS )
	{
		size = point_pixel_size_property_->getFloat();
	}
	else
	{
		size = point_world_size_property_->getFloat();
	}
	 for( std::map<int, CloudInfoPtr>::iterator it = cloud_infos_.begin(); it != cloud_infos_.end(); ++it )
	{
		it->second->cloud_->setDimensions( size, size, size );
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
		rtabmap_ros::GetMap getMapSrv;
		getMapSrv.request.global = true;
		getMapSrv.request.optimized = true;
		getMapSrv.request.graphOnly = false;
		ros::NodeHandle nh;
		QMessageBox * messageBox = new QMessageBox(
				QMessageBox::NoIcon,
				tr("Calling \"%1\" service...").arg(nh.resolveName("rtabmap/get_map_data").c_str()),
				tr("Downloading the map... please wait (rviz could become gray!)"),
				QMessageBox::NoButton);
		messageBox->setAttribute(Qt::WA_DeleteOnClose, true);
		messageBox->show();
		QApplication::processEvents();
		uSleep(100); // hack make sure the text in the QMessageBox is shown...
		QApplication::processEvents();
		if(!ros::service::call("rtabmap/get_map_data", getMapSrv))
		{
			ROS_ERROR("MapCloudDisplay: Can't call \"%s\" service. "
					  "Tip: if rtabmap node is not in rtabmap namespace, you can remap the service "
					  "to \"get_map_data\" in the launch "
					  "file like: <remap from=\"rtabmap/get_map_data\" to=\"get_map_data\"/>.",
					  nh.resolveName("rtabmap/get_map_data").c_str());
			messageBox->setText(tr("MapCloudDisplay: Can't call \"%1\" service. "
					  "Tip: if rtabmap node is not in rtabmap namespace, you can remap the service "
					  "to \"get_map_data\" in the launch "
					  "file like: <remap from=\"rtabmap/get_map_data\" to=\"get_map_data\"/>.").
					  arg(nh.resolveName("rtabmap/get_map_data").c_str()));
		}
		else
		{
			messageBox->setText(tr("Creating all clouds (%1 poses and %2 clouds downloaded)...")
					.arg(getMapSrv.response.data.graph.poses.size()).arg(getMapSrv.response.data.nodes.size()));
			QApplication::processEvents();
			this->reset();
			processMapData(getMapSrv.response.data);
			messageBox->setText(tr("Creating all clouds (%1 poses and %2 clouds downloaded)... done!")
					.arg(getMapSrv.response.data.graph.poses.size()).arg(getMapSrv.response.data.nodes.size()));

			QTimer::singleShot(1000, messageBox, SLOT(close()));
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
		rtabmap_ros::GetMap getMapSrv;
		getMapSrv.request.global = true;
		getMapSrv.request.optimized = true;
		getMapSrv.request.graphOnly = true;
		ros::NodeHandle nh;
		QMessageBox * messageBox = new QMessageBox(
				QMessageBox::NoIcon,
				tr("Calling \"%1\" service...").arg(nh.resolveName("rtabmap/get_map_data").c_str()),
				tr("Downloading the graph... please wait (rviz could become gray!)"),
				QMessageBox::NoButton);
		messageBox->setAttribute(Qt::WA_DeleteOnClose, true);
		messageBox->show();
		QApplication::processEvents();
		uSleep(100); // hack make sure the text in the QMessageBox is shown...
		QApplication::processEvents();
		if(!ros::service::call("rtabmap/get_map_data", getMapSrv))
		{
			ROS_ERROR("MapCloudDisplay: Can't call \"%s\" service. "
					  "Tip: if rtabmap node is not in rtabmap namespace, you can remap the service "
					  "to \"get_map_data\" in the launch "
					  "file like: <remap from=\"rtabmap/get_map_data\" to=\"get_map_data\"/>.",
					  nh.resolveName("rtabmap/get_map_data").c_str());
			messageBox->setText(tr("MapCloudDisplay: Can't call \"%1\" service. "
					  "Tip: if rtabmap node is not in rtabmap namespace, you can remap the service "
					  "to \"get_map_data\" in the launch "
					  "file like: <remap from=\"rtabmap/get_map_data\" to=\"get_map_data\"/>.").
					  arg(nh.resolveName("rtabmap/get_map_data").c_str()));
		}
		else
		{
			messageBox->setText(tr("Updating the map (%1 nodes downloaded)...").arg(getMapSrv.response.data.graph.poses.size()));
			QApplication::processEvents();
			processMapData(getMapSrv.response.data);
			messageBox->setText(tr("Updating the map (%1 nodes downloaded)... done!").arg(getMapSrv.response.data.graph.poses.size()));

			QTimer::singleShot(1000, messageBox, SLOT(close()));
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

void MapCloudDisplay::update( float wall_dt, float ros_dt )
{
	rviz::PointCloud::RenderMode mode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();

	if (needs_retransform_)
	{
		retransform();
		needs_retransform_ = false;
	}

	{
		boost::mutex::scoped_lock lock(new_clouds_mutex_);
		if( !new_cloud_infos_.empty() )
		{
			float size;
			if( mode == rviz::PointCloud::RM_POINTS ) {
				size = point_pixel_size_property_->getFloat();
			} else {
				size = point_world_size_property_->getFloat();
			}

			std::map<int, CloudInfoPtr>::iterator it = new_cloud_infos_.begin();
			std::map<int, CloudInfoPtr>::iterator end = new_cloud_infos_.end();
			for (; it != end; ++it)
			{
				CloudInfoPtr cloud_info = it->second;

				cloud_info->cloud_.reset( new rviz::PointCloud() );
				cloud_info->cloud_->addPoints( &(cloud_info->transformed_points_.front()), cloud_info->transformed_points_.size() );
				cloud_info->cloud_->setRenderMode( mode );
				cloud_info->cloud_->setAlpha( alpha_property_->getFloat() );
				cloud_info->cloud_->setDimensions( size, size, size );
				cloud_info->cloud_->setAutoSize(false);

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
		boost::recursive_mutex::scoped_try_lock lock( transformers_mutex_ );

		if( lock.owns_lock() )
		{
			if( new_xyz_transformer_ || new_color_transformer_ )
			{
				M_TransformerInfo::iterator it = transformers_.begin();
				M_TransformerInfo::iterator end = transformers_.end();
				for (; it != end; ++it)
				{
					const std::string& name = it->first;
					TransformerInfo& info = it->second;

					setPropertiesHidden( info.xyz_props, name != xyz_transformer_property_->getStdString() );
					setPropertiesHidden( info.color_props, name != color_transformer_property_->getStdString() );
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
		boost::mutex::scoped_lock lock(current_map_mutex_);
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
					if (context_->getFrameManager()->getTransform(cloudInfoIt->second->message_->header, framePosition, frameOrientation))
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
						ROS_ERROR("MapCloudDisplay: Could not update pose of node %d", it->first);
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

	this->setStatusStd(rviz::StatusProperty::Ok, "Points", tr("%1").arg(totalPoints).toStdString());
	this->setStatusStd(rviz::StatusProperty::Ok, "Nodes", tr("%1 shown of %2").arg(totalNodesShown).arg(cloud_infos_.size()).toStdString());
}

void MapCloudDisplay::reset()
{
	{
		boost::mutex::scoped_lock lock(new_clouds_mutex_);
		cloud_infos_.clear();
		new_cloud_infos_.clear();
	}
	{
		boost::mutex::scoped_lock lock(current_map_mutex_);
		current_map_.clear();
	}
	MFDClass::reset();
}

void MapCloudDisplay::updateXyzTransformer()
{
	boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
	if( transformers_.count( xyz_transformer_property_->getStdString() ) == 0 )
	{
		return;
	}
	new_xyz_transformer_ = true;
	causeRetransform();
}

void MapCloudDisplay::updateColorTransformer()
{
	boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
	if( transformers_.count( color_transformer_property_->getStdString() ) == 0 )
	{
		return;
	}
	new_color_transformer_ = true;
	causeRetransform();
}

void MapCloudDisplay::setXyzTransformerOptions( EnumProperty* prop )
{
	fillTransformerOptions( prop, rviz::PointCloudTransformer::Support_XYZ );
}

void MapCloudDisplay::setColorTransformerOptions( EnumProperty* prop )
{
	fillTransformerOptions( prop, rviz::PointCloudTransformer::Support_Color );
}

void MapCloudDisplay::fillTransformerOptions( rviz::EnumProperty* prop, uint32_t mask )
{
	prop->clearOptions();

	if (cloud_infos_.empty())
	{
		return;
	}

	boost::recursive_mutex::scoped_lock tlock(transformers_mutex_);

	const sensor_msgs::PointCloud2ConstPtr& msg = cloud_infos_.begin()->second->message_;

	M_TransformerInfo::iterator it = transformers_.begin();
	M_TransformerInfo::iterator end = transformers_.end();
	for (; it != end; ++it)
	{
		const rviz::PointCloudTransformerPtr& trans = it->second.transformer;
		if ((trans->supports(msg) & mask) == mask)
		{
			prop->addOption( QString::fromStdString( it->first ));
		}
	}
}

rviz::PointCloudTransformerPtr MapCloudDisplay::getXYZTransformer( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
	boost::recursive_mutex::scoped_lock lock( transformers_mutex_);
	M_TransformerInfo::iterator it = transformers_.find( xyz_transformer_property_->getStdString() );
	if( it != transformers_.end() )
	{
		const rviz::PointCloudTransformerPtr& trans = it->second.transformer;
		if( trans->supports( cloud ) & rviz::PointCloudTransformer::Support_XYZ )
		{
			return trans;
		}
	}

	return rviz::PointCloudTransformerPtr();
}

rviz::PointCloudTransformerPtr MapCloudDisplay::getColorTransformer( const sensor_msgs::PointCloud2ConstPtr& cloud )
{
	boost::recursive_mutex::scoped_lock lock( transformers_mutex_ );
	M_TransformerInfo::iterator it = transformers_.find( color_transformer_property_->getStdString() );
	if( it != transformers_.end() )
	{
		const rviz::PointCloudTransformerPtr& trans = it->second.transformer;
		if( trans->supports( cloud ) & rviz::PointCloudTransformer::Support_Color )
		{
			return trans;
		}
	}

	return rviz::PointCloudTransformerPtr();
}


void MapCloudDisplay::retransform()
{
	boost::recursive_mutex::scoped_lock lock(transformers_mutex_);

	for( std::map<int, CloudInfoPtr>::iterator it = cloud_infos_.begin(); it != cloud_infos_.end(); ++it )
	{
		const CloudInfoPtr& cloud_info = it->second;
		transformCloud(cloud_info, false);
		cloud_info->cloud_->clear();
		cloud_info->cloud_->addPoints(&cloud_info->transformed_points_.front(), cloud_info->transformed_points_.size());
	}
}

bool MapCloudDisplay::transformCloud(const CloudInfoPtr& cloud_info, bool update_transformers)
{
	rviz::V_PointCloudPoint& cloud_points = cloud_info->transformed_points_;
	cloud_points.clear();

	size_t size = cloud_info->message_->width * cloud_info->message_->height;
	rviz::PointCloud::Point default_pt;
	default_pt.color = Ogre::ColourValue(1, 1, 1);
	default_pt.position = Ogre::Vector3::ZERO;
	cloud_points.resize(size, default_pt);

	{
		boost::recursive_mutex::scoped_lock lock(transformers_mutex_);
		if( update_transformers )
		{
			updateTransformers( cloud_info->message_ );
		}
		rviz::PointCloudTransformerPtr xyz_trans = getXYZTransformer(cloud_info->message_);
		rviz::PointCloudTransformerPtr color_trans = getColorTransformer(cloud_info->message_);

		if (!xyz_trans)
		{
			std::stringstream ss;
			ss << "No position transformer available for cloud";
			this->setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
			return false;
		}

		if (!color_trans)
		{
			std::stringstream ss;
			ss << "No color transformer available for cloud";
			this->setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
			return false;
		}

		xyz_trans->transform(cloud_info->message_, rviz::PointCloudTransformer::Support_XYZ, Ogre::Matrix4::IDENTITY, cloud_points);
		color_trans->transform(cloud_info->message_, rviz::PointCloudTransformer::Support_Color, Ogre::Matrix4::IDENTITY, cloud_points);
	}

	for (rviz::V_PointCloudPoint::iterator cloud_point = cloud_points.begin(); cloud_point != cloud_points.end(); ++cloud_point)
	{
		if (!rviz::validateFloats(cloud_point->position))
		{
			cloud_point->position.x = 999999.0f;
			cloud_point->position.y = 999999.0f;
			cloud_point->position.z = 999999.0f;
		}
	}

	return true;
}

} // namespace rtabmap

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap_ros::MapCloudDisplay, rviz::Display )
