
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
#include <rtabmap/core/util3d.h>
#include <rtabmap/MsgConversion.h>

namespace rtabmap
{


MapCloudDisplay::CloudInfo::CloudInfo()
: manager_(0)
, scene_node_(0)
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

void MapCloudDisplay::processMessage( const rtabmap::InfoExConstPtr& msg )
{
	// Add new clouds...
	for(unsigned int i=0; i<msg->data.localTransformIDs.size() && i<msg->data.localTransforms.size(); ++i)
	{
		int id = msg->data.localTransformIDs[i];
		if(cloud_infos_.find(id) == cloud_infos_.end())
		{
			// Cloud not added to RVIZ, add it!
			rtabmap::Transform localTransform = transformFromGeometryMsg(msg->data.localTransforms[i]);
			if(!localTransform.isNull())
			{
				cv::Mat image, depth;
				float depthConstant = 0.0f;
				rtabmap::Transform pose;

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
				for(unsigned int i=0; i<msg->data.poseIDs.size() && i<msg->data.poses.size(); ++i)
				{
					if(msg->data.poseIDs[i] == id)
					{
						pose = transformFromPoseMsg(msg->data.poses[i]);
						break;
					}
				}

				if(!image.empty() && !depth.empty() && depthConstant > 0.0f && !pose.isNull())
				{
					int cloudDecimation_ = 4;
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudFromDepthRGB(image, depth, depthConstant, cloudDecimation_);

					float cloudMaxDepth_ = 4.0f;
					float cloudVoxelSize_ = 0.02f;
					if(cloudMaxDepth_ > 0)
					{
						cloud = util3d::passThrough(cloud, "z", 0, cloudMaxDepth_);
					}
					if(cloudVoxelSize_ > 0)
					{
						cloud = util3d::voxelize(cloud, cloudVoxelSize_);
					}

					cloud = util3d::transformPointCloud(cloud, localTransform);

					sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
					pcl::toROSMsg(*cloud, *cloudMsg);
					cloudMsg->header = msg->data.header;

					CloudInfoPtr info(new CloudInfo);
					info->message_ = cloudMsg;
					info->pose_ = pose;
					info->id_ = id;

					if (transformCloud(info, true))
					{
						boost::mutex::scoped_lock lock(new_clouds_mutex_);
						new_cloud_infos_.insert(std::make_pair(id, info));
					}
				}
			}
		}
	}

	// Update graph
	std::map<int, Transform> poses;
	for(unsigned int i=0; i<msg->data.poseIDs.size() && i<msg->data.poses.size(); ++i)
	{
		poses.insert(std::make_pair(msg->data.poseIDs[i], transformFromPoseMsg(msg->data.poses[i])));
	}

	{
		boost::mutex::scoped_lock lock(current_map_mutex_);
		current_map_ = poses;
	}

	this->emitTimeSignal(msg->header.stamp);
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
      } else
      {
        color_transformer_property_->setStringStd( valid_color.rbegin()->second );
      }
    }
  }
}

void MapCloudDisplay::updateAlpha()
{
  for ( unsigned i=0; i<cloud_infos_.size(); i++ )
  {
    cloud_infos_[i]->cloud_->setAlpha( alpha_property_->getFloat() );
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
  for( unsigned int i = 0; i < cloud_infos_.size(); i++ )
  {
    cloud_infos_[i]->cloud_->setRenderMode( mode );
  }
  updateBillboardSize();
}

void MapCloudDisplay::updateBillboardSize()
{
  rviz::PointCloud::RenderMode mode = (rviz::PointCloud::RenderMode) style_property_->getOptionInt();
  float size;
  if( mode == rviz::PointCloud::RM_POINTS ) {
    size = point_pixel_size_property_->getFloat();
  } else {
    size = point_world_size_property_->getFloat();
  }
  for ( unsigned i=0; i<cloud_infos_.size(); i++ )
  {
    cloud_infos_[i]->cloud_->setDimensions( size, size, size );
  }
  context_->queueRender();
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

				cloud_info->scene_node_ = scene_node_->createChildSceneNode( cloud_info->position_, cloud_info->orientation_ );

				cloud_info->scene_node_->attachObject( cloud_info->cloud_.get() );

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
					cloudInfoIt->second->pose_ = it->second;
					if (context_->getFrameManager()->getTransform(cloudInfoIt->second->message_->header, cloudInfoIt->second->position_, cloudInfoIt->second->orientation_))
					{
						// Set pose
						Ogre::Matrix4 frameTransform;
						frameTransform.makeTransform( cloudInfoIt->second->position_, Ogre::Vector3(1,1,1), cloudInfoIt->second->orientation_ );
						const rtabmap::Transform & p = cloudInfoIt->second->pose_;
						Ogre::Matrix4 pose(p[0], p[1], p[2], p[3],
										 p[4], p[5], p[6], p[7],
										 p[8], p[9], p[10], p[11],
										 0, 0, 0, 1);
						frameTransform = frameTransform * pose;
						cloudInfoIt->second->position_ = frameTransform.getTrans();
						cloudInfoIt->second->orientation_ = frameTransform.extractQuaternion();

						cloudInfoIt->second->scene_node_->setPosition(cloudInfoIt->second->position_);
						cloudInfoIt->second->scene_node_->setOrientation(cloudInfoIt->second->orientation_);
						cloudInfoIt->second->scene_node_->setVisible(true);
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

	std::stringstream ss;
	//ss << "Showing [" << total_point_count_ << "] points from [" << clouds_.size() << "] messages";
	this->setStatusStd(rviz::StatusProperty::Ok, "Points", ss.str());
}

void MapCloudDisplay::reset()
{
	MFDClass::reset();
	{
		boost::mutex::scoped_lock lock(new_clouds_mutex_);
		cloud_infos_.clear();
		new_cloud_infos_.clear();
	}
	{
		boost::mutex::scoped_lock lock(current_map_mutex_);
		current_map_.clear();
	}
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

  std::map<int, CloudInfoPtr>::iterator it = cloud_infos_.begin();
  std::map<int, CloudInfoPtr>::iterator end = cloud_infos_.end();
  for (; it != end; ++it)
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

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rtabmap::MapCloudDisplay, rviz::Display )
