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

#ifndef MAP_CLOUD_DISPLAY_H
#define MAP_CLOUD_DISPLAY_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

#include <deque>
#include <queue>
#include <vector>

#include <rtabmap_ros/MapData.h>
#include <rtabmap/core/Transform.h>

#include <pluginlib/class_loader.h>
#include <sensor_msgs/PointCloud2.h>

#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/point_cloud_transformer.h>

#endif

namespace rviz {
class IntProperty;
class BoolProperty;
class EnumProperty;
class FloatProperty;
class PointCloudTransformer;
typedef boost::shared_ptr<PointCloudTransformer> PointCloudTransformerPtr;
typedef std::vector<std::string> V_string;
}

using namespace rviz;

namespace rtabmap_ros
{

class PointCloudCommon;

/**
 * \class MapCloudDisplay
 * \brief Displays point clouds from rtabmap::MapData
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class MapCloudDisplay: public rviz::MessageFilterDisplay<rtabmap_ros::MapData>
{
Q_OBJECT
public:
	struct CloudInfo
	  {
		CloudInfo();
		~CloudInfo();

		// clear the point cloud, but keep selection handler around
		void clear();

		Ogre::SceneManager *manager_;

		sensor_msgs::PointCloud2ConstPtr message_;
		rtabmap::Transform pose_;
		int id_;

		Ogre::SceneNode *scene_node_;
		boost::shared_ptr<rviz::PointCloud> cloud_;

		std::vector<rviz::PointCloud::Point> transformed_points_;
	};
	typedef boost::shared_ptr<CloudInfo> CloudInfoPtr;

	MapCloudDisplay();
	virtual ~MapCloudDisplay();

	virtual void reset();
	virtual void update( float wall_dt, float ros_dt );

	rviz::FloatProperty* point_world_size_property_;
	rviz::FloatProperty* point_pixel_size_property_;
	rviz::FloatProperty* alpha_property_;
	rviz::EnumProperty* xyz_transformer_property_;
	rviz::EnumProperty* color_transformer_property_;
	rviz::EnumProperty* style_property_;
	rviz::IntProperty* cloud_decimation_;
	rviz::FloatProperty* cloud_max_depth_;
	rviz::FloatProperty* cloud_min_depth_;
	rviz::FloatProperty* cloud_voxel_size_;
	rviz::FloatProperty* cloud_filter_floor_height_;
	rviz::FloatProperty* cloud_filter_ceiling_height_;
	rviz::FloatProperty* node_filtering_radius_;
	rviz::FloatProperty* node_filtering_angle_;
	rviz::BoolProperty* download_map_;
	rviz::BoolProperty* download_graph_;

public Q_SLOTS:
	void causeRetransform();

private Q_SLOTS:
	void updateStyle();
	void updateBillboardSize();
	void updateAlpha();
	void updateXyzTransformer();
	void updateColorTransformer();
	void setXyzTransformerOptions( EnumProperty* prop );
	void setColorTransformerOptions( EnumProperty* prop );
	void updateCloudParameters();
	void downloadMap();
	void downloadGraph();

protected:
	/** @brief Do initialization. Overridden from MessageFilterDisplay. */
	virtual void onInitialize();

	/** @brief Process a single message.  Overridden from MessageFilterDisplay. */
	virtual void processMessage( const rtabmap_ros::MapDataConstPtr& cloud );

private:
	void processMapData(const rtabmap_ros::MapData& map);

	/**
	* \brief Transforms the cloud into the correct frame, and sets up our renderable cloud
	*/
	bool transformCloud(const CloudInfoPtr& cloud, bool fully_update_transformers);

	rviz::PointCloudTransformerPtr getXYZTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
	rviz::PointCloudTransformerPtr getColorTransformer(const sensor_msgs::PointCloud2ConstPtr& cloud);
	void updateTransformers( const sensor_msgs::PointCloud2ConstPtr& cloud );
	void retransform();

	void loadTransformers();

	void setPropertiesHidden( const QList<Property*>& props, bool hide );
	void fillTransformerOptions( rviz::EnumProperty* prop, uint32_t mask );

private:
	ros::AsyncSpinner spinner_;
	ros::CallbackQueue cbqueue_;

	std::map<int, CloudInfoPtr> cloud_infos_;

	std::map<int, CloudInfoPtr> new_cloud_infos_;
	boost::mutex new_clouds_mutex_;

	std::map<int, rtabmap::Transform> current_map_;
	boost::mutex current_map_mutex_;

	struct TransformerInfo
	{
		rviz::PointCloudTransformerPtr transformer;
		QList<Property*> xyz_props;
		QList<Property*> color_props;

		std::string readable_name;
		std::string lookup_name;
	};
	typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

	boost::recursive_mutex transformers_mutex_;
	M_TransformerInfo transformers_;
	bool new_xyz_transformer_;
	bool new_color_transformer_;
	bool needs_retransform_;

	pluginlib::ClassLoader<rviz::PointCloudTransformer>* transformer_class_loader_;
};

} // namespace rtabmap_ros

#endif
