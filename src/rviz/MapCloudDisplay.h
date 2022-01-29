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

#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <rtabmap_ros/visibility.h>
#include <rtabmap_ros/msg/map_data.hpp>
#include <rtabmap/core/Transform.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rviz_common/message_filter_display.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer_factory.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

#endif

namespace rviz_common
{

namespace properties
{

class BoolProperty;
class EnumProperty;
class IntProperty;
class FloatProperty;

}  // namespace properties

}  // namespace rviz_common

namespace rtabmap_ros
{

struct RTABMAP_ROS_PUBLIC CloudInfo
{
	CloudInfo();
	~CloudInfo();

	// clear the point cloud, but keep selection handler around
	void clear();

	rclcpp::Time receive_time_;

	Ogre::SceneManager *manager_;

	sensor_msgs::msg::PointCloud2::ConstSharedPtr message_;
	rtabmap::Transform pose_;
	int id_;

	Ogre::SceneNode *scene_node_;
	std::shared_ptr<rviz_rendering::PointCloud> cloud_;
	std::shared_ptr<rviz_default_plugins::PointCloudSelectionHandler> selection_handler_;

	std::vector<rviz_rendering::PointCloud::Point> transformed_points_;
};
typedef std::shared_ptr<CloudInfo> CloudInfoPtr;

/**
 * \class MapCloudDisplay
 * \brief Displays point clouds from rtabmap::MapData
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class RTABMAP_ROS_PUBLIC MapCloudDisplay: public rviz_common::MessageFilterDisplay<rtabmap_ros::msg::MapData>
{
Q_OBJECT
public:
	explicit MapCloudDisplay();
	virtual ~MapCloudDisplay() {}

	virtual void reset();
	virtual void update( float wall_dt, float ros_dt );

	bool auto_size_;

	rviz_common::properties::FloatProperty* point_world_size_property_;
	rviz_common::properties::FloatProperty* point_pixel_size_property_;
	rviz_common::properties::FloatProperty* alpha_property_;
	rviz_common::properties::EnumProperty* xyz_transformer_property_;
	rviz_common::properties::EnumProperty* color_transformer_property_;
	rviz_common::properties::EnumProperty* style_property_;
	rviz_common::properties::BoolProperty* cloud_from_scan_;
	rviz_common::properties::IntProperty* cloud_decimation_;
	rviz_common::properties::FloatProperty* cloud_max_depth_;
	rviz_common::properties::FloatProperty* cloud_min_depth_;
	rviz_common::properties::FloatProperty* cloud_voxel_size_;
	rviz_common::properties::FloatProperty* cloud_filter_floor_height_;
	rviz_common::properties::FloatProperty* cloud_filter_ceiling_height_;
	rviz_common::properties::FloatProperty* node_filtering_radius_;
	rviz_common::properties::FloatProperty* node_filtering_angle_;
	rviz_common::properties::StringProperty* download_namespace;
	rviz_common::properties::BoolProperty* download_map_;
	rviz_common::properties::BoolProperty* download_graph_;

public Q_SLOTS:
	void causeRetransform();

private Q_SLOTS:
	void updateStyle();
	void updateBillboardSize();
	void updateAlpha();
	void updateXyzTransformer();
	void updateColorTransformer();
	void setXyzTransformerOptions( rviz_common::properties::EnumProperty* prop );
	void setColorTransformerOptions( rviz_common::properties::EnumProperty* prop );
	void updateCloudParameters();
	void downloadNamespaceChanged();
	void downloadMap();
	void downloadGraph();

protected:
	/** @brief Process a single message.  Overridden from MessageFilterDisplay. */
	virtual void processMessage( const rtabmap_ros::msg::MapData::ConstSharedPtr cloud );
	void onInitialize();
private:
	void downloadMap(bool graphOnly);
	void processMapData(const rtabmap_ros::msg::MapData& map);

	/**
	* \brief Transforms the cloud into the correct frame, and sets up our renderable cloud
	*/
	bool transformCloud(const CloudInfoPtr& cloud, bool fully_update_transformers);

	std::shared_ptr<rviz_default_plugins::PointCloudTransformer> getXYZTransformer(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
	std::shared_ptr<rviz_default_plugins::PointCloudTransformer> getColorTransformer(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
	void updateTransformers( const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud );
	void retransform();

	void loadTransformers();
	void loadTransformer(
		std::shared_ptr<rviz_default_plugins::PointCloudTransformer> trans,
	    std::string name,
	    const std::string & lookup_name);

	void setPropertiesHidden( const QList<rviz_common::properties::Property*>& props, bool hide );
	void fillTransformerOptions( rviz_common::properties::EnumProperty* prop, uint32_t mask );

private:
	rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr republishNodeDataPub_;
	
	std::map<int, CloudInfoPtr> cloud_infos_;

	std::map<int, CloudInfoPtr> new_cloud_infos_;
	std::mutex new_clouds_mutex_;

	std::set<int> nodeDataReceived_;
	bool fromScan_;

	std::map<int, rtabmap::Transform> current_map_;
	std::mutex current_map_mutex_;
	bool current_map_updated_;

	int lastCloudAdded_;

	struct TransformerInfo
	{
		std::shared_ptr<rviz_default_plugins::PointCloudTransformer> transformer;
		QList<rviz_common::properties::Property*> xyz_props;
		QList<rviz_common::properties::Property*> color_props;

		std::string readable_name;
		std::string lookup_name;
	};
	typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

	std::recursive_mutex transformers_mutex_;
	M_TransformerInfo transformers_;
	bool new_xyz_transformer_;
	bool new_color_transformer_;
	bool needs_retransform_;

	std::unique_ptr<rviz_default_plugins::PointCloudTransformerFactory> transformer_factory_;

	rclcpp::Clock::SharedPtr clock_;

	static const std::string message_status_name_;
};

} // namespace rtabmap_ros

#endif
