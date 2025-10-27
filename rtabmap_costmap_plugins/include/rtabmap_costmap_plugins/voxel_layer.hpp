/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef RTABMAP_COSTMAP_PLUGINS__VOXEL_LAYER_HPP_
#define RTABMAP_COSTMAP_PLUGINS__VOXEL_LAYER_HPP_

#include <vector>

#include <rtabmap_costmap_plugins/visibility.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>

namespace rtabmap_costmap_plugins
{

/**
 * @class VoxelLayer
 * @brief Takes laser and pointcloud data to populate a 3D voxel representation of the environment
 */
class VoxelLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  RTABMAP_COSTMAP_PLUGINS_PUBLIC
  VoxelLayer()
  : voxel_grid_(0, 0, 0)
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class's parent class Costmap2D
  }

  /**
   * @brief Voxel Layer destructor
   */
  virtual ~VoxelLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  /**
   * @brief Update the layer's origin to a new pose, often when in a rolling costmap
   */
  void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief If layer is discretely populated
   */
  bool isDiscretized()
  {
    return true;
  }

  /**
   * @brief Match the size of the master costmap
   */
  virtual void matchSize();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() {return true;}

protected:
  /**
   * @brief Reset internal maps
   */
  virtual void resetMaps();

  /**
   * @brief Use raycasting between 2 points to clear freespace
   */
  virtual void raytraceFreespace(
    const nav2_costmap_2d::Observation & clearing_observation,
    double * min_x, double * min_y,
    double * max_x,
    double * max_y);

  bool publish_voxel_;
  std::string robot_base_frame_;
  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_pub_;
  nav2_voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;
  int unknown_threshold_, mark_threshold_, size_z_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    clearing_endpoints_pub_;

  /**
   * @brief Convert world coordinates into map coordinates
   */
  inline bool worldToMap3DFloat(
    double wx, double wy, double wz, double & mx, double & my,
    double & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    if (mx < size_x_ && my < size_y_ && mz < size_z_) {
      return true;
    }

    return false;
  }

  /**
   * @brief Convert world coordinates into map coordinates
   */
  inline bool worldToMap3D(
    double wx, double wy, double wz, unsigned int & mx, unsigned int & my,
    unsigned int & mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ && my < size_y_ && mz < (unsigned int)size_z_) {
      return true;
    }

    return false;
  }

  /**
   * @brief Convert map coordinates into world coordinates
   */
  inline void mapToWorld3D(
    unsigned int mx, unsigned int my, unsigned int mz, double & wx,
    double & wy,
    double & wz)
  {
    // returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  /**
   * @brief Find L2 norm distance in 3D
   */
  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
  }

  /**
   * @brief Get the height of the voxel sizes in meters
   */
  double getSizeInMetersZ() const
  {
    return (size_z_ - 1 + 0.5) * z_resolution_;
  }

  /**
   * @brief  Copy a region of a source map into a destination map
   * @param  source_map The source map
   * @param sm_lower_left_x The lower left x point of the source map to start the copy
   * @param sm_lower_left_y The lower left y point of the source map to start the copy
   * @param sm_size_x The x size of the source map
   * @param  dest_map The destination map
   * @param dm_lower_left_x The lower left x point of the destination map to start the copy
   * @param dm_lower_left_y The lower left y point of the destination map to start the copy
   * @param dm_size_x The x size of the destination map
   * @param region_size_x The x size of the region to copy
   * @param region_size_y The y size of the region to copy
   */
  template<typename data_type>
  void copyMapRegion3D(
    data_type * source_map, unsigned int sm_lower_left_x,
    unsigned int sm_lower_left_y,
    unsigned int sm_size_x, data_type * dest_map, unsigned int dm_lower_left_x,
    unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
    unsigned int region_size_y, int z_shift)
  {
    // we'll first need to compute the starting points for each map
    // this is like getting voxel column. We are not taking into account the z position of the voxel
    data_type * sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    data_type * dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

    uint32_t marked_bits_mask = (data_type) 0xFFFF0000;
    uint32_t unknown_bits_mask = (data_type) 0x0000FFFF;

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i) {
      memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));

      for (unsigned int j = 0; j < region_size_x; j++) {
        // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
				if (z_shift > 0) {
					dm_index[j] =
							// Shift marked cells, insert zeros for new unknowns
							((dm_index[j] & marked_bits_mask) >> z_shift & marked_bits_mask) |
							// Shift empty/unknown cells, insert ones for new unknowns
              (((dm_index[j] & unknown_bits_mask) >> z_shift | (~((data_type) 0) << (sizeof(data_type) * 4 - z_shift))) & unknown_bits_mask);

				} else if (z_shift < 0) {
					dm_index[j] =
							// Shift marked cells, insert zeros for new unknowns
							(dm_index[j] & marked_bits_mask) << z_shift * -1 |
							// Shift empty/unknown cells, insert ones for new unknowns
              ((dm_index[j] << z_shift * -1 & unknown_bits_mask) | ~(~((data_type) 0) << z_shift * -1));
				}
			}

      sm_index += sm_size_x;
      dm_index += dm_size_x;
    }
  }

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace rtabmap_costmap_plugins

#endif  // RTABMAP_COSTMAP_PLUGINS__VOXEL_LAYER_HPP_
