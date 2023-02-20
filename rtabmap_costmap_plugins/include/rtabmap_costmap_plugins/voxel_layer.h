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
#ifndef RTABMAP_ROS_VOXEL_LAYER_H_
#define RTABMAP_ROS_VOXEL_LAYER_H_

#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/VoxelGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/VoxelPluginConfig.h>
#include <costmap_2d/obstacle_layer.h>
#include <voxel_grid/voxel_grid.h>

namespace rtabmap_costmap_plugins
{

class VoxelLayer : public costmap_2d::ObstacleLayer
{
public:
  VoxelLayer() :
      voxel_grid_(0, 0, 0)
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class's parent class Costmap2D.
  }

  virtual ~VoxelLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  void updateOrigin(double new_origin_x, double new_origin_y);
  bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();
  virtual void reset();


protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  virtual void resetMaps();

private:
  void reconfigureCB(costmap_2d::VoxelPluginConfig &config, uint32_t level);
  void clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info);
  virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig> *voxel_dsrv_;

  bool publish_voxel_;
  std::string robot_base_frame_;
  ros::Publisher voxel_pub_;
  voxel_grid::VoxelGrid voxel_grid_;
  double z_resolution_, origin_z_;
  unsigned int unknown_threshold_, mark_threshold_, size_z_;
  ros::Publisher clearing_endpoints_pub_;
  sensor_msgs::PointCloud clearing_endpoints_;

  inline bool worldToMap3DFloat(double wx, double wy, double wz, double& mx, double& my, double& mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
      return false;
    mx = ((wx - origin_x_) / resolution_);
    my = ((wy - origin_y_) / resolution_);
    mz = ((wz - origin_z_) / z_resolution_);
    if (mx < size_x_ && my < size_y_ && mz < size_z_)
      return true;

    return false;
  }

  inline bool worldToMap3D(double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz)
  {
    if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
      return false;

    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);
    mz = (int)((wz - origin_z_) / z_resolution_);

    if (mx < size_x_ && my < size_y_ && mz < size_z_)
      return true;

    return false;
  }

  inline void mapToWorld3D(unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz)
  {
    // returns the center point of the cell
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
    wz = origin_z_ + (mz + 0.5) * z_resolution_;
  }

  inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
  {
    return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
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
    void copyMapRegion3D(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y, int z_shift)
    {
        // we'll first need to compute the starting points for each map
        // this is like getting voxel column. We are not taking into account the z position of the voxel
        data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
        data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x  + dm_lower_left_x);

        uint32_t marked_bits_mask = (data_type) 0xFFFF0000;
        uint32_t unknown_bits_mask = (data_type) 0x0000FFFF;

        // now, we'll copy the source map into the destination map
        for (unsigned int i = 0; i < region_size_y; ++i)
        {
            memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
            for (unsigned int j = 0; j < region_size_x; j++) {
            	// known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
				if (z_shift > 0) {
					dm_index[j] =
							// Shift marked cells, insert zeros for new unknowns
							((dm_index[j] & marked_bits_mask) >> z_shift & marked_bits_mask) |
							// Shift empty/unknown cells, insert ones for new unknowns
						    (((dm_index[j] & unknown_bits_mask) >> z_shift | (~((data_type) 0) << sizeof(data_type) * 4 - z_shift)) & unknown_bits_mask);

				} else if (z_shift < 0) {
					dm_index[j] =
							// Shift marked cells, insert zeros for new unknowns
							(dm_index[j] & marked_bits_mask) << z_shift * -1 |
							// Shift empty/unknown cells, insert ones for new unknowns
						    ((dm_index[j] << z_shift * -1 & unknown_bits_mask) | ~(~((data_type) 0) << z_shift * -1));
				}
			}

            dm_index += dm_size_x;
            sm_index += sm_size_x;
        }
    }
};

}  // namespace rtabmap_ros

#endif  // RTABMAP_ROS_VOXEL_LAYER_H_
