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

/**
 * Modified matlabbe:
 * Added option to choose between unknown, free and marked cells
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>

namespace rtabmap_costmap_plugins 
{

// FREE, UNKNOWN, MARKED
double g_voxel_colors_r[] = {0.0, 1.0, 1.0};
double g_voxel_colors_g[] = {1.0, 1.0, 0.0};
double g_voxel_colors_b[] = {1.0, 1.0, 0.0};
double g_voxel_colors_a[] = {0.5, 0.1, 0.5};

class VoxelMarker: public rclcpp::Node
{
public:
	explicit VoxelMarker(const rclcpp::NodeOptions & options) :
    rclcpp::Node("voxel_marker", options)
  {
    cell_type_ = this->declare_parameter("cell_type", (int)nav2_voxel_grid::VoxelStatus::MARKED);
    color_r_ = this->declare_parameter("r", g_voxel_colors_r[cell_type_]);
    color_g_ = this->declare_parameter("g", g_voxel_colors_g[cell_type_]);
    color_b_ = this->declare_parameter("b", g_voxel_colors_b[cell_type_]);
    color_a_ = this->declare_parameter("a", g_voxel_colors_a[cell_type_]);  
    
    voxel_sub_ = this->create_subscription<nav2_msgs::msg::VoxelGrid>("voxel_grid", rclcpp::QoS(1), std::bind(&VoxelMarker::voxelCallback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", rclcpp::QoS(1));

  }
	virtual ~VoxelMarker() {}

  void voxelCallback(const nav2_msgs::msg::VoxelGrid::SharedPtr grid)
  {
    if (grid->data.empty())
    {
      RCLCPP_ERROR(get_logger(), "Received empty voxel grid");
      return;
    }

    visualization_msgs::msg::Marker m;
    m.header.frame_id = grid->header.frame_id;
    m.header.stamp = grid->header.stamp;
    m.ns = "voxel_grid";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = color_r_;
    m.color.g = color_g_;
    m.color.b = color_b_;
    m.color.a = color_a_;

    const uint32_t* data = &grid->data.front();
    const double x_origin = grid->origin.x;
    const double y_origin = grid->origin.y;
    const double z_origin = grid->origin.z;
    const double x_res = grid->resolutions.x;
    const double y_res = grid->resolutions.y;
    const double z_res = grid->resolutions.z;
    const uint32_t x_size = grid->size_x;
    const uint32_t y_size = grid->size_y;
    const uint32_t z_size = grid->size_z;
    for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid)
    {
      for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid)
      {
        for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid)
        {
          nav2_voxel_grid::VoxelStatus status = nav2_voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid, x_size, y_size, z_size,
                                                                          data);

          if (status == (nav2_voxel_grid::VoxelStatus)cell_type_)
          {
            geometry_msgs::msg::Point p;
            p.x = x_origin + (x_grid + 0.5) * x_res;
            p.y = y_origin + (y_grid + 0.5) * y_res;
            p.z = z_origin + (z_grid + 0.5) * z_res;
            m.points.push_back(p);
          }
        }
      }
    }
    m.scale.x = x_res;
    m.scale.y = y_res;
    m.scale.z = z_res;

    marker_pub_->publish(m);
  }

private:
  int cell_type_;
  double color_r_;
  double color_g_;
  double color_b_;
  double color_a_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
	rclcpp::Subscription<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_sub_;
};

} // rtabmap_costmap_plugins

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<rtabmap_costmap_plugins::VoxelMarker>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
}
