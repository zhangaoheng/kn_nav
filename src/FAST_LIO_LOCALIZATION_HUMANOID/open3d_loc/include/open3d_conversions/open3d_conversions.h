// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// 文件：open3d_conversions.h
// 说明：open3d_loc 包的核心数据桥接头文件：在
//       sensor_msgs::msg::PointCloud2 与 Open3D 点云（几何版 geometry 与
//       Tensor 版 t::geometry）之间做双向转换，是雷达话题与 ICP 配准的接口层。
// ============================================================================
#ifndef OPEN3D_CONVERSIONS_HPP_
#define OPEN3D_CONVERSIONS_HPP_

// Open3D
#include <open3d/Open3D.h>

// ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Eigen
#include <Eigen/Dense>

// C++
#include <memory>
#include <string>

// open3d_conversions 命名空间：PointCloud2 与 Open3D 点云的互转入口。
namespace open3d_conversions
{
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointCloud2ConstPtr = std::shared_ptr<const PointCloud2>;

    /**
     * @brief Copy data from a open3d::geometry::PointCloud to a sensor_msgs::msg::PointCloud2
     *
     * @param pointcloud Reference to the open3d PointCloud
     * @param ros_pc2 Reference to the sensor_msgs PointCloud2
     * @param frame_id The string to be placed in the frame_id of the PointCloud2
     */
// Open3D 几何点云 -> ROS PointCloud2：带/不带颜色两种字段布局。
    void open3dToRos(const open3d::geometry::PointCloud &pointcloud, PointCloud2 &ros_pc2,
                     std::string frame_id = "open3d_pointcloud");

    /**
     * @brief Copy data from a sensor_msgs::msg::PointCloud2 to a open3d::geometry::PointCloud
     *
     * @param ros_pc2 Reference to the sensor_msgs PointCloud2
     * @param o3d_pc Reference to the open3d PointCloud
     * @param skip_colors If true, only xyz fields will be copied
     */
// ROS PointCloud2 -> Open3D 几何点云；skip_colors=true 时只拷贝 xyz。
    void rosToOpen3d(const PointCloud2ConstPtr &ros_pc2, open3d::geometry::PointCloud &o3d_pc,
                     bool skip_colors = false);

    /**
     * @brief Copy data from a open3d::t::geometry::PointCloud to a sensor_msgs::msg::PointCloud2
     *
     * @param pointcloud Reference to the open3d tgeometry PointCloud
     * @param ros_pc2 Reference to the sensor_msgs PointCloud2
     * @param frame_id The string to be placed in the frame_id of the PointCloud2
     * @param t_num_fields Twice the number of fields that the pointcloud contains
     * @param var_args Strings of field names followed succeeded by their datatype ("int" / "float")
     */
    void open3dToRos(const open3d::t::geometry::PointCloud &pointcloud, PointCloud2 &ros_pc2,
                     std::string frame_id = "open3d_pointcloud", int t_num_fields = 2, ...);

    /**
     * @brief Copy data from a sensor_msgs::msg::PointCloud2 to a open3d::t::geometry::PointCloud
     *
     * @param ros_pc2 Reference to the sensor_msgs PointCloud2
     * @param o3d_pc Reference to the open3d tgeometry PointCloud
     * @param skip_colors If true, only xyz fields will be copied
     */
    void rosToOpen3d(const PointCloud2ConstPtr &ros_pc2, open3d::t::geometry::PointCloud &o3d_pc,
                     bool skip_colors = false);
} // namespace open3d_conversions

#endif // OPEN3D_CONVERSIONS_HPP_