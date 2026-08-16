// ============================================================================
// 文件名：utilities.hpp
// 用途：通用工具函数头文件：Eigen 位姿 <-> ROS 消息/tf 的转换、PCL 点云 <->
//       ROS 消息转换、点云位姿变换与体素降采样。
// 结构：
//   - PointType：全包统一的点类型（pcl::PointXYZI）
//   - 转换函数：poseEigToPoseStamped / poseEigToROSTf / pclToPclRos
//   - 变换函数：transformPcd / voxelizePcd（两个重载：值传入与智能指针传入）
// 依赖：ROS tf、PCL、Eigen
// ============================================================================

#ifndef FAST_LIO_LOCALIZATION_SC_QN_UTILITIES_HPP
#define FAST_LIO_LOCALIZATION_SC_QN_UTILITIES_HPP

///// common headers
#include <string>
///// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
///// PCL
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/conversions.h>                 //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h> //voxelgrid
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)

// 统一点类型：强度点云，兼容 FAST-LIO 的 /cloud_registered 输出与 NanoGICP/Quatro
using PointType = pcl::PointXYZI;

//////////////////////////////////////////////////////////////////////
///// conversions
// Eigen 4x4 位姿 -> geometry_msgs::PoseStamped（默认 frame 为 map）
inline geometry_msgs::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                                                       std::string frame_id = "map")
{
    double r, p, y;
    tf::Matrix3x3 mat;
    tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
    mat.getRPY(r, p, y);
    tf::Quaternion quat = tf::createQuaternionFromRPY(r, p, y);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    pose.pose.orientation.w = quat.getW();
    pose.pose.orientation.x = quat.getX();
    pose.pose.orientation.y = quat.getY();
    pose.pose.orientation.z = quat.getZ();
    return pose;
}

// Eigen 4x4 位姿 -> tf::Transform（用于 TF 广播）
inline tf::Transform poseEigToROSTf(const Eigen::Matrix4d &pose)
{
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    return transform;
}

template<typename T>
// PCL 点云 -> sensor_msgs::PointCloud2（设置 frame_id 后供发布）
inline sensor_msgs::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud,
                                            std::string frame_id = "map")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

///// transformation
template<typename T>
// 按 4x4 位姿变换整个点云（空点云直接返回原样）
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf)
{
    if (cloud_in.size() == 0)
    {
        return cloud_in;
    }
    pcl::PointCloud<T> pcl_out = cloud_in;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
    return pcl_out;
}

// 体素降采样，配准前统一点云分辨率（接收值拷贝版本）
inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in,
                                                   const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_in_ptr->reserve(pcd_in.size());
    pcd_out->reserve(pcd_in.size());
    *pcd_in_ptr = pcd_in;
    voxelgrid.setInputCloud(pcd_in_ptr);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

// 体素降采样（接收智能指针版本，避免拷贝）
inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
                                                   const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_out->reserve(pcd_in->size());
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

#endif
