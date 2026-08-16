// ============================================================================
// 文件名：pose_pcd.hpp
// 用途：键帧数据结构的头文件。定义"位姿 + 点云"的打包结构：PosePcd（在线
//       键帧，含原始位姿与校正位姿）与 PosePcdReduced（离线地图键帧），并
//       提供从 ROS 消息构造的构造函数。
// 结构：
//   - PosePcd：在线键帧（雷达系点云 + 原始位姿 + 校正位姿 + 索引）
//   - PosePcdReduced：离线地图键帧（地图系点云 + 位姿 + 索引）
// 依赖：utilities.hpp（PointType 定义）
// ============================================================================

#ifndef FAST_LIO_LOCALIZATION_SC_QN_POSE_PCD_HPP
#define FAST_LIO_LOCALIZATION_SC_QN_POSE_PCD_HPP

///// coded headers
#include "utilities.hpp"

// 在线键帧：pcd_ 为雷达系点云，pose_eig_ 为 FAST-LIO 原始位姿，
// pose_corrected_eig_ 为全局匹配校正后的位姿，idx_ 为键帧序号
struct PosePcd
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    int idx_;
    bool processed_ = false;
    PosePcd() {}
    PosePcd(const nav_msgs::Odometry &odom_in,
            const sensor_msgs::PointCloud2 &pcd_in,
            const int &idx_in);
};

// 离线地图键帧（从 rosbag 加载）：只保留位姿与点云，用于匹配与检索
struct PosePcdReduced
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    int idx_;
    PosePcdReduced() {}
    PosePcdReduced(const geometry_msgs::PoseStamped &pose_in,
                   const sensor_msgs::PointCloud2 &pcd_in,
                   const int &idx_in);
};

// 由里程计 + 点云消息构造键帧。坐标系约定：FAST-LIO 发布的点云在世界系，
// 因此用位姿的逆变换把它变换回雷达系存储（见下方 transformPcd 调用）
inline PosePcd::PosePcd(const nav_msgs::Odometry &odom_in,
                        const sensor_msgs::PointCloud2 &pcd_in,
                        const int &idx_in)
{
    tf::Quaternion q(odom_in.pose.pose.orientation.x,
                     odom_in.pose.pose.orientation.y,
                     odom_in.pose.pose.orientation.z,
                     odom_in.pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat_tf(q);
    Eigen::Matrix3d rot_mat_eig;
    tf::matrixTFToEigen(rot_mat_tf, rot_mat_eig);
    pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;
    pcl::PointCloud<PointType> tmp_pcd_;
    pcl::fromROSMsg(pcd_in, tmp_pcd_);
    pcd_ = transformPcd(tmp_pcd_,
                        pose_eig_.inverse()); // FAST-LIO publish data in world
                                              // frame, so save it in LiDAR frame
    idx_ = idx_in;
}

// 由 PoseStamped + 点云消息构造离线地图键帧（点云保持地图系）
inline PosePcdReduced::PosePcdReduced(const geometry_msgs::PoseStamped &pose_in,
                                      const sensor_msgs::PointCloud2 &pcd_in,
                                      const int &idx_in)
{
    tf::Quaternion q(pose_in.pose.orientation.x,
                     pose_in.pose.orientation.y,
                     pose_in.pose.orientation.z,
                     pose_in.pose.orientation.w);
    tf::Matrix3x3 rot_mat_tf(q);
    Eigen::Matrix3d rot_mat_eig;
    tf::matrixTFToEigen(rot_mat_tf, rot_mat_eig);
    pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
    pose_eig_(0, 3) = pose_in.pose.position.x;
    pose_eig_(1, 3) = pose_in.pose.position.y;
    pose_eig_(2, 3) = pose_in.pose.position.z;
    pcl::fromROSMsg(pcd_in, pcd_);
    idx_ = idx_in;
}

#endif // FAST_LIO_LOCALIZATION_SC_QN_POSE_PCD_HPP
