// ============================================================================
// 文件名：fast_lio_localization_sc_qn.h
// 用途：全局定位主节点类的头文件。本模块把 FAST-LIO 里程计输出与离线地图
//       （rosbag 键帧）做全局匹配（ScanContext 回环检索 + Quatro 粗配准 +
//       NanoGICP 精配准），输出校正后的位姿/轨迹。注意：本包仍处于测试阶段
//       （包根目录有 COLCON_IGNORE，暂不参与编译）。
// 结构：
//   - FastLioLocalizationScQn：唯一主类，聚合参数、共享数据、可视化数据、
//     ROS 发布/订阅与 MapMatcher 匹配器
//   - odom_pcd_sync_pol：里程计与点云的近似时间同步策略别名
// 数据流：
//   /Odometry 与 /cloud_registered（FAST-LIO 输出）--> odomPcdCallback 维护键帧
//   --> matchingTimerFunc 定时触发 MapMatcher 全局匹配 --> 更新累积校正 TF，
//   发布 /corrected_odom、/corrected_path、/pose_stamped 及调试点云
// 依赖：utilities.hpp（工具函数）、pose_pcd.hpp（键帧数据结构）、
//       map_matcher.h（全局匹配器）、Nano-GICP、Quatro、ScanContext
// ============================================================================

#ifndef FAST_LIO_LOCALIZATION_SC_QN_MAIN_H
#define FAST_LIO_LOCALIZATION_SC_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <utility> // pair, make_pair
///// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>               // load map
#include <rosbag/view.h>              // load map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/common/transforms.h>           //transformPointCloud
#include <pcl/conversions.h>                 //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/filters/voxel_grid.h>          //voxelgrid
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// coded headers
#include "utilities.hpp"
#include "pose_pcd.hpp"
#include "map_matcher.h"

using namespace std::chrono;
// 里程计与点云的近似时间同步策略（FAST-LIO 两话题时间戳不完全一致，用近似同步）
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
// 职责：全局定位主类。接收 FAST-LIO 的里程计/点云，按距离阈值维护键帧，
//       周期性调用 MapMatcher 做全局匹配，把匹配得到的校正变换叠加到里程计
//       输出上，并维护可视化用的原始/校正轨迹与匹配连线。
// 线程模型：odomPcdCallback 运行在订阅回调线程，matchingTimerFunc 运行在
//       定时器线程，两者通过 keyframes_mutex_ 与 vis_mutex_ 保护共享数据。
class FastLioLocalizationScQn
{
private:
    ///// basic params
    std::string map_frame_;
    ///// shared data - odom and pcd
    std::mutex keyframes_mutex_, vis_mutex_;
    bool is_initialized_ = false;
    // 当前键帧序号，同时用作轨迹数组下标
    int current_keyframe_idx_ = 0;
    // 最近保存的键帧（待匹配的最新帧）
    PosePcd last_keyframe_;
    std::vector<PosePcdReduced> saved_map_from_bag_;
    // 累积校正变换：实时位姿 = last_corrected_TF_ * 里程计位姿
    Eigen::Matrix4d last_corrected_TF_ = Eigen::Matrix4d::Identity();
    ///// map match
    double keyframe_dist_thr_;
    double voxel_res_;
    ///// visualize
    // 地图发布开关：有订阅者时只发一次，订阅者消失后复位，避免反复发布大点云
    bool saved_map_vis_switch_ = true;
    tf::TransformBroadcaster broadcaster_;
    nav_msgs::Path raw_odom_path_, corrected_odom_path_;
    // 校正前/后位姿点对，用于可视化匹配连线
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> matched_pairs_xyz_; // for vis
    pcl::PointCloud<pcl::PointXYZ> raw_odoms_, corrected_odoms_;
    pcl::PointCloud<PointType> saved_map_pcd_; // for vis
    ///// ros
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, realtime_pose_pub_, map_match_pub_;
    ros::Publisher saved_map_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    ros::Timer match_timer_;
    // odom, pcd sync subscriber
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_ = nullptr;
    ///// Map match
    std::shared_ptr<MapMatcher> map_matcher_;

public:
    explicit FastLioLocalizationScQn(const ros::NodeHandle &n_private);
    ~FastLioLocalizationScQn() {};

private:
    // methods
    // 把键帧的原始/校正位姿追加进轨迹点云与路径容器（仅用于可视化）
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    // 依据平移距离阈值（keyframe_dist_thr_，基于校正后位姿）判断当前帧是否为新键帧
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    // 把校正/原始位姿点对转成 rviz 线段 Marker，可视化匹配连线
    visualization_msgs::Marker getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs);
    // 从 rosbag 读取离线地图键帧（话题 /keyframe_pcd、/keyframe_pose），
    // 并注册进 ScanContext 数据库与可视化地图点云
    void loadMap(const std::string &saved_map_path);
    // cb
    // 里程计+点云同步回调：实时发布校正位姿与 TF，并按键帧策略保存最新帧
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    // 匹配定时器回调：对最新键帧执行全局匹配，成功则更新累积校正 TF 并发布结果
    void matchingTimerFunc(const ros::TimerEvent &event);
};

#endif
