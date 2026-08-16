// ============================================================================
// 文件名：map_matcher.h
// 用途：全局匹配器（MapMatcher）的头文件。负责"查询键帧 vs 离线地图键帧"的
//       全局定位匹配：先由 ScanContext 检索最近键帧，再用 Quatro（FPFH 特征
//       粗配准）+ NanoGICP 精配准（或仅 NanoGICP）得到相对变换。本包仍在
//       测试阶段（根目录有 COLCON_IGNORE，暂不参与编译）。
// 结构：
//   - 配置结构体：NanoGICPConfig / QuatroConfig / MapMatcherConfig
//   - RegistrationOutput：配准结果（有效性/收敛性/分数/相对变换）
//   - MapMatcher：核心匹配类
// 依赖：Scancontext.h（回环检索）、quatro（粗配准）、nano_gicp（精配准）、
//       pose_pcd.hpp、utilities.hpp
// ============================================================================

#ifndef FAST_LIO_LOCALIZATION_SC_QN_MAP_MATCHER_H
#define FAST_LIO_LOCALIZATION_SC_QN_MAP_MATCHER_H

///// C++ common headers
#include <tuple>
#include <vector>
#include <memory>
#include <limits>
#include <iostream>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
///// Eigen
#include <Eigen/Eigen>
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// ScanContext
#include <Scancontext.h>
///// coded headers
#include "pose_pcd.hpp"
#include "utilities.hpp"
// 配准输入点云对（源、目标）
using PcdPair = std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

// NanoGICP 精配准参数：线程数、对应点随机数、迭代上限、RANSAC、最大对应
// 距离与配准分数阈值等
struct NanoGICPConfig
{
    int nano_thread_number_ = 0;
    int nano_correspondences_number_ = 15;
    int nano_max_iter_ = 32;
    int nano_ransac_max_iter_ = 5;
    double max_corr_dist_ = 2.0;
    double icp_score_thr_ = 10.0;
    double transformation_epsilon_ = 0.01;
    double euclidean_fitness_epsilon_ = 0.01;
    double ransac_outlier_rejection_threshold_ = 1.0;
};

// Quatro 粗配准参数：FPFH 特征半径、噪声界、GNC 鲁棒优化因子等，用于估计
// 全局初始位姿（抗局部极小）
struct QuatroConfig
{
    bool use_optimized_matching_ = true;
    bool estimat_scale_ = false;
    int quatro_max_num_corres_ = 500;
    int quatro_max_iter_ = 50;
    double quatro_distance_threshold_ = 30.0;
    double fpfh_normal_radius_ = 0.30; // It should be 2.5 - 3.0 * `voxel_res`
    double fpfh_radius_ = 0.50;        // It should be 5.0 * `voxel_res`
    double noise_bound_ = 0.30;
    double rot_gnc_factor_ = 1.40;
    double rot_cost_diff_thr_ = 0.0001;
};

// 匹配器总配置：是否启用 Quatro、子地图键帧范围、体素分辨率、ScanContext
// 距离阈值，以及 GICP/Quatro 子配置
struct MapMatcherConfig
{
    bool enable_quatro_ = true;
    int num_submap_keyframes_ = 10;
    double voxel_res_ = 0.1;
    double scancontext_max_correspondence_distance_;
    NanoGICPConfig gicp_config_;
    QuatroConfig quatro_config_;
};

// 配准结果：is_valid_ 是否通过阈值判定，is_converged_ 是否收敛，
// score_ 为配准分数（越低越好），pose_between_eig_ 为源到目标的相对变换
struct RegistrationOutput
{
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::max();
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};

// 职责：全局匹配器。内部维护 ScanContext 数据库（离线地图键帧注册）、
//       NanoGICP 精配准器与 Quatro 粗配准器，对外提供"检索最近键帧 +
//       粗到精配准"的完整流程。
// 关键流程：fetchClosestKeyframeIdx（ScanContext 回环候选 + 距离过滤）
//       --> setSrcAndDstCloud（构造源/目标点云）--> coarseToFineAlignment
//       或 icpAlignment（按配置选择）。
class MapMatcher
{
private:
    // ScanContext 数据库：生成并保存地图键帧描述子，用于回环候选检索
    SCManager sc_manager_;
    // NanoGICP 精配准器（scan-to-map 局部精配准）
    nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
    // Quatro 粗配准器（FPFH 特征全局配准，提供初始位姿）
    std::shared_ptr<quatro<PointType>> quatro_handler_ = nullptr;
    // 最近一次检索到的最近键帧索引（-1 表示未找到）
    int closest_keyframe_idx_ = -1;
    // 最近一次匹配的源/目标点云（调试可视化用）
    pcl::PointCloud<PointType>::Ptr src_cloud_;
    pcl::PointCloud<PointType>::Ptr dst_cloud_;
    // Quatro 粗对齐后的点云（调试可视化用）
    pcl::PointCloud<PointType> coarse_aligned_;
    // 精配准最终对齐点云（调试可视化用）
    pcl::PointCloud<PointType> aligned_;
    // 匹配器配置（构造函数传入）
    MapMatcherConfig config_;

public:
    // 按配置初始化 NanoGICP（精配准）与 Quatro（粗配准）实例
    explicit MapMatcher(const MapMatcherConfig &config);
    ~MapMatcher();
    // 把一个键帧点云注册进 ScanContext 数据库（离线地图加载与在线键帧都会调用）
    void updateScancontext(pcl::PointCloud<PointType> cloud);
    // 用 ScanContext 检索与查询帧最相似的键帧作为回环候选，并校验位姿距离
    // 是否在阈值内；无候选或距离过远返回 -1
    int fetchClosestKeyframeIdx(const PosePcd &front_keyframe,
                                const std::vector<PosePcdReduced> &saved_map);
    // 构造配准输入：源为查询键帧（变换到地图系）；目标为最近键帧本身
    // （Quatro 模式）或它前后 submap_range 帧拼接的子地图（ICP 模式，经验上
    // scan-to-submap 效果更好）；均做体素降采样后返回
    PcdPair setSrcAndDstCloud(const PosePcd &query_keyframe,
                              const std::vector<PosePcdReduced> &saved_keyframes,
                              const int dst_idx,
                              const int submap_range,
                              const double voxel_res,
                              const bool enable_quatro);
    // 纯 NanoGICP 精配准：收敛且 fitness score 低于阈值才算有效（分数越低
    // 越好），返回相对变换
    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &dst);
    // 粗到精级联配准：Quatro 先给出全局初始变换（抗局部极小），再 NanoGICP
    // 精配准；最终变换 = GICP 结果 * Quatro 结果
    RegistrationOutput coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst);
    // 全局匹配总入口：构造点云对后按配置选择粗到精级联或纯 GICP，并保存
    // 源/目标点云供调试发布
    RegistrationOutput performMapMatcher(const PosePcd &query_keyframe,
                                         const std::vector<PosePcdReduced> &saved_keyframes,
                                         const int closest_keyframe_idx);
    // 返回最近一次匹配的源点云（调试可视化用）
    pcl::PointCloud<PointType> getSourceCloud();
    // 返回最近一次匹配的目标点云（调试可视化用）
    pcl::PointCloud<PointType> getTargetCloud();
    // 返回 Quatro 粗对齐后的点云（调试可视化用）
    pcl::PointCloud<PointType> getCoarseAlignedCloud();
    // 返回精配准最终对齐点云（调试可视化用；ICP-only 模式下同样有效）
    pcl::PointCloud<PointType> getFinalAlignedCloud();
    // 返回最近一次检索到的最近键帧索引
    int getClosestKeyframeidx();
};

#endif // FAST_LIO_LOCALIZATION_SC_QN_MAP_MATCHER_H
