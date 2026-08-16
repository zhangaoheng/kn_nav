// ============================================================================
// 文件名：map_matcher.cpp
// 用途：MapMatcher 匹配器的实现：ScanContext 最近键帧检索、源/目标点云构造、
//       Quatro 粗配准 + NanoGICP 精配准（或纯 GICP）的级联流程。
// 结构：
//   - 构造函数：NanoGICP / Quatro 初始化
//   - updateScancontext：地图键帧注册进 ScanContext 数据库
//   - fetchClosestKeyframeIdx：ScanContext 回环候选 + 位姿距离过滤
//   - setSrcAndDstCloud：构造配准输入点云对
//   - icpAlignment / coarseToFineAlignment / performMapMatcher：配准流程
// 依赖：Scancontext.h、quatro、nano_gicp、utilities.hpp、pose_pcd.hpp
// ============================================================================

#include "map_matcher.h"

// 按配置初始化 NanoGICP 精配准器（线程/迭代/阈值等）与 Quatro 粗配准器
// （FPFH 半径/噪声界/GNC 因子等），并分配源/目标点云容器
MapMatcher::MapMatcher(const MapMatcherConfig &config)
{
    config_ = config;
    const auto &gc = config_.gicp_config_;
    const auto &qc = config_.quatro_config_;
    ////// nano_gicp init
    nano_gicp_.setNumThreads(gc.nano_thread_number_);
    nano_gicp_.setCorrespondenceRandomness(gc.nano_correspondences_number_);
    nano_gicp_.setMaximumIterations(gc.nano_max_iter_);
    nano_gicp_.setRANSACIterations(gc.nano_ransac_max_iter_);
    nano_gicp_.setMaxCorrespondenceDistance(gc.max_corr_dist_);
    nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon_);
    nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
    nano_gicp_.setRANSACOutlierRejectionThreshold(gc.ransac_outlier_rejection_threshold_);
    ////// quatro init
    quatro_handler_ = std::make_shared<quatro<PointType>>(qc.fpfh_normal_radius_,
                                                          qc.fpfh_radius_,
                                                          qc.noise_bound_,
                                                          qc.rot_gnc_factor_,
                                                          qc.rot_cost_diff_thr_,
                                                          qc.quatro_max_iter_,
                                                          qc.estimat_scale_,
                                                          qc.use_optimized_matching_,
                                                          qc.quatro_distance_threshold_,
                                                          qc.quatro_max_num_corres_);
    src_cloud_.reset(new pcl::PointCloud<PointType>);
    dst_cloud_.reset(new pcl::PointCloud<PointType>);
}

MapMatcher::~MapMatcher() {}

// 把键帧点云生成 ScanContext 描述子并存入数据库（离线地图加载与在线键帧都会调用）
void MapMatcher::updateScancontext(pcl::PointCloud<PointType> cloud)
{
    sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}

// 用 ScanContext 检索与查询帧最相似的键帧作为回环候选并返回其索引；
// 无候选或候选距离超出阈值时返回 -1（表示本次不做匹配）
int MapMatcher::fetchClosestKeyframeIdx(const PosePcd &front_keyframe,
                                        const std::vector<PosePcdReduced> &saved_map)
{
    // from ScanContext, get the loop candidate
    std::pair<int, float> sc_detected_ = sc_manager_.detectLoopClosureIDGivenScan(front_keyframe.pcd_); // int: nearest node index,
                                                                                                        // float: relative yaw
    int candidate_keyframe_idx = sc_detected_.first;
    if (candidate_keyframe_idx >= 0) // if exists
    {
        // if close enough
        // 候选键帧与查询帧的位姿距离须在阈值内，防止回环误检
        if ((saved_map[candidate_keyframe_idx].pose_eig_.block<3, 1>(0, 3) - front_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
        if ((saved_map[candidate_keyframe_idx].pose_eig_.block<3, 1>(0, 3) - front_keyframe.pose_corrected_eig_.block<3, 1>(0, 3))
                .norm() < config_.scancontext_max_correspondence_distance_)
        {
            return candidate_keyframe_idx;
        }
    }
    return -1;
}

// 构造配准输入点云对：
//   源 = 查询键帧经校正位姿变换到地图系；目标 = 最近键帧本身（Quatro 模式）
//   或它前后 submap_range 帧拼接的子地图（纯 ICP 模式）；返回体素降采样后的点云对
PcdPair MapMatcher::setSrcAndDstCloud(const PosePcd &query_keyframe,
                                      const std::vector<PosePcdReduced> &saved_keyframes,
                                      const int dst_idx,
                                      const int submap_range,
                                      const double voxel_res,
                                      const bool enable_quatro)
{
    pcl::PointCloud<PointType> dst_accum, src_out;
    int num_approx = saved_keyframes[dst_idx].pcd_.size() * 2 * submap_range;
    dst_accum.reserve(num_approx);

    // 源点云：查询键帧变换到地图系（使用校正位姿）
    src_out = transformPcd(query_keyframe.pcd_, query_keyframe.pose_corrected_eig_);
    src_out = transformPcd(query_keyframe.pcd_, query_keyframe.pose_corrected_eig_);
    if (enable_quatro)
    {
        dst_accum = transformPcd(saved_keyframes[dst_idx].pcd_, saved_keyframes[dst_idx].pose_eig_);
    }
    else
    {
        // For ICP matching,
        // empirically scan-to-submap matching works better
        for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
        {
            if (i >= 0 && i < static_cast<int>(saved_keyframes.size() - 1))
            {
                dst_accum += transformPcd(saved_keyframes[i].pcd_, saved_keyframes[i].pose_eig_);
            }
        }
    }
    return {*voxelizePcd(src_out, voxel_res), *voxelizePcd(dst_accum, voxel_res)};
}

// 纯 NanoGICP 精配准：设置源/目标并计算协方差后 align，以"收敛 + fitness
// score 低于阈值"判定有效（分数越低越相似），返回相对变换
RegistrationOutput MapMatcher::icpAlignment(const pcl::PointCloud<PointType> &src,
                                            const pcl::PointCloud<PointType> &dst)
{
    RegistrationOutput reg_output;
    aligned_.clear();
    // merge subkeyframes before ICP
    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;
    nano_gicp_.setInputSource(src_cloud);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(dst_cloud);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_);

    // handle results
    reg_output.score_ = nano_gicp_.getFitnessScore();
    // if matchness score is lower than threshold, (lower is better)
    if (nano_gicp_.hasConverged() && reg_output.score_ < config_.gicp_config_.icp_score_thr_)
    {
        reg_output.is_valid_ = true;
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = nano_gicp_.getFinalTransformation().cast<double>();
    }
    return reg_output;
}

// 粗到精级联配准：Quatro 用 FPFH 特征做全局配准得到初始变换（不收敛则直接
// 返回无效），再用 NanoGICP 精配准修正；最终变换 = 精配准结果 * 粗配准结果
RegistrationOutput MapMatcher::coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                                     const pcl::PointCloud<PointType> &dst)
{
    RegistrationOutput reg_output;
    coarse_aligned_.clear();

    reg_output.pose_between_eig_ = (quatro_handler_->align(src, dst, reg_output.is_converged_));
    if (!reg_output.is_converged_)
    {
        return reg_output;
    }
    else // if valid,
    {
        // coarse align with the result of Quatro
        coarse_aligned_ = transformPcd(src, reg_output.pose_between_eig_);
        // 在粗对齐结果之上再做精配准（逐级修正）
        const auto &fine_output = icpAlignment(coarse_aligned_, dst);
        const auto &fine_output = icpAlignment(coarse_aligned_, dst);
        const auto quatro_tf_ = reg_output.pose_between_eig_;
        reg_output = fine_output;
        // 最终变换 = GICP 精配准结果 * Quatro 粗配准结果（先粗后精）
        reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * quatro_tf_;
        reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * quatro_tf_;
    }
    return reg_output;
}

// 全局匹配总入口：按配置构造源/目标点云，Quatro 模式走粗到精级联，
// 否则直接纯 GICP；同时保存源/目标点云供主类调试发布
RegistrationOutput MapMatcher::performMapMatcher(const PosePcd &query_keyframe,
                                                 const std::vector<PosePcdReduced> &saved_keyframes,
                                                 const int closest_keyframe_idx)
{
    RegistrationOutput reg_output;
    closest_keyframe_idx_ = closest_keyframe_idx;
    if (closest_keyframe_idx_ >= 0)
    {
        // Quatro + NANO-GICP to check loop (from front_keyframe to closest
        // keyframe's neighbor)
        const auto &[src_cloud, dst_cloud] = setSrcAndDstCloud(query_keyframe,
                                                               saved_keyframes,
                                                               closest_keyframe_idx_,
                                                               config_.num_submap_keyframes_,
                                                               config_.voxel_res_,
                                                               config_.enable_quatro_);
        // Only for visualization
        *src_cloud_ = src_cloud;
        *dst_cloud_ = dst_cloud;

        if (config_.enable_quatro_)
        {
            std::cout << "\033[1;35mExecute coarse-to-fine alignment: "
                      << src_cloud.size() << " vs " << dst_cloud.size() << "\033[0m\n";
            return coarseToFineAlignment(src_cloud, dst_cloud);
        }
        else
        {
            std::cout << "\033[1;35mExecute GICP: " << src_cloud.size() << " vs "
                      << dst_cloud.size() << "\033[0m\n";
            return icpAlignment(src_cloud, dst_cloud);
        }
    }
    else
    {
        return reg_output; // dummy output whose `is_valid` is false
    }
}

// 返回最近一次匹配的源点云（供主类调试发布）
pcl::PointCloud<PointType> MapMatcher::getSourceCloud()
{
    return *src_cloud_;
}

// 返回最近一次匹配的目标点云（供主类调试发布）
pcl::PointCloud<PointType> MapMatcher::getTargetCloud()
{
    return *dst_cloud_;
}

// 返回 Quatro 粗对齐后的点云（供主类调试发布）
pcl::PointCloud<PointType> MapMatcher::getCoarseAlignedCloud()
{
    return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
// 返回精配准最终对齐点云（供主类调试发布）
pcl::PointCloud<PointType> MapMatcher::getFinalAlignedCloud()
{
    return aligned_;
}

// 返回最近一次检索到的最近键帧索引
int MapMatcher::getClosestKeyframeidx()
{
    return closest_keyframe_idx_;
}
