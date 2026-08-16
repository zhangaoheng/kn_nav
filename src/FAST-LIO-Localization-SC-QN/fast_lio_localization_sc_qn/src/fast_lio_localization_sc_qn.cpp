// ============================================================================
// 文件名：fast_lio_localization_sc_qn.cpp
// 用途：FastLioLocalizationScQn 主类的实现：参数读取、离线地图加载、
//       里程计/点云同步回调（键帧维护）与定时全局匹配（校正 TF 更新与发布）。
// 结构：
//   - 构造函数：读参数、初始化 MapMatcher、加载地图、建立 ROS 发布/订阅/定时器
//   - odomPcdCallback：实时位姿修正与键帧保存
//   - matchingTimerFunc：定时执行全局匹配并更新累积校正 TF
//   - 辅助：updateOdomsAndPaths / getMatchMarker / checkIfKeyframe / loadMap
// 数据流：FAST-LIO 的 /Odometry 与 /cloud_registered --> 键帧 --> MapMatcher
//       全局匹配 --> last_corrected_TF_ 累积校正 --> 发布校正轨迹与可视化。
//       注意：本包仍处于测试阶段，未参与实际编译（COLCON_IGNORE）。
// ============================================================================

#include "fast_lio_localization_sc_qn.h"

// 构造函数：从私有参数服务器读取全部参数（地图路径、键帧阈值、匹配频率、
// GICP/Quatro 参数等），初始化 MapMatcher，加载离线地图，并建立发布者、
// 同步订阅与匹配定时器。注意：包尚未编译，参数默认值可能未与 config.yaml 对齐。
FastLioLocalizationScQn::FastLioLocalizationScQn(const ros::NodeHandle &n_private):
    nh_(n_private)
{
    ////// ROS params
    // temp vars, only used in constructor
    std::string saved_map_path;
    double map_match_hz;
    MapMatcherConfig mm_config;
    auto &gc = mm_config.gicp_config_;
    auto &qc = mm_config.quatro_config_;
    // get params
    /* basic */
    nh_.param<std::string>("/basic/map_frame", map_frame_, "map");
    nh_.param<std::string>("/basic/saved_map", saved_map_path, "/home/mason/kitti.bag");
    nh_.param<double>("/basic/map_match_hz", map_match_hz, 1.0);
    nh_.param<double>("/basic/visualize_voxel_size", voxel_res_, 1.0);
    /* keyframe */
    nh_.param<double>("/keyframe/keyframe_threshold", keyframe_dist_thr_, 1.0);
    nh_.param<int>("/keyframe/num_submap_keyframes", mm_config.num_submap_keyframes_, 5);
    /* match */
    nh_.param<double>("/match/scancontext_max_correspondence_distance",
                      mm_config.scancontext_max_correspondence_distance_,
                      15.0);
    nh_.param<double>("/match/quatro_nano_gicp_voxel_resolution", mm_config.voxel_res_, 0.3);
    /* nano */
    nh_.param<int>("/nano_gicp/thread_number", gc.nano_thread_number_, 0);
    nh_.param<double>("/nano_gicp/icp_score_threshold", gc.icp_score_thr_, 10.0);
    nh_.param<int>("/nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
    nh_.param<double>("/nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 0.01);
    nh_.param<int>("/nano_gicp/max_iter", gc.nano_max_iter_, 32);
    nh_.param<double>("/nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
    nh_.param<double>("/nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
    nh_.param<int>("/nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
    nh_.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);
    /* quatro */
    nh_.param<bool>("/quatro/enable", mm_config.enable_quatro_, false);
    nh_.param<bool>("/quatro/optimize_matching", qc.use_optimized_matching_, true);
    nh_.param<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_, 30.0);
    nh_.param<int>("/quatro/max_correspondences", qc.quatro_max_num_corres_, 200);
    nh_.param<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.02);
    nh_.param<double>("/quatro/fpfh_radius", qc.fpfh_radius_, 0.04);
    nh_.param<bool>("/quatro/estimating_scale", qc.estimat_scale_, false);
    nh_.param<double>("/quatro/noise_bound", qc.noise_bound_, 0.25);
    nh_.param<double>("/quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 0.25);
    nh_.param<double>("/quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thr_, 0.25);
    nh_.param<int>("/quatro/rotation/num_max_iter", qc.quatro_max_iter_, 50);

    // 按参数初始化全局匹配器（ScanContext + Quatro + NanoGICP）
    ////// Matching init
    ////// Matching init
    map_matcher_ = std::make_shared<MapMatcher>(mm_config);

    // 从 rosbag 加载离线地图并注册进 ScanContext
    ////// Load map
    ////// Load map
    loadMap(saved_map_path);

    ////// ROS things
    raw_odom_path_.header.frame_id = map_frame_;
    corrected_odom_path_.header.frame_id = map_frame_;
    // 发布者：原始/校正轨迹、实时位姿、匹配调试点云与离线地图
    // publishers
    // publishers
    odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/ori_path", 10, true);
    corrected_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>("/corrected_path", 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
    map_match_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_match", 10, true);
    realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
    saved_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/saved_map", 10, true);
    debug_src_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10);
    debug_dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10);
    debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10);
    // 订阅者：FAST-LIO 的 /Odometry 与 /cloud_registered，近似时间同步后进回调
    // subscribers
    // subscribers
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, "/Odometry", 10);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, "/cloud_registered", 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(boost::bind(&FastLioLocalizationScQn::odomPcdCallback, this, _1, _2));
    // 定时器最后创建：确保参数/地图/订阅全部就绪后才开始周期匹配
    // Timers at the end
    // Timers at the end
    match_timer_ = nh_.createTimer(ros::Duration(1 / map_match_hz), &FastLioLocalizationScQn::matchingTimerFunc, this);

    ROS_WARN("Main class, starting node...");
}

// 里程计 + 点云同步回调（FAST-LIO 输出）。职责：
//   1. 用累积校正 TF 计算实时校正位姿并发布（/pose_stamped、TF、当前点云）；
//   2. 首次数据无条件保存为初始键帧，之后按距离阈值决定是否保存新键帧。
void FastLioLocalizationScQn::odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
    PosePcd current_frame = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    //// 1. realtime pose = last TF * odom
    // 实时位姿 = 最近一次累积校正 TF * 里程计位姿（把全局匹配的校正量实时叠加）
    current_frame.pose_corrected_eig_ = last_corrected_TF_ * current_frame.pose_eig_;
    current_frame.pose_corrected_eig_ = last_corrected_TF_ * current_frame.pose_eig_;
    geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(current_frame.pose_corrected_eig_, map_frame_);
    realtime_pose_pub_.publish(current_pose_stamped_);
    broadcaster_.sendTransform(tf::StampedTransform(poseEigToROSTf(current_frame.pose_corrected_eig_),
                                                    ros::Time::now(),
                                                    map_frame_,
                                                    "robot"));
    // pub current scan in corrected pose frame
    corrected_current_pcd_pub_.publish(pclToPclRos(transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_), map_frame_));

    // 初始化分支：无条件保存首帧作为匹配基准
    if (!is_initialized_) //// init only once
    if (!is_initialized_) //// init only once
    {
        // 1. save first keyframe
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            last_keyframe_ = current_frame;
        }
        current_keyframe_idx_++;
        //// 2. vis
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            updateOdomsAndPaths(current_frame);
        }
        is_initialized_ = true;
    }
    else
    {
        //// 1. check if keyframe
        if (checkIfKeyframe(current_frame, last_keyframe_))
        {
            // 2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                last_keyframe_ = current_frame;
            }
            current_keyframe_idx_++;
            //// 3. vis
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame);
            }
        }
    }
    return;
}

// 匹配定时器回调：对最新未处理键帧执行全局匹配（ScanContext 检索 +
// Quatro/NanoGICP 配准）。匹配有效时把相对变换累积进 last_corrected_TF_
// 并修正校正轨迹，最后发布轨迹、匹配连线与调试点云。
void FastLioLocalizationScQn::matchingTimerFunc(const ros::TimerEvent &event)
{
    if (!is_initialized_)
    {
        return;
    }

    // 在锁内拷贝最新键帧并标记为已处理，避免重复匹配同一帧
    //// 1. copy not processed keyframes
    //// 1. copy not processed keyframes
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    PosePcd last_keyframe_copy;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        last_keyframe_copy = last_keyframe_;
        last_keyframe_.processed_ = true;
    }
    // 跳过初始帧（无匹配基准）与已处理帧
    if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_)
    if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_)
    {
        return; // already processed or initial keyframe
    }

    //// 2. detect match and calculate TF
    // from last_keyframe_copy keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
    int closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(last_keyframe_copy, saved_map_from_bag_);
    if (closest_keyframe_idx < 0)
    {
        return; // if no matched candidate
    }
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(last_keyframe_copy,
                                                                           saved_map_from_bag_,
                                                                           closest_keyframe_idx);

    //// 3. handle corrected results
    // 匹配通过有效性判定：更新累积校正 TF，并修正该键帧的校正位姿
    if (reg_output.is_valid_) // TF the pose with the result of match
    if (reg_output.is_valid_) // TF the pose with the result of match
    {
        ROS_INFO("\033[1;32mMap matching accepted. Score: %.3f\033[0m", reg_output.score_);
        // 累积校正：匹配得到的相对变换左乘到既有校正之上（校正量随时间叠加）
        last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
        last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
        Eigen::Matrix4d TFed_pose = reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;
        // correct poses in vis data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_.points[last_keyframe_copy.idx_] = pcl::PointXYZ(TFed_pose(0, 3), TFed_pose(1, 3), TFed_pose(2, 3));
            corrected_odom_path_.poses[last_keyframe_copy.idx_] = poseEigToPoseStamped(TFed_pose, map_frame_);
        }
        // map matches
        matched_pairs_xyz_.push_back({corrected_odoms_.points[last_keyframe_copy.idx_], raw_odoms_.points[last_keyframe_copy.idx_]}); // for vis
        map_match_pub_.publish(getMatchMarker(matched_pairs_xyz_));
    }
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();

    // 发布调试点云：源/目标/Quatro 粗对齐/GICP 精对齐
    debug_src_pub_.publish(pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
    debug_src_pub_.publish(pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
    debug_dst_pub_.publish(pclToPclRos(map_matcher_->getTargetCloud(), map_frame_));
    debug_coarse_aligned_pub_.publish(pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_));
    debug_fine_aligned_pub_.publish(pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_));

    // 发布原始与校正后的轨迹（点云 + 路径）
    // publish odoms and paths
    // publish odoms and paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_.publish(corrected_odom_path_);
    }
    odom_pub_.publish(pclToPclRos(raw_odoms_, map_frame_));
    path_pub_.publish(raw_odom_path_);
    // 离线地图点云只在有订阅者时发布一次，避免反复序列化大点云
    // publish saved map
    // publish saved map
    if (saved_map_vis_switch_ && saved_map_pub_.getNumSubscribers() > 0)
    {
        saved_map_pub_.publish(pclToPclRos(saved_map_pcd_, map_frame_));
        saved_map_vis_switch_ = false;
    }
    if (!saved_map_vis_switch_ && saved_map_pub_.getNumSubscribers() == 0)
    {
        saved_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    ROS_INFO("Matching: %.1fms, vis: %.1fms",
             duration_cast<microseconds>(t2_ - t1_).count() / 1e3,
             duration_cast<microseconds>(t3_ - t2_).count() / 1e3);
    return;
}

// 把键帧的原始/校正位姿追加进轨迹点云与路径容器（可视化数据，与键帧索引一一对应）
void FastLioLocalizationScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    raw_odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                                   pose_pcd_in.pose_eig_(1, 3),
                                   pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    raw_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

// 把"校正后位姿-原始位姿"点对转成 rviz 线段 Marker（type 5 为 LINE_LIST）
visualization_msgs::Marker FastLioLocalizationScQn::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs)
{
    visualization_msgs::Marker edges_;
    edges_.type = 5u;
    edges_.scale.x = 0.2f;
    edges_.header.frame_id = map_frame_;
    edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f;
    edges_.color.g = 1.0f;
    edges_.color.b = 1.0f;
    edges_.color.a = 1.0f;
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i)
    {
        geometry_msgs::Point p_, p2_;
        p_.x = match_xyz_pairs[i].first.x;
        p_.y = match_xyz_pairs[i].first.y;
        p_.z = match_xyz_pairs[i].first.z;
        p2_.x = match_xyz_pairs[i].second.x;
        p2_.y = match_xyz_pairs[i].second.y;
        p2_.z = match_xyz_pairs[i].second.z;
        edges_.points.push_back(p_);
        edges_.points.push_back(p2_);
    }
    return edges_;
}

// 平移距离超过阈值即视为新键帧（基于校正后位姿计算）
bool FastLioLocalizationScQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_dist_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}

// 从 rosbag 读取离线地图键帧（话题 /keyframe_pcd 与 /keyframe_pose），
// 构造 PosePcdReduced 列表，拼接可视化地图点云并注册进 ScanContext 数据库
void FastLioLocalizationScQn::loadMap(const std::string &saved_map_path)
{
    rosbag::Bag bag;
    bag.open(saved_map_path, rosbag::bagmode::Read);
    rosbag::View view1(bag, rosbag::TopicQuery("/keyframe_pcd"));
    rosbag::View view2(bag, rosbag::TopicQuery("/keyframe_pose"));
    std::vector<sensor_msgs::PointCloud2> load_pcd_vec;
    std::vector<geometry_msgs::PoseStamped> load_pose_vec;
    for (const rosbag::MessageInstance &pcd_msg : view1)
    {
        sensor_msgs::PointCloud2::ConstPtr pcd_msg_ptr = pcd_msg.instantiate<sensor_msgs::PointCloud2>();
        if (pcd_msg_ptr != nullptr)
        {
            load_pcd_vec.push_back(*pcd_msg_ptr);
        }
    }
    for (const rosbag::MessageInstance &pose_msg : view2)
    {
        geometry_msgs::PoseStamped::ConstPtr pose_msg_ptr = pose_msg.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msg_ptr != nullptr)
        {
            load_pose_vec.push_back(*pose_msg_ptr);
        }
    }
    // 键帧点云与位姿数量必须一致，否则说明 bag 文件有问题
    if (load_pcd_vec.size() != load_pose_vec.size())
    if (load_pcd_vec.size() != load_pose_vec.size())
    {
        ROS_ERROR("WRONG BAG FILE!!!!!");
    }
    // 逐帧构造 PosePcdReduced：拼接地图点云，并注册 ScanContext 用于回环候选检索
    for (size_t i = 0; i < load_pose_vec.size(); ++i)
    for (size_t i = 0; i < load_pose_vec.size(); ++i)
    {
        saved_map_from_bag_.push_back(PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));
        saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_, saved_map_from_bag_[i].pose_eig_);
        map_matcher_->updateScancontext(saved_map_from_bag_[i].pcd_); // note: update scan context for loop candidate detection
    }
    saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);
    bag.close();
    return;
}
