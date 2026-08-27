// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// ============================================================
// laserMapping.cpp —— FAST-LIO 主节点实现（上游开源算法）
// 本文件是 FAST-LIO 的核心：以误差状态卡尔曼滤波(ESKF, esekfom)
// 为状态估计器，融合 IMU 前向传播与 ikd-Tree 增量地图上的点到面
// 配准（ICP），输出连续雷达惯性里程计与局部地图。每帧主流程：
//   订阅 IMU/点云 -> 时间同步打包 -> IMU 传播与点云去畸变 ->
//   地图 FOV 分割 -> 特征降采样 -> ikd-Tree 最近面搜索 ->
//   迭代 ESKF 更新（点到面残差）-> 地图增量更新 -> 发布里程计/点云。
// 工作区内作为机器人前端连续里程计使用（对应 /Odometry_fast_lio）。
// ============================================================
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <cstdint>
#include <limits>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <livox_interfaces/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

// ---- 全局统计变量：记录 ikd-Tree 增删/搜索耗时、配准与求解耗时等 ----
/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false, publish_odom_imu_tf_en = false;
bool is_first_lidar = true;

// Low-overhead, one-second aggregate diagnostics. Sensor callbacks only update
// counters and maxima; no per-message console or disk I/O is performed.
struct FastlioInputDiagnostics
{
    using Clock = std::chrono::steady_clock;

    bool enabled = true;
    double imu_gap_warn_s = 0.02;
    double arrival_gap_warn_s = 0.02;
    double slow_lidar_warn_s = 0.05;
    double slow_timer_warn_s = 0.05;
    double accel_axis_warn = 3.8;
    int min_imu_per_scan = 10;

    Clock::time_point window_start = Clock::now();
    Clock::time_point last_imu_arrival{};
    Clock::time_point last_lidar_arrival{};
    bool have_imu = false;
    bool have_lidar = false;
    double last_imu_stamp = 0.0;
    double last_lidar_stamp = 0.0;

    uint64_t imu_count = 0;
    uint64_t imu_gap_count = 0;
    uint64_t imu_backward_count = 0;
    uint64_t imu_arrival_gap_count = 0;
    double imu_last_dt_s = 0.0;
    double imu_max_dt_s = 0.0;
    double imu_max_arrival_dt_s = 0.0;
    double imu_max_lock_wait_s = 0.0;
    double worst_imu_gap_dt_s = 0.0;
    double worst_imu_gap_stamp = 0.0;
    size_t imu_buffer_max = 0;
    double imu_acc_axis_max = 0.0;
    double imu_acc_norm_max = 0.0;
    double imu_gyr_axis_max = 0.0;
    double imu_gyr_norm_max = 0.0;
    double imu_peak_stamp = 0.0;
    uint64_t imu_acc_over_count = 0;
    uint64_t imu_acc_over_streak = 0;
    uint64_t imu_acc_over_streak_max = 0;

    uint64_t lidar_count = 0;
    uint64_t lidar_gap_count = 0;
    uint64_t lidar_slow_count = 0;
    double lidar_max_dt_s = 0.0;
    double lidar_max_arrival_dt_s = 0.0;
    double lidar_max_lock_wait_s = 0.0;
    double lidar_max_preprocess_s = 0.0;
    double lidar_max_callback_s = 0.0;
    size_t lidar_buffer_max = 0;

    uint64_t sync_count = 0;
    uint64_t sync_low_imu_count = 0;
    size_t sync_imu_min = std::numeric_limits<size_t>::max();
    size_t sync_imu_max = 0;
    double sync_max_imu_dt_s = 0.0;
    double sync_max_span_s = 0.0;

    uint64_t timer_count = 0;
    uint64_t timer_slow_count = 0;
    double timer_total_s = 0.0;
    double timer_max_s = 0.0;

    uint64_t estimator_count = 0;
    size_t effective_min = std::numeric_limits<size_t>::max();
    size_t effective_max = 0;
    double effective_ratio_min = std::numeric_limits<double>::infinity();
    double residual_last = 0.0;
    double residual_max = 0.0;
    double update_dpos_max = 0.0;
    double update_drot_max_deg = 0.0;
    double update_dvel_max = 0.0;
    double frame_dpos_max = 0.0;
    double frame_drot_max_deg = 0.0;
    double state_vel_norm_max = 0.0;
    V3D state_pos = V3D::Zero();
    V3D state_vel = V3D::Zero();
    V3D state_ba = V3D::Zero();
    V3D state_bg = V3D::Zero();
    V3D state_grav = V3D::Zero();
    V3D previous_state_pos = V3D::Zero();
    M3D previous_state_rot = M3D::Identity();
    bool have_previous_state = false;

    static double seconds(const Clock::time_point &newer,
                          const Clock::time_point &older)
    {
        return std::chrono::duration<double>(newer - older).count();
    }

    void observeImu(double stamp, const Clock::time_point &arrival,
                    double lock_wait_s, size_t buffer_size,
                    const sensor_msgs::msg::Imu &imu)
    {
        if (!enabled)
            return;
        ++imu_count;
        if (have_imu)
        {
            const double header_dt = stamp - last_imu_stamp;
            const double arrival_dt = seconds(arrival, last_imu_arrival);
            imu_last_dt_s = header_dt;
            imu_max_dt_s = std::max(imu_max_dt_s, header_dt);
            imu_max_arrival_dt_s = std::max(imu_max_arrival_dt_s, arrival_dt);
            if (header_dt <= 0.0)
                ++imu_backward_count;
            else if (header_dt > imu_gap_warn_s)
            {
                ++imu_gap_count;
                if (header_dt >= worst_imu_gap_dt_s)
                {
                    worst_imu_gap_dt_s = header_dt;
                    worst_imu_gap_stamp = stamp;
                }
            }
            if (arrival_dt > arrival_gap_warn_s)
                ++imu_arrival_gap_count;
        }
        have_imu = true;
        last_imu_stamp = stamp;
        last_imu_arrival = arrival;
        imu_max_lock_wait_s = std::max(imu_max_lock_wait_s, lock_wait_s);
        imu_buffer_max = std::max(imu_buffer_max, buffer_size);

        const auto &acc = imu.linear_acceleration;
        const auto &gyr = imu.angular_velocity;
        const double acc_axis = std::max(
            std::abs(acc.x), std::max(std::abs(acc.y), std::abs(acc.z)));
        const double acc_norm = std::sqrt(
            acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
        const double gyr_axis = std::max(
            std::abs(gyr.x), std::max(std::abs(gyr.y), std::abs(gyr.z)));
        const double gyr_norm = std::sqrt(
            gyr.x * gyr.x + gyr.y * gyr.y + gyr.z * gyr.z);
        if (acc_axis >= imu_acc_axis_max)
        {
            imu_acc_axis_max = acc_axis;
            imu_peak_stamp = stamp;
        }
        imu_acc_norm_max = std::max(imu_acc_norm_max, acc_norm);
        imu_gyr_axis_max = std::max(imu_gyr_axis_max, gyr_axis);
        imu_gyr_norm_max = std::max(imu_gyr_norm_max, gyr_norm);
        if (acc_axis >= accel_axis_warn)
        {
            ++imu_acc_over_count;
            ++imu_acc_over_streak;
            imu_acc_over_streak_max = std::max(
                imu_acc_over_streak_max, imu_acc_over_streak);
        }
        else
        {
            imu_acc_over_streak = 0;
        }
    }

    void observeLidar(double stamp, const Clock::time_point &arrival,
                      double lock_wait_s, double preprocess_s,
                      double callback_s, size_t buffer_size)
    {
        if (!enabled)
            return;
        ++lidar_count;
        if (have_lidar)
        {
            const double header_dt = stamp - last_lidar_stamp;
            const double arrival_dt = seconds(arrival, last_lidar_arrival);
            lidar_max_dt_s = std::max(lidar_max_dt_s, header_dt);
            lidar_max_arrival_dt_s = std::max(lidar_max_arrival_dt_s, arrival_dt);
            if (header_dt <= 0.0 || header_dt > 0.2)
                ++lidar_gap_count;
        }
        have_lidar = true;
        last_lidar_stamp = stamp;
        last_lidar_arrival = arrival;
        lidar_max_lock_wait_s = std::max(lidar_max_lock_wait_s, lock_wait_s);
        lidar_max_preprocess_s = std::max(lidar_max_preprocess_s, preprocess_s);
        lidar_max_callback_s = std::max(lidar_max_callback_s, callback_s);
        lidar_buffer_max = std::max(lidar_buffer_max, buffer_size);
        if (callback_s > slow_lidar_warn_s)
            ++lidar_slow_count;
    }

    void observeSync(const MeasureGroup &meas)
    {
        if (!enabled)
            return;
        ++sync_count;
        const size_t count = meas.imu.size();
        sync_imu_min = std::min(sync_imu_min, count);
        sync_imu_max = std::max(sync_imu_max, count);
        if (count < static_cast<size_t>(std::max(0, min_imu_per_scan)))
            ++sync_low_imu_count;
        if (count >= 2)
        {
            double previous = get_time_sec(meas.imu.front()->header.stamp);
            const double first = previous;
            double last = previous;
            for (size_t i = 1; i < count; ++i)
            {
                last = get_time_sec(meas.imu[i]->header.stamp);
                sync_max_imu_dt_s = std::max(sync_max_imu_dt_s, last - previous);
                previous = last;
            }
            sync_max_span_s = std::max(sync_max_span_s, last - first);
        }
    }

    void observeTimer(double duration_s)
    {
        if (!enabled)
            return;
        ++timer_count;
        timer_total_s += duration_s;
        timer_max_s = std::max(timer_max_s, duration_s);
        if (duration_s > slow_timer_warn_s)
            ++timer_slow_count;
    }

    void observeEstimator(const state_ikfom &predicted,
                          const state_ikfom &candidate,
                          const state_ikfom &output,
                          size_t downsampled, size_t effective,
                          double residual)
    {
        if (!enabled)
            return;
        ++estimator_count;
        effective_min = std::min(effective_min, effective);
        effective_max = std::max(effective_max, effective);
        if (downsampled > 0)
        {
            effective_ratio_min = std::min(
                effective_ratio_min,
                static_cast<double>(effective) / downsampled);
        }
        residual_last = residual;
        residual_max = std::max(residual_max, residual);

        update_dpos_max = std::max(
            update_dpos_max, (candidate.pos - predicted.pos).norm());
        update_dvel_max = std::max(
            update_dvel_max, (candidate.vel - predicted.vel).norm());
        update_drot_max_deg = std::max(
            update_drot_max_deg,
            Log((predicted.rot.conjugate() * candidate.rot).toRotationMatrix()).norm() *
                180.0 / PI_M);

        if (have_previous_state)
        {
            frame_dpos_max = std::max(
                frame_dpos_max, (output.pos - previous_state_pos).norm());
            const M3D frame_delta_rot = previous_state_rot.transpose() *
                output.rot.toRotationMatrix();
            frame_drot_max_deg = std::max(
                frame_drot_max_deg,
                Log(frame_delta_rot).norm() * 180.0 / PI_M);
        }
        previous_state_pos = output.pos;
        previous_state_rot = output.rot.toRotationMatrix();
        have_previous_state = true;

        state_pos = output.pos;
        state_vel = output.vel;
        state_ba = output.ba;
        state_bg = output.bg;
        state_grav << output.grav[0], output.grav[1], output.grav[2];
        state_vel_norm_max = std::max(
            state_vel_norm_max, output.vel.norm());
    }

    void resetWindow(const Clock::time_point &now)
    {
        window_start = now;
        imu_count = imu_gap_count = imu_backward_count = 0;
        imu_arrival_gap_count = 0;
        imu_last_dt_s = imu_max_dt_s = imu_max_arrival_dt_s = 0.0;
        imu_max_lock_wait_s = 0.0;
        worst_imu_gap_dt_s = 0.0;
        worst_imu_gap_stamp = 0.0;
        imu_buffer_max = 0;
        imu_acc_axis_max = imu_acc_norm_max = 0.0;
        imu_gyr_axis_max = imu_gyr_norm_max = 0.0;
        imu_peak_stamp = 0.0;
        imu_acc_over_count = 0;
        imu_acc_over_streak_max = imu_acc_over_streak;
        lidar_count = lidar_gap_count = lidar_slow_count = 0;
        lidar_max_dt_s = lidar_max_arrival_dt_s = 0.0;
        lidar_max_lock_wait_s = lidar_max_preprocess_s = 0.0;
        lidar_max_callback_s = 0.0;
        lidar_buffer_max = 0;
        sync_count = sync_low_imu_count = 0;
        sync_imu_min = std::numeric_limits<size_t>::max();
        sync_imu_max = 0;
        sync_max_imu_dt_s = sync_max_span_s = 0.0;
        timer_count = timer_slow_count = 0;
        timer_total_s = timer_max_s = 0.0;
        estimator_count = 0;
        effective_min = std::numeric_limits<size_t>::max();
        effective_max = 0;
        effective_ratio_min = std::numeric_limits<double>::infinity();
        residual_last = residual_max = 0.0;
        update_dpos_max = update_drot_max_deg = update_dvel_max = 0.0;
        frame_dpos_max = frame_drot_max_deg = 0.0;
        state_vel_norm_max = 0.0;
    }
};

FastlioInputDiagnostics input_diagnostics;

class ScopedFastlioTimer
{
public:
    ScopedFastlioTimer() : start_(FastlioInputDiagnostics::Clock::now()) {}
    ~ScopedFastlioTimer()
    {
        input_diagnostics.observeTimer(FastlioInputDiagnostics::seconds(
            FastlioInputDiagnostics::Clock::now(), start_));
    }

private:
    FastlioInputDiagnostics::Clock::time_point start_;
};

vector<vector<int>> pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

// ---- ESKF 输入输出：Measurements 测量组、卡尔曼滤波器 kf、当前状态 state_point ----
/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

// 信号处理：收到 SIGINT 时置退出标志、唤醒等待线程并关闭 ROS 事件循环。
void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

// 将当前 ESKF 状态（姿态角、位置、速度、零偏、重力）写入日志文件。
inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

// 将机体系点云坐标经外参 + 位姿变换到世界系（使用指定状态 s，供迭代更新中使用）。
void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// 将机体系点变换到世界系（使用当前全局状态 state_point 的位姿与外参）。
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

// 收集 ikd-Tree 因局部地图滑动而被删除的历史点（缓存回收，供调试/复用）。
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
// 局部地图 FOV 分割：以当前位置为中心维护固定尺寸的局部立方体地图，
// 当机器人靠近立方体边缘时平移地图窗口，并从 ikd-Tree 中删除移出
// 区域的历史点，保证地图规模与搜索开销可控。
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

// 标准 PointCloud2 点云回调：加锁缓存点云与时间戳，调用预处理
// （特征提取/抽稀）后存入待同步队列，并唤醒主处理线程。
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    const auto callback_start = FastlioInputDiagnostics::Clock::now();
    const auto lock_start = callback_start;
    mtx_buffer.lock();
    const auto lock_acquired = FastlioInputDiagnostics::Clock::now();
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    const double preprocess_s = omp_get_wtime() - preprocess_start_time;
    s_plot11[scan_count] = preprocess_s;
    const size_t buffer_size = lidar_buffer.size();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    const auto callback_end = FastlioInputDiagnostics::Clock::now();
    input_diagnostics.observeLidar(
        cur_time, callback_start,
        FastlioInputDiagnostics::seconds(lock_acquired, lock_start),
        preprocess_s,
        FastlioInputDiagnostics::seconds(callback_end, callback_start),
        buffer_size);
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
// Livox 自定义消息回调：与 standard_pcl_cbk 相同的数据流，另含
// IMU/雷达时间戳同步逻辑（time_sync_en 使能时估计时间偏差）。
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
// void livox_pcl_cbk(const livox_interfaces::msg::CustomMsg::UniquePtr msg)
{
    const auto callback_start = FastlioInputDiagnostics::Clock::now();
    const auto lock_start = callback_start;
    mtx_buffer.lock();
    const auto lock_acquired = FastlioInputDiagnostics::Clock::now();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar)
    {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    const double preprocess_s = omp_get_wtime() - preprocess_start_time;
    s_plot11[scan_count] = preprocess_s;
    const size_t buffer_size = lidar_buffer.size();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    const auto callback_end = FastlioInputDiagnostics::Clock::now();
    input_diagnostics.observeLidar(
        cur_time, callback_start,
        FastlioInputDiagnostics::seconds(lock_acquired, lock_start),
        preprocess_s,
        FastlioInputDiagnostics::seconds(callback_end, callback_start),
        buffer_size);
}

// IMU 回调：按时间同步偏差修正时间戳后压入 IMU 缓存队列，供帧同步与预测使用。
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    const auto callback_arrival = FastlioInputDiagnostics::Clock::now();
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    const double input_timestamp = get_time_sec(msg_in->header.stamp);
    msg->header.stamp = get_ros_time(input_timestamp - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    const auto lock_start = FastlioInputDiagnostics::Clock::now();
    mtx_buffer.lock();
    const auto lock_acquired = FastlioInputDiagnostics::Clock::now();

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    const size_t buffer_size = imu_buffer.size();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    input_diagnostics.observeImu(
        input_timestamp, callback_arrival,
        FastlioInputDiagnostics::seconds(lock_acquired, lock_start),
        buffer_size, *msg_in);
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
// 时间同步打包：将队首雷达帧与覆盖该帧时间范围的 IMU 序列组合为一个
// MeasureGroup；当 IMU 时间戳尚未覆盖到帧尾时返回 false 等待更多数据。
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    input_diagnostics.observeSync(meas);
    return true;
}

int process_increments = 0;
// 地图增量更新：将本帧配准后的特征点变换到世界系，经体素网格最近点
// 去重判定后，把新增点插入 ikd-Tree（含无需降采样的点）。
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min)
            {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
// 发布世界系点云帧（可选，dense_pub_en 决定稠密/稀疏），并支持按
// pcd_save_interval 间隔把累积点云保存为 PCD 文件。
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    
}

// 发布机体(IMU)坐标系下的去畸变点云帧，便于在机器人本体视角查看。
void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "imu_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

// 发布本帧实际参与配准（有效）的特征点，便于可视化调试配准质量。
void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

// 将本帧点云累积进 pcl_wait_pub 并发布增量地图点云（map_pub_en 使能时）。
void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "odom";
    // pubLaserCloudMap->publish(laserCloudMap);
}

// 将累积的地图点云写为 PCD 文件（由 map_save 服务回调触发）。
void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template <typename T>
void set_posestamp(T &out, const state_ikfom &state)
{
    out.pose.position.x = state.pos(0);
    out.pose.position.y = state.pos(1);
    out.pose.position.z = state.pos(2);
    out.pose.orientation.x = state.rot.coeffs()[0];
    out.pose.orientation.y = state.rot.coeffs()[1];
    out.pose.orientation.z = state.rot.coeffs()[2];
    out.pose.orientation.w = state.rot.coeffs()[3];
}

// 发布里程计消息（位姿 + 协方差），并可选广播 odom -> imu_link 的 TF 变换。
void publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br,
    const state_ikfom &published_state,
    bool localization_valid)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "imu_link";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose, published_state);
    odomAftMapped.twist.twist.linear.x = published_state.vel.x();
    odomAftMapped.twist.twist.linear.y = published_state.vel.y();
    odomAftMapped.twist.twist.linear.z = published_state.vel.z();
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }
    if (!localization_valid)
    {
        for (int i = 0; i < 6; ++i)
            odomAftMapped.pose.covariance[i * 6 + i] = 1e6;
    }
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "imu_link";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    if (publish_odom_imu_tf_en && localization_valid)
    {
        tf_br->sendTransform(trans);
    }
}

// 发布运动轨迹路径，每 10 帧添加一个位姿点，避免路径过长导致 RViz 卡顿。
void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose, state_point);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

// ESKF 迭代更新的量测模型：为每个降采样特征点在 ikd-Tree 中搜索最近
// 邻域并拟合平面，计算点到面距离残差与雅可比矩阵 H，供迭代卡尔曼更新使用；
// 本函数在每次迭代中由 esekf 回调执行。
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); // 23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

// ROS 2 节点封装：声明并读取全部 yaml 参数，创建话题订阅/发布、
// 周期定时器与地图保存服务；每帧主处理在 timer_callback 中执行。
class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<bool>("publish.odom_imu_tf_en", false);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<bool>("diagnostics.enable", true);
        this->declare_parameter<double>("diagnostics.report_period_s", 1.0);
        this->declare_parameter<double>("diagnostics.imu_gap_warn_s", 0.02);
        this->declare_parameter<double>("diagnostics.arrival_gap_warn_s", 0.02);
        this->declare_parameter<double>("diagnostics.slow_lidar_warn_s", 0.05);
        this->declare_parameter<double>("diagnostics.slow_timer_warn_s", 0.05);
        this->declare_parameter<double>("diagnostics.accel_axis_warn", 3.8);
        this->declare_parameter<int>("diagnostics.min_imu_per_scan", 10);
        this->declare_parameter<bool>("robustness.enable", true);
        this->declare_parameter<double>("robustness.imu_accel_saturation_threshold", 3.9);
        this->declare_parameter<double>("robustness.imu_saturation_noise_scale", 100.0);
        this->declare_parameter<double>("robustness.imu_saturation_ratio_warn", 0.05);
        this->declare_parameter<int>("robustness.imu_saturation_streak_warn", 3);
        this->declare_parameter<double>("robustness.min_effective_ratio", 0.30);
        this->declare_parameter<int>("robustness.min_effective_points", 50);
        this->declare_parameter<double>("robustness.max_mean_residual", 0.05);
        this->declare_parameter<double>("robustness.critical_effective_ratio", 0.15);
        this->declare_parameter<double>("robustness.critical_mean_residual", 0.08);
        this->declare_parameter<double>("robustness.max_update_translation", 0.15);
        this->declare_parameter<double>("robustness.max_update_rotation_deg", 2.0);
        this->declare_parameter<double>("robustness.max_update_velocity", 1.0);
        this->declare_parameter<double>("robustness.critical_update_translation", 0.35);
        this->declare_parameter<double>("robustness.critical_update_rotation_deg", 5.0);
        this->declare_parameter<double>("robustness.critical_update_velocity", 1.5);
        this->declare_parameter<int>("robustness.degraded_enter_frames", 3);
        this->declare_parameter<int>("robustness.recover_enter_frames", 10);
        this->declare_parameter<int>("robustness.recover_normal_frames", 10);
        this->declare_parameter<double>("robustness.max_degraded_duration", 3.0);
        this->declare_parameter<int>("robustness.zero_effective_lost_frames", 5);
        this->declare_parameter<double>("robustness.max_imu_dt", 0.02);
        this->declare_parameter<double>("robustness.imu_gap_noise_scale", 100.0);
        this->declare_parameter<double>("robustness.max_propagation_translation", 0.20);
        this->declare_parameter<double>("robustness.max_propagation_velocity", 2.0);
        this->declare_parameter<bool>("robustness.lost_reinit_enable", true);
        this->declare_parameter<int>("robustness.lost_reinit_frames", 20);
        this->declare_parameter<double>("robustness.lost_reinit_cooldown", 5.0);
        this->declare_parameter<int>("robustness.recovery_bootstrap_frames", 3);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<bool>("publish.odom_imu_tf_en", publish_odom_imu_tf_en, false);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic, "/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner", filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min, 0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min, 0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<bool>("diagnostics.enable", input_diagnostics.enabled, true);
        this->get_parameter_or<double>("diagnostics.imu_gap_warn_s", input_diagnostics.imu_gap_warn_s, 0.02);
        this->get_parameter_or<double>("diagnostics.arrival_gap_warn_s", input_diagnostics.arrival_gap_warn_s, 0.02);
        this->get_parameter_or<double>("diagnostics.slow_lidar_warn_s", input_diagnostics.slow_lidar_warn_s, 0.05);
        this->get_parameter_or<double>("diagnostics.slow_timer_warn_s", input_diagnostics.slow_timer_warn_s, 0.05);
        this->get_parameter_or<double>("diagnostics.accel_axis_warn", input_diagnostics.accel_axis_warn, 3.8);
        this->get_parameter_or<int>("diagnostics.min_imu_per_scan", input_diagnostics.min_imu_per_scan, 10);
        this->get_parameter_or<bool>("robustness.enable", robustness_enable_, true);
        this->get_parameter_or<double>("robustness.imu_accel_saturation_threshold", imu_accel_saturation_threshold_, 3.9);
        this->get_parameter_or<double>("robustness.imu_saturation_noise_scale", imu_saturation_noise_scale_, 100.0);
        this->get_parameter_or<double>("robustness.imu_saturation_ratio_warn", imu_saturation_ratio_warn_, 0.05);
        this->get_parameter_or<int>("robustness.imu_saturation_streak_warn", imu_saturation_streak_warn_, 3);
        this->get_parameter_or<double>("robustness.min_effective_ratio", min_effective_ratio_, 0.30);
        this->get_parameter_or<int>("robustness.min_effective_points", min_effective_points_, 50);
        this->get_parameter_or<double>("robustness.max_mean_residual", max_mean_residual_, 0.05);
        this->get_parameter_or<double>("robustness.critical_effective_ratio", critical_effective_ratio_, 0.15);
        this->get_parameter_or<double>("robustness.critical_mean_residual", critical_mean_residual_, 0.08);
        this->get_parameter_or<double>("robustness.max_update_translation", max_update_translation_, 0.15);
        this->get_parameter_or<double>("robustness.max_update_rotation_deg", max_update_rotation_deg_, 2.0);
        this->get_parameter_or<double>("robustness.max_update_velocity", max_update_velocity_, 1.0);
        this->get_parameter_or<double>("robustness.critical_update_translation", critical_update_translation_, 0.35);
        this->get_parameter_or<double>("robustness.critical_update_rotation_deg", critical_update_rotation_deg_, 5.0);
        this->get_parameter_or<double>("robustness.critical_update_velocity", critical_update_velocity_, 1.5);
        this->get_parameter_or<int>("robustness.degraded_enter_frames", degraded_enter_frames_, 3);
        this->get_parameter_or<int>("robustness.recover_enter_frames", recover_enter_frames_, 10);
        this->get_parameter_or<int>("robustness.recover_normal_frames", recover_normal_frames_, 10);
        this->get_parameter_or<double>("robustness.max_degraded_duration", max_degraded_duration_, 3.0);
        this->get_parameter_or<int>("robustness.zero_effective_lost_frames", zero_effective_lost_frames_, 5);
        this->get_parameter_or<double>("robustness.max_imu_dt", max_imu_dt_, 0.02);
        this->get_parameter_or<double>("robustness.imu_gap_noise_scale", imu_gap_noise_scale_, 100.0);
        this->get_parameter_or<double>("robustness.max_propagation_translation", max_propagation_translation_, 0.20);
        this->get_parameter_or<double>("robustness.max_propagation_velocity", max_propagation_velocity_, 2.0);
        this->get_parameter_or<bool>("robustness.lost_reinit_enable", lost_reinit_enable_, true);
        this->get_parameter_or<int>("robustness.lost_reinit_frames", lost_reinit_frames_, 20);
        this->get_parameter_or<double>("robustness.lost_reinit_cooldown", lost_reinit_cooldown_, 5.0);
        this->get_parameter_or<int>("robustness.recovery_bootstrap_frames", recovery_bootstrap_frames_, 3);
        lost_reinit_frames_ = std::max(1, lost_reinit_frames_);
        lost_reinit_cooldown_ = std::max(0.0, lost_reinit_cooldown_);
        recovery_bootstrap_frames_ = std::max(0, recovery_bootstrap_frames_);
        double diagnostics_report_period_s = 1.0;
        this->get_parameter_or<double>("diagnostics.report_period_s", diagnostics_report_period_s, 1.0);
        diagnostics_report_period_s = std::max(0.2, diagnostics_report_period_s);
        input_diagnostics.resetWindow(FastlioInputDiagnostics::Clock::now());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "odom";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu->set_saturation_protection(
            imu_accel_saturation_threshold_,
            robustness_enable_ ? imu_saturation_noise_scale_ : 1.0);
        p_imu->set_timing_protection(
            max_imu_dt_, robustness_enable_ ? imu_gap_noise_scale_ : 1.0);

        RCLCPP_INFO(
            this->get_logger(),
            "FAST-LIO robustness %s: imu_sat=%.3f noise_scale=%.1f "
            "effective_ratio>=%.2f effective_points>=%d residual<=%.3f",
            robustness_enable_ ? "enabled" : "disabled",
            imu_accel_saturation_threshold_, imu_saturation_noise_scale_,
            min_effective_ratio_, min_effective_points_, max_mean_residual_);

        fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
            // sub_pcl_livox_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_1", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body_1", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected_1", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map_1", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_loc", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path_1", 20);
        auto validity_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        validity_qos.reliable().transient_local();
        pubLocalizationValid_ = this->create_publisher<std_msgs::msg::Bool>(
            "/fastlio/localization_valid", validity_qos);
        publish_localization_valid(false);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));

        if (input_diagnostics.enabled)
        {
            diagnostics_timer_ = create_wall_timer(
                std::chrono::duration<double>(diagnostics_report_period_s),
                std::bind(&LaserMappingNode::diagnostics_report_callback, this));
            RCLCPP_INFO(
                this->get_logger(),
                "FAST-LIO input diagnostics enabled: report=%.2fs imu_gap=%.3fs "
                "slow_lidar=%.3fs slow_timer=%.3fs",
                diagnostics_report_period_s, input_diagnostics.imu_gap_warn_s,
                input_diagnostics.slow_lidar_warn_s,
                input_diagnostics.slow_timer_warn_s);
        }

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        fclose(fp);
    }

private:
    enum class LocalizationHealth
    {
        NORMAL,
        DEGRADED,
        RECOVERING,
        LOST
    };

    const char *health_name(LocalizationHealth health) const
    {
        switch (health)
        {
        case LocalizationHealth::NORMAL:
            return "NORMAL";
        case LocalizationHealth::DEGRADED:
            return "DEGRADED";
        case LocalizationHealth::RECOVERING:
            return "RECOVERING";
        case LocalizationHealth::LOST:
            return "LOST";
        }
        return "UNKNOWN";
    }

    void publish_localization_valid(bool valid)
    {
        if (!pubLocalizationValid_ ||
            (localization_valid_published_ && last_localization_valid_ == valid))
            return;
        std_msgs::msg::Bool msg;
        msg.data = valid;
        pubLocalizationValid_->publish(msg);
        last_localization_valid_ = valid;
        localization_valid_published_ = true;
    }

    void set_health(LocalizationHealth next, const char *reason)
    {
        if (next == localization_health_)
            return;
        const auto previous = localization_health_;
        localization_health_ = next;
        if (next == LocalizationHealth::DEGRADED)
            degraded_start_time_ = lidar_end_time;
        else if (next == LocalizationHealth::NORMAL)
            degraded_start_time_ = -1.0;
        if (next == LocalizationHealth::LOST)
            lost_scan_count_ = 0;
        publish_localization_valid(next == LocalizationHealth::NORMAL &&
                                   last_good_valid_);
        RCLCPP_WARN(
            get_logger(), "[FASTLIO_HEALTH] %s -> %s reason=%s",
            health_name(previous), health_name(next), reason);
    }

    void update_health(bool scan_bad, bool scan_critical, bool imu_stressed,
                       bool no_effective_points)
    {
        if (!robustness_enable_)
            return;

        if (scan_bad)
        {
            healthy_scan_count_ = 0;
            ++bad_scan_count_;
            zero_effective_count_ = no_effective_points
                ? zero_effective_count_ + 1 : 0;
            if (zero_effective_count_ >= zero_effective_lost_frames_)
            {
                set_health(LocalizationHealth::LOST, "no_effective_points");
                return;
            }
            // LOST is terminal for bad scans.  A bad/critical scan must never
            // promote it back to DEGRADED; only healthy scans may start
            // recovery.
            if (localization_health_ == LocalizationHealth::LOST)
                return;
            if (localization_health_ == LocalizationHealth::RECOVERING)
            {
                set_health(LocalizationHealth::LOST, "recovery_scan_bad");
                return;
            }
            if (localization_health_ == LocalizationHealth::RECOVERING ||
                scan_critical ||
                bad_scan_count_ >= (imu_stressed
                    ? std::max(1, degraded_enter_frames_ - 1)
                    : degraded_enter_frames_))
            {
                set_health(LocalizationHealth::DEGRADED,
                           scan_critical ? "critical_scan" : "scan_quality");
            }
            // Evaluate the current scan before applying the timeout.  A scan
            // that has recovered must be allowed to clear degradation even
            // when the previous bad interval was close to the time limit.
            if (localization_health_ == LocalizationHealth::DEGRADED &&
                degraded_start_time_ >= 0.0 &&
                lidar_end_time - degraded_start_time_ >= max_degraded_duration_)
            {
                set_health(LocalizationHealth::LOST, "degraded_timeout");
            }
            return;
        }

        bad_scan_count_ = 0;
        zero_effective_count_ = 0;
        if (localization_health_ == LocalizationHealth::NORMAL)
            return;

        ++healthy_scan_count_;
        if ((localization_health_ == LocalizationHealth::DEGRADED ||
             localization_health_ == LocalizationHealth::LOST) &&
            healthy_scan_count_ >= recover_enter_frames_)
        {
            healthy_scan_count_ = 0;
            set_health(LocalizationHealth::RECOVERING, "stable_scans");
        }
        else if (localization_health_ == LocalizationHealth::RECOVERING &&
                 healthy_scan_count_ >= recover_normal_frames_)
        {
            healthy_scan_count_ = 0;
            set_health(LocalizationHealth::NORMAL, "recovered");
        }
    }

    bool reinitialize_local_map(const state_ikfom &predicted_state)
    {
        if (!robustness_enable_ || !lost_reinit_enable_ ||
            localization_health_ != LocalizationHealth::LOST ||
            !last_good_valid_ || lost_scan_count_ < lost_reinit_frames_ ||
            feats_down_size < 5)
            return false;
        if (last_reinit_time_ >= 0.0 &&
            lidar_end_time - last_reinit_time_ < lost_reinit_cooldown_)
            return false;

        // Preserve the bounded IMU-propagated pose, bias and gravity so a
        // manually moving robot keeps odom continuity.  Rebuild the local map
        // around the current scan and reset velocity to prevent the previous
        // inertial runaway from continuing.
        state_ikfom recovery_state = predicted_state;
        recovery_state.vel.setZero();
        kf.change_x(recovery_state);
        kf.change_P(last_good_covariance_);
        state_point = recovery_state;
        pos_lid = state_point.pos +
            state_point.rot * state_point.offset_T_L_I;

        PointVector seed_points;
        seed_points.resize(feats_down_size);
        for (int i = 0; i < feats_down_size; ++i)
            pointBodyToWorld(&feats_down_body->points[i], &seed_points[i]);
        ikdtree.set_downsample_param(filter_size_map_min);
        ikdtree.Reset(seed_points);
        Localmap_Initialized = false;
        cub_needrm.clear();
        Nearest_Points.clear();
        pointSearchInd_surf.clear();

        last_reinit_time_ = lidar_end_time;
        ++local_map_reinit_count_;
        bad_scan_count_ = 0;
        healthy_scan_count_ = 0;
        zero_effective_count_ = 0;
        lost_scan_count_ = 0;
        recovery_bootstrap_remaining_ = recovery_bootstrap_frames_;
        set_health(LocalizationHealth::RECOVERING,
                   "local_map_reinitialized");
        RCLCPP_WARN(
            get_logger(),
            "[FASTLIO_RECOVERY] rebuilt local map with %zu points at "
            "anchor=(%.3f, %.3f, %.3f), attempt=%llu",
            seed_points.size(), recovery_state.pos.x(), recovery_state.pos.y(),
            recovery_state.pos.z(),
            static_cast<unsigned long long>(local_map_reinit_count_));
        return true;
    }

    // 主循环（定时触发，FAST-LIO 每帧主流程）：
    // 同步数据 -> IMU 传播/点云去畸变 -> 地图 FOV 分割 -> 特征降采样
    // -> ikd-Tree 初始化或最近面搜索 -> 迭代 ESKF 更新 -> 地图增量更新
    // -> 发布里程计/点云/路径；并在使能时记录各阶段耗时日志。
    void timer_callback()
    {
        ScopedFastlioTimer diagnostics_timer_guard;
        if (sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            const state_ikfom propagation_base_state = kf.get_x();
            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (0) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            state_ikfom predicted_state = state_point;
            auto predicted_covariance = kf.get_P();
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            const state_ikfom candidate_state = kf.get_x();

            const double effective_ratio = feats_down_size > 0
                ? static_cast<double>(effct_feat_num) / feats_down_size
                : 0.0;
            const double update_translation =
                (candidate_state.pos - predicted_state.pos).norm();
            const double update_velocity =
                (candidate_state.vel - predicted_state.vel).norm();
            const double update_rotation_deg =
                Log((predicted_state.rot.conjugate() * candidate_state.rot)
                        .toRotationMatrix()).norm() * 180.0 / PI_M;
            const bool finite_update = std::isfinite(effective_ratio) &&
                std::isfinite(res_mean_last) &&
                std::isfinite(update_translation) &&
                std::isfinite(update_velocity) &&
                std::isfinite(update_rotation_deg) &&
                candidate_state.pos.allFinite() && candidate_state.vel.allFinite();
            const bool scan_bad = robustness_enable_ && flg_EKF_inited &&
                (!finite_update ||
                 effct_feat_num < min_effective_points_ ||
                 effective_ratio < min_effective_ratio_ ||
                 res_mean_last > max_mean_residual_ ||
                 update_translation > max_update_translation_ ||
                 update_rotation_deg > max_update_rotation_deg_ ||
                 update_velocity > max_update_velocity_);
            const bool scan_critical = robustness_enable_ && flg_EKF_inited &&
                (!finite_update ||
                 effective_ratio < critical_effective_ratio_ ||
                 res_mean_last > critical_mean_residual_);
            const bool correction_critical = robustness_enable_ && flg_EKF_inited &&
                (!finite_update ||
                 update_translation > critical_update_translation_ ||
                 update_rotation_deg > critical_update_rotation_deg_ ||
                 update_velocity > critical_update_velocity_);
            const bool imu_stressed =
                p_imu->last_scan_saturation_ratio() >=
                    imu_saturation_ratio_warn_ ||
                static_cast<int>(p_imu->last_scan_saturation_streak()) >=
                    imu_saturation_streak_warn_ ||
                p_imu->last_scan_time_gap_count() > 0;
            if (p_imu->last_scan_saturated_samples() > 0)
                ++saturated_scan_count_window_;

            update_health(scan_bad, scan_critical, imu_stressed,
                          effct_feat_num == 0);
            if (localization_health_ == LocalizationHealth::LOST)
                ++lost_scan_count_;
            else
                lost_scan_count_ = 0;
            const bool local_map_reinitialized =
                reinitialize_local_map(predicted_state);
            // 软退化只冻结地图，仍接受幅度合理的激光校正来约束持续运动；
            // 只有匹配临界退化或校正量异常时才拒绝本次激光更新。
            const bool accept_lidar_update =
                !local_map_reinitialized &&
                localization_health_ != LocalizationHealth::LOST &&
                !scan_critical && !correction_critical;
            if (!accept_lidar_update && !local_map_reinitialized)
            {
                state_ikfom bounded_prediction = predicted_state;
                if (!bounded_prediction.pos.allFinite() ||
                    !bounded_prediction.vel.allFinite())
                    bounded_prediction = propagation_base_state;
                if (robustness_enable_ && flg_EKF_inited && last_good_valid_)
                {
                    V3D propagation_delta =
                        bounded_prediction.pos - propagation_base_state.pos;
                    const double propagation_distance = propagation_delta.norm();
                    if (propagation_distance > max_propagation_translation_)
                    {
                        bounded_prediction.pos = propagation_base_state.pos +
                            propagation_delta *
                            (max_propagation_translation_ / propagation_distance);
                    }
                    const double velocity_norm = bounded_prediction.vel.norm();
                    if (velocity_norm > max_propagation_velocity_)
                        bounded_prediction.vel *=
                            max_propagation_velocity_ / velocity_norm;
                }
                kf.change_x(bounded_prediction);
                kf.change_P(predicted_covariance);
                ++rejected_update_count_window_;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "[FASTLIO_HEALTH] reject lidar update: health=%s "
                    "effective=%d/%d ratio=%.3f residual=%.4f "
                    "dpos=%.3f drot=%.2fdeg dvel=%.3f imu_sat=%.1f%% "
                    "streak=%zu imu_gap=%zu max_dt=%.1fms "
                    "bounded_step=%.3f bounded_vel=%.3f",
                    health_name(localization_health_), effct_feat_num,
                    feats_down_size, effective_ratio, res_mean_last,
                    update_translation, update_rotation_deg, update_velocity,
                    100.0 * p_imu->last_scan_saturation_ratio(),
                    p_imu->last_scan_saturation_streak(),
                    p_imu->last_scan_time_gap_count(),
                    1000.0 * p_imu->last_scan_max_imu_dt(),
                    (bounded_prediction.pos - propagation_base_state.pos).norm(),
                    bounded_prediction.vel.norm());
            }
            state_point = kf.get_x();
            if (flg_EKF_inited && accept_lidar_update && !scan_bad &&
                localization_health_ == LocalizationHealth::NORMAL)
            {
                last_good_state_ = state_point;
                last_good_covariance_ = kf.get_P();
                last_good_valid_ = true;
                publish_localization_valid(true);
            }
            input_diagnostics.observeEstimator(
                predicted_state, candidate_state, state_point,
                static_cast<size_t>(std::max(0, feats_down_size)),
                static_cast<size_t>(std::max(0, effct_feat_num)),
                res_mean_last);
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            const bool localization_valid =
                localization_health_ == LocalizationHealth::NORMAL &&
                last_good_valid_;
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_,
                             state_point, localization_valid);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            const bool recovery_bootstrap = robustness_enable_ &&
                localization_health_ == LocalizationHealth::RECOVERING &&
                recovery_bootstrap_remaining_ > 0;
            const bool allow_map_increment = accept_lidar_update && !scan_bad &&
                (!robustness_enable_ ||
                 localization_health_ == LocalizationHealth::NORMAL ||
                 recovery_bootstrap);
            if (allow_map_increment)
            {
                map_incremental();
                if (recovery_bootstrap)
                    --recovery_bootstrap_remaining_;
            }
            else
            {
                ++skipped_map_count_window_;
            }
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (path_en)
                publish_path(pubPath_);
            if (scan_pub_en)
                publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en)
                publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    // 周期回调：定时发布增量地图点云（map_pub_en 使能时）。
    void map_publish_callback()
    {
        if (map_pub_en)
            publish_map(pubLaserCloudMap_);
    }

    // Report only aggregate counters once per period. This keeps the 200 Hz IMU
    // callback free of formatting, terminal output and disk flushes.
    void diagnostics_report_callback()
    {
        const auto now = FastlioInputDiagnostics::Clock::now();
        const double elapsed = std::max(
            1e-9, FastlioInputDiagnostics::seconds(
                      now, input_diagnostics.window_start));
        const double imu_rate = input_diagnostics.imu_count / elapsed;
        const double timer_mean_ms = input_diagnostics.timer_count > 0
            ? input_diagnostics.timer_total_s * 1000.0 /
                  input_diagnostics.timer_count
            : 0.0;
        const size_t sync_imu_min = input_diagnostics.sync_count > 0
            ? input_diagnostics.sync_imu_min
            : 0;
        const bool input_warning = input_diagnostics.imu_gap_count > 0 ||
            input_diagnostics.imu_backward_count > 0 ||
            input_diagnostics.imu_arrival_gap_count > 0 ||
            input_diagnostics.lidar_gap_count > 0;
        const bool execution_warning = input_diagnostics.lidar_slow_count > 0 ||
            input_diagnostics.timer_slow_count > 0 ||
            input_diagnostics.sync_low_imu_count > 0 ||
            input_diagnostics.sync_max_imu_dt_s >
                input_diagnostics.imu_gap_warn_s;

        if (input_warning)
        {
            RCLCPP_WARN(
                get_logger(),
                "[FASTLIO_INPUT_DIAG] period=%.3fs imu_count=%lu rate=%.2fHz "
                "header_dt_last=%.3fms header_dt_max=%.3fms gaps=%lu backward=%lu "
                "worst_gap_stamp=%.9f arrival_dt_max=%.3fms arrival_gaps=%lu "
                "lock_wait_max=%.3fms buffer_max=%zu | lidar_count=%lu "
                "header_dt_max=%.3fms arrival_dt_max=%.3fms gaps=%lu buffer_max=%zu",
                elapsed, static_cast<unsigned long>(input_diagnostics.imu_count),
                imu_rate, input_diagnostics.imu_last_dt_s * 1000.0,
                input_diagnostics.imu_max_dt_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.imu_gap_count),
                static_cast<unsigned long>(input_diagnostics.imu_backward_count),
                input_diagnostics.worst_imu_gap_stamp,
                input_diagnostics.imu_max_arrival_dt_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.imu_arrival_gap_count),
                input_diagnostics.imu_max_lock_wait_s * 1000.0,
                input_diagnostics.imu_buffer_max,
                static_cast<unsigned long>(input_diagnostics.lidar_count),
                input_diagnostics.lidar_max_dt_s * 1000.0,
                input_diagnostics.lidar_max_arrival_dt_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.lidar_gap_count),
                input_diagnostics.lidar_buffer_max);
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "[FASTLIO_INPUT_DIAG] period=%.3fs imu_count=%lu rate=%.2fHz "
                "header_dt_last=%.3fms header_dt_max=%.3fms gaps=0 backward=0 "
                "arrival_dt_max=%.3fms arrival_gaps=0 lock_wait_max=%.3fms "
                "buffer_max=%zu | lidar_count=%lu header_dt_max=%.3fms "
                "arrival_dt_max=%.3fms gaps=0 buffer_max=%zu",
                elapsed, static_cast<unsigned long>(input_diagnostics.imu_count),
                imu_rate, input_diagnostics.imu_last_dt_s * 1000.0,
                input_diagnostics.imu_max_dt_s * 1000.0,
                input_diagnostics.imu_max_arrival_dt_s * 1000.0,
                input_diagnostics.imu_max_lock_wait_s * 1000.0,
                input_diagnostics.imu_buffer_max,
                static_cast<unsigned long>(input_diagnostics.lidar_count),
                input_diagnostics.lidar_max_dt_s * 1000.0,
                input_diagnostics.lidar_max_arrival_dt_s * 1000.0,
                input_diagnostics.lidar_buffer_max);
        }

        if (execution_warning)
        {
            RCLCPP_WARN(
                get_logger(),
                "[FASTLIO_EXEC_DIAG] lidar_preprocess_max=%.3fms "
                "lidar_callback_max=%.3fms lidar_lock_max=%.3fms slow_lidar=%lu | "
                "sync_count=%lu imu_per_scan=%zu..%zu low_imu=%lu "
                "sync_imu_dt_max=%.3fms sync_span_max=%.3fms | "
                "timer_count=%lu timer_mean=%.3fms timer_max=%.3fms slow_timer=%lu",
                input_diagnostics.lidar_max_preprocess_s * 1000.0,
                input_diagnostics.lidar_max_callback_s * 1000.0,
                input_diagnostics.lidar_max_lock_wait_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.lidar_slow_count),
                static_cast<unsigned long>(input_diagnostics.sync_count),
                sync_imu_min, input_diagnostics.sync_imu_max,
                static_cast<unsigned long>(input_diagnostics.sync_low_imu_count),
                input_diagnostics.sync_max_imu_dt_s * 1000.0,
                input_diagnostics.sync_max_span_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.timer_count),
                timer_mean_ms, input_diagnostics.timer_max_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.timer_slow_count));
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "[FASTLIO_EXEC_DIAG] lidar_preprocess_max=%.3fms "
                "lidar_callback_max=%.3fms lidar_lock_max=%.3fms slow_lidar=0 | "
                "sync_count=%lu imu_per_scan=%zu..%zu low_imu=0 "
                "sync_imu_dt_max=%.3fms sync_span_max=%.3fms | "
                "timer_count=%lu timer_mean=%.3fms timer_max=%.3fms slow_timer=0",
                input_diagnostics.lidar_max_preprocess_s * 1000.0,
                input_diagnostics.lidar_max_callback_s * 1000.0,
                input_diagnostics.lidar_max_lock_wait_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.sync_count),
                sync_imu_min, input_diagnostics.sync_imu_max,
                input_diagnostics.sync_max_imu_dt_s * 1000.0,
                input_diagnostics.sync_max_span_s * 1000.0,
                static_cast<unsigned long>(input_diagnostics.timer_count),
                timer_mean_ms, input_diagnostics.timer_max_s * 1000.0);
        }

        RCLCPP_INFO(
            get_logger(),
            "[FASTLIO_IMU_VALUE_DIAG] acc_axis_max=%.6f acc_norm_max=%.6f "
            "gyr_axis_max=%.6f gyr_norm_max=%.6f acc_over_%.3f=%lu "
            "over_streak_max=%lu peak_stamp=%.9f",
            input_diagnostics.imu_acc_axis_max,
            input_diagnostics.imu_acc_norm_max,
            input_diagnostics.imu_gyr_axis_max,
            input_diagnostics.imu_gyr_norm_max,
            input_diagnostics.accel_axis_warn,
            static_cast<unsigned long>(input_diagnostics.imu_acc_over_count),
            static_cast<unsigned long>(input_diagnostics.imu_acc_over_streak_max),
            input_diagnostics.imu_peak_stamp);

        const size_t effective_min = input_diagnostics.estimator_count > 0
            ? input_diagnostics.effective_min : 0;
        const double effective_ratio_min =
            std::isfinite(input_diagnostics.effective_ratio_min)
                ? input_diagnostics.effective_ratio_min : 0.0;
        RCLCPP_INFO(
            get_logger(),
            "[FASTLIO_ESTIMATOR_DIAG] frames=%lu effective=%zu..%zu "
            "ratio_min=%.6f residual_last=%.6f residual_max=%.6f | "
            "pos=(%.6f,%.6f,%.6f) vel=(%.6f,%.6f,%.6f) "
            "vel_norm=%.6f vel_norm_max=%.6f | "
            "ba=(%.6f,%.6f,%.6f) bg=(%.6f,%.6f,%.6f) "
            "grav=(%.6f,%.6f,%.6f) | "
            "update_dpos_max=%.6f update_drot_max_deg=%.6f "
            "update_dvel_max=%.6f frame_dpos_max=%.6f "
            "frame_drot_max_deg=%.6f",
            static_cast<unsigned long>(input_diagnostics.estimator_count),
            effective_min, input_diagnostics.effective_max,
            effective_ratio_min, input_diagnostics.residual_last,
            input_diagnostics.residual_max,
            input_diagnostics.state_pos.x(), input_diagnostics.state_pos.y(),
            input_diagnostics.state_pos.z(), input_diagnostics.state_vel.x(),
            input_diagnostics.state_vel.y(), input_diagnostics.state_vel.z(),
            input_diagnostics.state_vel.norm(),
            input_diagnostics.state_vel_norm_max,
            input_diagnostics.state_ba.x(), input_diagnostics.state_ba.y(),
            input_diagnostics.state_ba.z(), input_diagnostics.state_bg.x(),
            input_diagnostics.state_bg.y(), input_diagnostics.state_bg.z(),
            input_diagnostics.state_grav.x(), input_diagnostics.state_grav.y(),
            input_diagnostics.state_grav.z(),
            input_diagnostics.update_dpos_max,
            input_diagnostics.update_drot_max_deg,
            input_diagnostics.update_dvel_max,
            input_diagnostics.frame_dpos_max,
            input_diagnostics.frame_drot_max_deg);

        RCLCPP_INFO(
            get_logger(),
            "[FASTLIO_HEALTH_DIAG] state=%s bad_streak=%d healthy_streak=%d "
            "zero_effective_streak=%d valid=%d last_good=%d "
            "lost_frames=%d reinit_count=%lu saturated_scans=%lu "
            "rejected_updates=%lu skipped_map_updates=%lu",
            health_name(localization_health_), bad_scan_count_,
            healthy_scan_count_, zero_effective_count_,
            last_localization_valid_ ? 1 : 0, last_good_valid_ ? 1 : 0,
            lost_scan_count_,
            static_cast<unsigned long>(local_map_reinit_count_),
            static_cast<unsigned long>(saturated_scan_count_window_),
            static_cast<unsigned long>(rejected_update_count_window_),
            static_cast<unsigned long>(skipped_map_count_window_));

        input_diagnostics.resetWindow(now);
        saturated_scan_count_window_ = 0;
        rejected_update_count_window_ = 0;
        skipped_map_count_window_ = 0;
    }

    // map_save 服务回调：pcd_save_en 使能时将累积地图保存为 PCD 文件。
    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubLocalizationValid_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    // rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool effect_pub_en = false, map_pub_en = false;
    bool robustness_enable_ = true;
    double imu_accel_saturation_threshold_ = 3.9;
    double imu_saturation_noise_scale_ = 100.0;
    double imu_saturation_ratio_warn_ = 0.05;
    int imu_saturation_streak_warn_ = 3;
    double min_effective_ratio_ = 0.30;
    int min_effective_points_ = 50;
    double max_mean_residual_ = 0.05;
    double critical_effective_ratio_ = 0.15;
    double critical_mean_residual_ = 0.08;
    double max_update_translation_ = 0.15;
    double max_update_rotation_deg_ = 2.0;
    double max_update_velocity_ = 1.0;
    double critical_update_translation_ = 0.35;
    double critical_update_rotation_deg_ = 5.0;
    double critical_update_velocity_ = 1.5;
    int degraded_enter_frames_ = 3;
    int recover_enter_frames_ = 10;
    int recover_normal_frames_ = 10;
    double max_degraded_duration_ = 3.0;
    int zero_effective_lost_frames_ = 5;
    double max_imu_dt_ = 0.02;
    double imu_gap_noise_scale_ = 100.0;
    double max_propagation_translation_ = 0.20;
    double max_propagation_velocity_ = 2.0;
    bool lost_reinit_enable_ = true;
    int lost_reinit_frames_ = 20;
    double lost_reinit_cooldown_ = 5.0;
    int recovery_bootstrap_frames_ = 3;
    int recovery_bootstrap_remaining_ = 0;
    LocalizationHealth localization_health_ = LocalizationHealth::NORMAL;
    int bad_scan_count_ = 0;
    int healthy_scan_count_ = 0;
    int zero_effective_count_ = 0;
    int lost_scan_count_ = 0;
    double last_reinit_time_ = -1.0;
    uint64_t local_map_reinit_count_ = 0;
    double degraded_start_time_ = -1.0;
    using StateCovariance =
        esekfom::esekf<state_ikfom, 12, input_ikfom>::cov;
    state_ikfom last_good_state_;
    StateCovariance last_good_covariance_ = StateCovariance::Identity();
    bool last_good_valid_ = false;
    bool localization_valid_published_ = false;
    bool last_localization_valid_ = false;
    uint64_t saturated_scan_count_window_ = 0;
    uint64_t rejected_update_count_window_ = 0;
    uint64_t skipped_map_count_window_ = 0;
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp;
    ofstream fout_pre, fout_out, fout_dbg;
};

// 程序入口：初始化 ROS 2、注册信号处理并 spin 主节点；退出时按需
// 保存最终地图点云与各阶段耗时的统计日志（fast_lio_time_log.csv）。
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++)
        {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
