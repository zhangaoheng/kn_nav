// ============================================================================
// 文件名：pure_pursuit_planner_component.hpp
// 用途：纯追踪（Pure Pursuit）核心算法组件的声明。输入一条离散全局路径
//       （x/y/切向角/曲率）与当前位姿，输出线速度 v 与角速度 w。
// 结构：
//   - Pose2D：二维位姿（x/y/yaw）；
//   - PurePursuitConfig：算法参数集合（前视、限速、限转角、终段对准等）；
//   - PurePursuitComponent：无 ROS 依赖的算法类，可独立单元测试。
// 关键逻辑：最近点搜索（首次全路径、后续局部窗口）→ 前视点搜索 → 曲率限速
//           → 转向偏差计算（含原地转向状态机）→ 输出 (v, w)。
// 依赖：仅标准库；被 pure_pursuit_planner_node 与测试代码调用。
// ============================================================================

// Directory: pure_pursuit_planner/include/pure_pursuit_planner/pure_pursuit_component.hpp
#pragma once

#include <vector>
#include <utility>

namespace pure_pursuit_planner {

// 二维平面位姿：x/y 为地图坐标，yaw 为朝向角（弧度）

struct Pose2D {
    double x;
    double y;
    double yaw;
};

// 纯追踪算法参数（与 config/*.yaml 中字段一一对应）：
// 前视距离 Lf = k*v + Lfc；速度由曲率映射到 [minVelocity, maxVelocity]；
// 角度相关阈值用于原地转向与终段朝向对准状态机

struct PurePursuitConfig {
    double k = 0.5;
    double Lfc = 0.8;
    double Kp = 1.0;
    double dt = 0.1;
    double goal_threshold = 0.4;
    double final_heading_entry_distance = 0.20;
    double final_heading_command_deadband = 0.02;
    double min_final_angular_velocity = 0.20;
    double max_acceleration = 0.08;
    double minCurvature = 0.0;
    double maxCurvature = 3.0;
    double minVelocity = 0.4;
    double maxVelocity = 0.7;
    double maxAngularVelocity = 1.3;
    double rotate_to_path_threshold = 1.047;
    double rotate_to_path_tolerance = 0.349;
    double goal_yaw_tolerance = 0.175;
    double rotate_to_heading_gain = 1.0;
    bool standalone_goal_completion = false;
    double obstacle_th = 0.5;
    double odom_timeout = 0.3;
};

// 纯追踪核心算法类（无 ROS 依赖）。
// 对外主入口 computeVelocity()：单次调用即输出 (v, w)；
// 其余方法为可单独测试的算法子步骤

class PurePursuitComponent {
public:
    PurePursuitComponent(const PurePursuitConfig& config);

    void setPath(const std::vector<double>& cx,
                 const std::vector<double>& cy,
                 const std::vector<double>& cyaw,
                 const std::vector<double>& ck);

    void setPose(const Pose2D& pose, double velocity);

    // 主入口：根据路径与当前位姿计算本周期速度指令，返回 {线速度, 角速度}。
    // 处理顺序：终段朝向对准 → 独立完成判定 → 原地转向对准 → 前视点追踪

    std::vector<double> computeVelocity(
        const std::vector<double>& cx,
        const std::vector<double>& cy,
        const std::vector<double>& cyaw,
        const std::vector<double>& ck,
        const Pose2D& pose, 
        double velocity,
        bool final_approach = false
    );

    // 计算前视距离：Lf = k * 当前速度 + Lfc（速度越快看得越远）

    double calcLf(double k, double current_velocity, double Lfc) const;

    // 以下两个成员设为 public 仅为测试访问方便：
    // odom_sub_flag：是否已装载路径（未装载时 searchTargetIndex 拒绝计算）

    bool odom_sub_flag = false;

    // 最近点索引缓存（-1 表示未初始化，触发全路径搜索）

    int oldNearestPointIndex = -1;

    // 后续在上一最近点附近的小窗口内搜索，降低计算量并避免索引跳变

    int calcOldNearestPointIndex() const;

    // 首次（或重置后）全路径扫描，返回距当前位置最近的点索引

    int calcFirstNearestPointIndex() const;

    // 目标到达判定（当前实现为透传 v/w，不再强制停车；停车由上层终段逻辑负责）

    std::pair<double, double> isGoalReached(double v, double w) const;

    // 搜索前视目标点：返回 {目标点索引, 前视距离 Lf}

    std::pair<int, double> searchTargetIndex();

    // 曲率→目标速度映射：曲率越大速度越低（弯道减速）

    double curvatureToVelocity(double curvature) const;

    // 角度偏差 alpha 归一化到 [-π, π]，并对 ±π 附近的奇异点做补偿

    double alphaExceptionHandling(double tempAlpha) const;

    // 纯追踪转向律：w = v * sin(alpha) / Lf

    double calculateAngularVelocity(double v, double alpha, double Lf) const;

private:
    double calcDistance(double x1, double y1, double x2, double y2) const;
    double normalizeAngle(double angle) const;
    double calculateFinalRotationAngularVelocity(double yaw_error) const;
    double calculateRotationAngularVelocity(double yaw_error) const;
    std::pair<double, std::pair<double, double>> calcClosestPointOnPath();
    std::pair<double, double> calcAcceleration(double current_vel);

    // 内部状态：路径/位姿/搜索缓存、障碍与原地转向标志、加速度平滑量

    // Config and state
    PurePursuitConfig cfg_;
    Pose2D current_pose_;
    double current_velocity_ = 0.0;

    std::vector<double> cx_, cy_, cyaw_, ck_;
    int oldNearestPointIndex_ = -1;
    int targetIndex_ = 0;

    double obstacle_x_ = 0.0;
    double obstacle_y_ = 0.0;
    bool obstacle_detected_ = false;
    bool avoidance_flag_ = false;

    double previous_velocity_ = 0.0;
    double previous_time_ = 0.0;
    bool goal_reached_ = false;
    bool rotating_to_path_ = false;

    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double temp_target_x_ = 0.0;
    double temp_target_y_ = 0.0;
    double pre_min_distance_ = 0.0;
    double diff_min_dist_ = 0.0;
};

} // namespace pure_pursuit_planner
