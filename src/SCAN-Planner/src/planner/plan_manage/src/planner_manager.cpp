// ============================================================================
// 文件名：planner_manager.cpp
// 用途：SCAN 局部规划管理器（SCANPlannerManager）的实现文件。实现一次局部
//       重规划的完整流水线：初始路径生成（多项式/上一段轨迹）--> B 样条参数化
//       --> 走廊约束优化 --> 时间重分配细化 --> 动态可行性检查 -->
//       整条轨迹安全校验（结合 PCT 走廊），并实现紧急停车与全局轨迹规划。
// 结构：
//   - 匿名命名空间：路径几何工具（最近点、投影、走廊截取、Z 参考映射）
//   - reboundReplan：核心重规划流水线（STEP 1 INIT / STEP 2 OPTIMIZE /
//     STEP 3 REFINE）
//   - checkFullTrajectorySafety：整条轨迹安全校验（栅格碰撞 + 走廊偏离，最近新增）
//   - planGlobalTraj / planGlobalTrajWaypoints：全局参考轨迹生成
// 依赖：planner_manager.h（声明）、bspline_optimizer、uniform_bspline、
//       plan_env/grid_map、traj_utils/polynomial_traj
// ============================================================================
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <chrono>
#include <limits>
#include <thread>

namespace scan_planner
{
  namespace
  {
    // 计算三维点到折线路径的最近点（逐段投影取最小距离），
    // 供整条轨迹安全校验中计算走廊偏离度使用。
    Eigen::Vector3d nearestPointOnPath(const Eigen::Vector3d &point,
                                       const std::vector<Eigen::Vector3d> &path)
    {
      if (path.empty())
        return point;
      Eigen::Vector3d nearest = path.front();
      double best_distance_sq = (point - nearest).squaredNorm();
      for (size_t i = 1; i < path.size(); ++i)
      {
        const Eigen::Vector3d &from = path[i - 1];
        const Eigen::Vector3d segment = path[i] - from;
        const double length_sq = segment.squaredNorm();
        const double ratio = length_sq > 1e-12
                                 ? std::clamp((point - from).dot(segment) / length_sq, 0.0, 1.0)
                                 : 0.0;
        const Eigen::Vector3d candidate = from + ratio * segment;
        const double distance_sq = (point - candidate).squaredNorm();
        if (distance_sq < best_distance_sq)
        {
          best_distance_sq = distance_sq;
          nearest = candidate;
        }
      }
      return nearest;
    }

    // 路径投影结果：记录最近投影点所在的线段索引、投影点坐标与距离平方。
    struct PathProjection
    {
      size_t segment_index{0};
      Eigen::Vector3d point{Eigen::Vector3d::Zero()};
      double distance_sq{std::numeric_limits<double>::max()};
    };

    // 将点投影到路径上，从 first_segment 线段开始向后搜索（支持增量搜索，
    // 避免每次从 0 号线段全量扫描），返回最近的投影结果。
    PathProjection projectOnPath(const Eigen::Vector3d &point,
                                 const std::vector<Eigen::Vector3d> &path,
                                 size_t first_segment)
    {
      PathProjection result;
      if (path.size() < 2)
        return result;
      first_segment = std::min(first_segment, path.size() - 2);
      for (size_t i = first_segment; i + 1 < path.size(); ++i)
      {
        const Eigen::Vector3d segment = path[i + 1] - path[i];
        const double length_sq = segment.squaredNorm();
        const double ratio = length_sq > 1e-12
                                 ? std::clamp((point - path[i]).dot(segment) / length_sq, 0.0, 1.0)
                                 : 0.0;
        const Eigen::Vector3d candidate = path[i] + ratio * segment;
        const double distance_sq = (point - candidate).squaredNorm();
        if (distance_sq < result.distance_sq)
        {
          result.segment_index = i;
          result.point = candidate;
          result.distance_sq = distance_sq;
        }
      }
      return result;
    }

    // 截取走廊子段：以起点、目标点在走廊上的投影为界，取出中间这一段
    // 折线路径，作为本次局部重规划的局部走廊（PCT 走廊约束的输入）。
    std::vector<Eigen::Vector3d> extractPathSegment(
        const std::vector<Eigen::Vector3d> &path, const Eigen::Vector3d &start,
        const Eigen::Vector3d &target)
    {
      if (path.size() < 2)
        return path;
      const PathProjection start_projection = projectOnPath(start, path, 0);
      const PathProjection target_projection =
          projectOnPath(target, path, start_projection.segment_index);
      std::vector<Eigen::Vector3d> segment;
      segment.push_back(start_projection.point);
      for (size_t i = start_projection.segment_index + 1;
           i <= target_projection.segment_index && i < path.size(); ++i)
        segment.push_back(path[i]);
      if ((segment.back() - target_projection.point).norm() > 1e-6)
        segment.push_back(target_projection.point);
      return segment;
    }

    // 把走廊路径的 Z 值按弧长比例映射到给定点列上（首尾强制为 start_z/
    // target_z），使局部轨迹高度与 PCT 走廊的 Z 参考保持一致。
    void applyPathZReference(std::vector<Eigen::Vector3d> &points,
                             const std::vector<Eigen::Vector3d> &path,
                             double start_z, double target_z)
    {
      if (points.empty() || path.size() < 2)
        return;

      std::vector<double> path_arc(path.size(), 0.0);
      for (size_t i = 1; i < path.size(); ++i)
        path_arc[i] = path_arc[i - 1] + (path[i] - path[i - 1]).norm();
      if (path_arc.back() <= 1e-9)
        return;

      std::vector<double> point_arc(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
        point_arc[i] = point_arc[i - 1] + (points[i] - points[i - 1]).norm();
      const double point_length = point_arc.back();
      size_t path_index = 1;
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = point_length > 1e-9
                                 ? point_arc[i] / point_length
                                 : static_cast<double>(i) / std::max<size_t>(1, points.size() - 1);
        const double target_arc = ratio * path_arc.back();
        while (path_index + 1 < path.size() && path_arc[path_index] < target_arc)
          ++path_index;
        const double segment_length = path_arc[path_index] - path_arc[path_index - 1];
        const double segment_ratio = segment_length > 1e-9
                                         ? (target_arc - path_arc[path_index - 1]) / segment_length
                                         : 0.0;
        points[i](2) = path[path_index - 1](2) +
                       segment_ratio * (path[path_index](2) - path[path_index - 1](2));
      }
      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  // 初始化各规划子模块：读取规划参数（速度/加速度/控制点间距/走廊最大偏离等）、
  // 创建栅格地图、B 样条优化器与 A* 初始化，并绑定可视化对象。
  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    corridor_max_deviation_ = get_double("manager.corridor_max_deviation", 0.6);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  // 核心重规划流水线（与头文件声明对应）。流程：
  //   STEP 1 INIT：生成初始路径点集（首次/指定时用多项式轨迹，否则沿用
  //                上一段轨迹重采样），参数化得到 B 样条控制点；
  //   STEP 2 OPTIMIZE：调用优化器在 PCT 走廊约束下优化控制点；
  //   STEP 3 REFINE：动态可行性检查失败时按比例重分配时间并再次优化；
  //   最终通过动态可行性 + 整条轨迹安全校验（碰撞/走廊偏离）后写入 local_data_。
  // 连续失败会累计 continuous_failures_count_（用于随机扰动幅度衰减）。
  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> &corridor_path)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 点睛：从全局走廊中截取起点到局部目标之间的子段作为局部走廊，
    // 并施加 Z 参考；走廊约束（corridor_max_deviation_）在优化器与
    // 整条轨迹安全校验中被共用。
    const std::vector<Eigen::Vector3d> local_corridor =
        extractPathSegment(corridor_path, start_pt, local_target_pt);
    applyPathZReference(point_set, local_corridor, start_pt(2), local_target_pt(2));
    bspline_optimizer_rebound_->setCorridorPath(local_corridor, corridor_max_deviation_);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos) ||
        !checkFullTrajectorySafety(pos, local_corridor))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  // 紧急停车：把全部控制点压到 stop_pos 处（零速 B 样条），立即刷新
  // local_data_ 使轨迹变为原地停住。
  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  // 全局轨迹规划（waypoint 模式）：把起点与各航点连成折线，距离过大的
  // 航段自动插值细分，用最小 snap 多项式生成全局参考轨迹并写入 global_data_
  // （终点不约束位置，只约束速度/加速度）。
  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  // 全局轨迹规划（单目标模式）：起点到终点的最小 snap 全局参考轨迹
  // （planGlobalTrajWaypoints 的单点特例）。
  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  // 轨迹细化：按 ratio 重分配时间（reparamBspline）后，以重采样点作为
  // 参考点再次调用优化器细化，返回最终最优控制点。
  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  // 刷新局部轨迹数据：写入位置/速度/加速度 B 样条、起点与时长，轨迹编号 +1；
  // 是 local_data_ 的唯一写入入口（供可视化与闭环控制读取）。
  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  // 动态可行性检查：按采样步长遍历整条轨迹，任何时刻速度/加速度模长
  // 超过（物理限制 + 容差）即告失败。
  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  // 整条轨迹安全校验（重点，最近新增逻辑）：按分辨率自适应步长密集采样
  // 整条轨迹，逐点用带偏航的膨胀占据查询检测碰撞；若传入 PCT 走廊路径，
  // 还校验轨迹点到走廊最近点的偏离不超过 corridor_max_deviation_。
  bool SCANPlannerManager::checkFullTrajectorySafety(
      UniformBspline position_traj,
      const std::vector<Eigen::Vector3d> &corridor_path)
  {
    if (!grid_map_)
      return false;

    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::clamp(
        grid_map_->getResolution() / std::max(2.0 * pp_.max_vel_, 1e-3), 0.01, 0.05);
    const int sample_count = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    for (int sample = 0; sample <= sample_count; ++sample)
    {
      const double t = std::min(sample * sample_dt, duration);
      const double previous_t = std::max(t - sample_dt, 0.0);
      const double next_t = std::min(t + sample_dt, duration);
      const Eigen::Vector3d point = position_traj.evaluateDeBoorT(t);
      const Eigen::Vector3d previous = position_traj.evaluateDeBoorT(previous_t);
      const Eigen::Vector3d next = position_traj.evaluateDeBoorT(next_t);
      const Eigen::Vector2d direction = (next - previous).head<2>();
      const double yaw = direction.squaredNorm() > 1e-8
                             ? std::atan2(direction(1), direction(0))
                             : 0.0;
      if (grid_map_->getInflateOccupancy(point, yaw) > 0)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Reject full trajectory: collision at t=%.3f [%.2f %.2f %.2f]",
                    t, point(0), point(1), point(2));
        return false;
      }

      if (corridor_path.size() >= 2 && corridor_max_deviation_ > 0.0)
      {
        const double deviation = (point - nearestPointOnPath(point, corridor_path)).norm();
        if (deviation > corridor_max_deviation_)
        {
          RCLCPP_WARN(node_->get_logger(),
                      "Reject full trajectory: corridor deviation at t=%.3f is %.3f > %.3f",
                      t, deviation, corridor_max_deviation_);
          return false;
        }
      }
    }
    return true;
  }

  // B 样条重参数化：将轨迹时间整体缩放 ratio 倍，按新时间步长重采样
  // 点集并重新参数化为 B 样条，返回新控制点、时间步长与时间增量。
  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
