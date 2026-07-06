#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <art_planner_msgs/action/plan_to_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rog_map/rog_map.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using PlanToGoal = art_planner_msgs::action::PlanToGoal;
using GoalHandlePlanToGoal = rclcpp_action::ServerGoalHandle<PlanToGoal>;
using Vec3 = Eigen::Vector3d;

class StdoutRosLogger
{
public:
  explicit StdoutRosLogger(const rclcpp::Node::SharedPtr & node)
  : logger_(node->get_logger())
  {
    if (pipe(pipe_fd_) != 0) {
      RCLCPP_WARN(
        logger_, "Failed to capture ROG-Map stdout: pipe() failed: %s", std::strerror(errno));
      return;
    }

    saved_stdout_ = dup(STDOUT_FILENO);
    if (saved_stdout_ < 0) {
      RCLCPP_WARN(
        logger_, "Failed to capture ROG-Map stdout: dup() failed: %s", std::strerror(errno));
      closePipe();
      return;
    }

    std::cout.flush();
    std::fflush(stdout);
    if (dup2(pipe_fd_[1], STDOUT_FILENO) < 0) {
      RCLCPP_WARN(
        logger_, "Failed to capture ROG-Map stdout: dup2() failed: %s", std::strerror(errno));
      close(saved_stdout_);
      saved_stdout_ = -1;
      closePipe();
      return;
    }

    close(pipe_fd_[1]);
    pipe_fd_[1] = -1;
    setvbuf(stdout, nullptr, _IOLBF, 0);
    reader_ = std::thread(&StdoutRosLogger::readLoop, this);
  }

  StdoutRosLogger(const StdoutRosLogger &) = delete;
  StdoutRosLogger & operator=(const StdoutRosLogger &) = delete;

  ~StdoutRosLogger()
  {
    restoreStdout();
    if (reader_.joinable()) {
      reader_.join();
    }
    closePipe();
  }

private:
  void restoreStdout()
  {
    if (saved_stdout_ < 0) {
      return;
    }

    std::cout.flush();
    std::fflush(stdout);
    dup2(saved_stdout_, STDOUT_FILENO);
    close(saved_stdout_);
    saved_stdout_ = -1;
  }

  void readLoop()
  {
    std::string pending;
    std::array<char, 512> buffer{};
    ssize_t bytes_read = 0;
    while ((bytes_read = read(pipe_fd_[0], buffer.data(), buffer.size())) > 0) {
      pending.append(buffer.data(), static_cast<size_t>(bytes_read));
      size_t newline = std::string::npos;
      while ((newline = pending.find('\n')) != std::string::npos) {
        logLine(pending.substr(0, newline));
        pending.erase(0, newline + 1);
      }
    }

    if (!pending.empty()) {
      logLine(pending);
    }
  }

  void logLine(const std::string & raw_line) const
  {
    const std::string line = trim(stripAnsi(raw_line));
    if (line.empty()) {
      return;
    }
    RCLCPP_INFO(logger_, "[rog_map] %s", line.c_str());
  }

  static std::string stripAnsi(const std::string & input)
  {
    std::string output;
    output.reserve(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
      if (input[i] == '\033' && i + 1 < input.size() && input[i + 1] == '[') {
        i += 2;
        while (i < input.size() && !std::isalpha(static_cast<unsigned char>(input[i]))) {
          ++i;
        }
        continue;
      }
      if (input[i] != '\r') {
        output.push_back(input[i]);
      }
    }
    return output;
  }

  static std::string trim(const std::string & input)
  {
    size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
      ++begin;
    }
    size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
      --end;
    }
    return input.substr(begin, end - begin);
  }

  void closePipe()
  {
    for (int & fd : pipe_fd_) {
      if (fd >= 0) {
        close(fd);
        fd = -1;
      }
    }
  }

  rclcpp::Logger logger_;
  int pipe_fd_[2]{-1, -1};
  int saved_stdout_{-1};
  std::thread reader_;
};

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

geometry_msgs::msg::Quaternion yawPitchToQuaternion(double yaw, double pitch)
{
  tf2::Quaternion q;
  q.setRPY(0.0, -pitch, yaw);
  return tf2::toMsg(q);
}

double poseYaw(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  return tf2::getYaw(q);
}

Vec3 posePosition(const geometry_msgs::msg::PoseStamped & pose)
{
  return Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

struct SearchNode
{
  bool seen{false};
  bool closed{false};
  int parent{-1};
  int ix{0};
  int iy{0};
  int step_x{0};
  int step_y{0};
  double g{std::numeric_limits<double>::infinity()};
  double f{std::numeric_limits<double>::infinity()};
};

struct QueueItem
{
  double f;
  int index;
  bool operator<(const QueueItem & other) const { return f > other.f; }
};

struct PlanAttempt
{
  uint8_t status{PlanToGoal::Feedback::NO_SOLUTION};
  std::string reason;
  nav_msgs::msg::Path path;
};

struct FootprintCheck
{
  bool free{true};
  Vec3 query{Vec3::Zero()};
  rog_map::GridType grid_type{rog_map::UNDEFINED};
  std::string reason;
  double longitudinal_offset{0.0};
  double pitch{0.0};
  double dz{0.0};
  int radial_index{0};
};

std::string vecToString(const Vec3 & value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << '(' << value.x() << ", " << value.y() << ", " << value.z() << ')';
  return out.str();
}

std::string gridTypeToString(const rog_map::GridType grid_type)
{
  const auto index = static_cast<size_t>(grid_type);
  if (index < rog_map::GridTypeStr.size()) {
    return rog_map::GridTypeStr[index];
  }
  return "UNKNOWN_GRID_TYPE_" + std::to_string(static_cast<int>(grid_type));
}

std::string footprintFailureToString(const FootprintCheck & check)
{
  std::ostringstream out;
  out << check.reason
      << ", query=" << vecToString(check.query)
      << ", grid=" << gridTypeToString(check.grid_type)
      << ", longitudinal_offset=" << std::fixed << std::setprecision(3)
      << check.longitudinal_offset
      << ", pitch=" << check.pitch
      << ", dz=" << check.dz
      << ", radial_index=" << check.radial_index;
  return out.str();
}

class RogLocalPlanner
{
public:
  explicit RogLocalPlanner(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
    map_ = std::make_shared<rog_map::ROGMap>(node_);
    readParameters();

    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).transient_local());
    if (visualization_enable_footprint_markers_) {
      footprint_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        visualization_footprint_marker_topic_, rclcpp::QoS(1).transient_local());
    }
    action_server_ = rclcpp_action::create_server<PlanToGoal>(
      node_,
      action_name_,
      std::bind(&RogLocalPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RogLocalPlanner::handleCancel, this, std::placeholders::_1),
      std::bind(&RogLocalPlanner::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      node_->get_logger(),
      "ROG local planner ready: action=%s, path=%s, frame=%s, robot=%s",
      action_name_.c_str(), path_topic_.c_str(), global_frame_.c_str(), robot_frame_.c_str());
  }

private:
  void readParameters()
  {
    global_frame_ = declareAndGet<std::string>("global_frame", "map");
    robot_frame_ = declareAndGet<std::string>("robot_frame", "base_link");
    action_name_ = declareAndGet<std::string>("action_name", "/art_planner/plan_to_goal");
    path_topic_ = declareAndGet<std::string>("path_topic", "/art_planner/path");

    replan_rate_ = declareAndGet<double>("planner.replan_rate", 2.0);
    planning_timeout_ = declareAndGet<double>("planner.planning_timeout", 0.20);
    search_resolution_ = declareAndGet<double>("planner.search_resolution", 0.0);
    search_margin_ = declareAndGet<double>("planner.search_margin", 3.0);
    output_spacing_ = declareAndGet<double>("planner.output_spacing", 0.15);
    unknown_as_occupied_ = declareAndGet<bool>("planner.unknown_as_occupied", false);
    require_map_ready_ = declareAndGet<bool>("planner.require_map_ready", true);
    allow_diagonal_ = declareAndGet<bool>("planner.allow_diagonal", true);
    enable_path_smoothing_ = declareAndGet<bool>("planner.enable_path_smoothing", false);
    path_smoothing_iterations_ = declareAndGet<int>("planner.path_smoothing_iterations", 20);
    path_smoothing_data_weight_ = declareAndGet<double>("planner.path_smoothing_data_weight", 0.35);
    path_smoothing_smooth_weight_ = declareAndGet<double>("planner.path_smoothing_smooth_weight", 0.20);
    path_smoothing_max_deviation_ = declareAndGet<double>("planner.path_smoothing_max_deviation", 0.20);
    path_smoothing_max_deviation_ = declareAndGet<double>(
      "smoothing.max_deviation", path_smoothing_max_deviation_);
    smoothing_min_point_spacing_ = declareAndGet<double>("smoothing.min_point_spacing", 0.10);
    smoothing_collinear_angle_threshold_ = declareAndGet<double>(
      "smoothing.collinear_angle_threshold", 0.08);
    smoothing_max_segment_slope_ = declareAndGet<double>("smoothing.max_segment_slope", 0.40);
    smoothing_max_turn_angle_ = declareAndGet<double>("smoothing.max_turn_angle", 1.20);

    cost_height_weight_ = declareAndGet<double>("cost.height_weight", 1.0);
    cost_unknown_penalty_ = declareAndGet<double>("cost.unknown_penalty", 1.0);
    cost_obstacle_weight_ = declareAndGet<double>("cost.obstacle_weight", 4.0);
    cost_inflation_weight_ = declareAndGet<double>("cost.inflation_weight", 2.0);
    cost_sample_radius_ = declareAndGet<double>("cost.cost_sample_radius", 0.35);
    cost_sample_step_ = declareAndGet<double>("cost.cost_sample_step", 0.10);
    cost_turn_weight_ = declareAndGet<double>("cost.turn_weight", 0.2);

    projection_enable_ = declareAndGet<bool>("projection.enable", true);
    projection_radius_ = declareAndGet<double>("projection.radius", 0.45);
    projection_step_ = declareAndGet<double>("projection.step", 0.10);

    cylinder_radius_ = declareAndGet<double>("robot.cylinder_radius", 0.28);
    cylinder_offset_ = declareAndGet<double>("robot.cylinder_offset", 0.23);
    clearance_up_ = declareAndGet<double>("robot.clearance_up", 0.45);
    clearance_down_ = declareAndGet<double>("robot.clearance_down", 0.35);
    vertical_step_ = declareAndGet<double>("robot.vertical_step", 0.15);
    radial_samples_ = declareAndGet<int>("robot.radial_samples", 8);
    enable_pitch_footprint_ = declareAndGet<bool>("robot.enable_pitch_footprint", true);
    max_footprint_pitch_ = declareAndGet<double>("robot.max_footprint_pitch", 0.70);
    footprint_z_offset_ = declareAndGet<double>("robot.footprint_z_offset", 0.0);

    visualization_enable_footprint_markers_ = declareAndGet<bool>(
      "visualization.enable_footprint_markers", true);
    visualization_footprint_marker_topic_ = declareAndGet<std::string>(
      "visualization.footprint_marker_topic", "/rog_local_planner/footprint_markers");
    visualization_footprint_marker_stride_ = declareAndGet<int>(
      "visualization.footprint_marker_stride", 2);
    visualization_footprint_marker_alpha_ = declareAndGet<double>(
      "visualization.footprint_marker_alpha", 0.28);

    replan_rate_ = std::max(0.1, replan_rate_);
    planning_timeout_ = std::max(0.01, planning_timeout_);
    output_spacing_ = std::max(0.03, output_spacing_);
    search_margin_ = std::max(0.0, search_margin_);
    path_smoothing_iterations_ = std::max(0, path_smoothing_iterations_);
    path_smoothing_data_weight_ = std::max(0.0, path_smoothing_data_weight_);
    path_smoothing_smooth_weight_ = std::max(0.0, path_smoothing_smooth_weight_);
    path_smoothing_max_deviation_ = std::max(0.0, path_smoothing_max_deviation_);
    smoothing_min_point_spacing_ = std::max(0.0, smoothing_min_point_spacing_);
    smoothing_collinear_angle_threshold_ = std::clamp(
      smoothing_collinear_angle_threshold_, 0.0, M_PI);
    smoothing_max_segment_slope_ = std::max(0.0, smoothing_max_segment_slope_);
    smoothing_max_turn_angle_ = std::clamp(smoothing_max_turn_angle_, 0.0, M_PI);
    cost_height_weight_ = std::max(0.0, cost_height_weight_);
    cost_unknown_penalty_ = std::max(0.0, cost_unknown_penalty_);
    cost_obstacle_weight_ = std::max(0.0, cost_obstacle_weight_);
    cost_inflation_weight_ = std::max(0.0, cost_inflation_weight_);
    cost_sample_radius_ = std::max(0.0, cost_sample_radius_);
    cost_sample_step_ = std::max(0.03, cost_sample_step_);
    cost_turn_weight_ = std::max(0.0, cost_turn_weight_);
    projection_radius_ = std::max(0.0, projection_radius_);
    projection_step_ = std::max(0.03, projection_step_);
    cylinder_radius_ = std::max(0.0, cylinder_radius_);
    cylinder_offset_ = std::max(0.0, cylinder_offset_);
    clearance_up_ = std::max(0.0, clearance_up_);
    clearance_down_ = std::max(0.0, clearance_down_);
    vertical_step_ = std::max(0.03, vertical_step_);
    radial_samples_ = std::max(4, radial_samples_);
    max_footprint_pitch_ = std::clamp(max_footprint_pitch_, 0.0, 0.5 * M_PI);
    footprint_z_offset_ = std::max(0.0, footprint_z_offset_);
    visualization_footprint_marker_stride_ = std::max(1, visualization_footprint_marker_stride_);
    visualization_footprint_marker_alpha_ = std::clamp(visualization_footprint_marker_alpha_, 0.0, 1.0);

    RCLCPP_INFO(
      node_->get_logger(),
      "ROG local planner footprint: radius=%.3f, offset=%.3f, clearance_up=%.3f, clearance_down=%.3f, "
      "vertical_step=%.3f, radial_samples=%d, pitch_enabled=%s, max_pitch=%.3f rad, footprint_z_offset=%.3f",
      cylinder_radius_, cylinder_offset_, clearance_up_, clearance_down_, vertical_step_,
      radial_samples_, enable_pitch_footprint_ ? "true" : "false", max_footprint_pitch_,
      footprint_z_offset_);
  }

  template<typename T>
  T declareAndGet(const std::string & name, const T & default_value)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    return node_->get_parameter(name).get_value<T>();
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlanToGoal::Goal> goal)
  {
    if (goal->goal.header.frame_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Rejecting local goal with empty frame_id");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlanToGoal>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandlePlanToGoal> goal_handle)
  {
    std::thread{std::bind(&RogLocalPlanner::executeGoal, this, goal_handle)}.detach();
  }

  void executeGoal(const std::shared_ptr<GoalHandlePlanToGoal> goal_handle)
  {
    publishFeedback(goal_handle, PlanToGoal::Feedback::GOAL_RECEIVED);
    const auto result = std::make_shared<PlanToGoal::Result>();
    const auto sleep_duration = std::chrono::duration<double>(1.0 / replan_rate_);

    while (rclcpp::ok() && goal_handle->is_active()) {
      if (goal_handle->is_canceling()) {
        publishEmptyPath();
        clearFootprintMarkers();
        goal_handle->canceled(result);
        return;
      }

      publishFeedback(goal_handle, PlanToGoal::Feedback::PLANNING);
      const PlanAttempt attempt = planOnce(goal_handle->get_goal()->goal);
      if (attempt.status == PlanToGoal::Feedback::FOUND_SOLUTION) {
        path_pub_->publish(attempt.path);
        publishFootprintMarkers(attempt.path);
        publishFeedback(goal_handle, PlanToGoal::Feedback::FOUND_SOLUTION);
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "ROG local planning failed: %s",
          attempt.reason.c_str());
        publishFeedback(goal_handle, attempt.status);
        clearFootprintMarkers();
        goal_handle->abort(result);
        return;
      }

      std::this_thread::sleep_for(sleep_duration);
    }
  }

  void publishFeedback(const std::shared_ptr<GoalHandlePlanToGoal> & goal_handle, uint8_t status) const
  {
    if (!goal_handle || !goal_handle->is_active()) {
      return;
    }
    auto feedback = std::make_shared<PlanToGoal::Feedback>();
    feedback->status = status;
    goal_handle->publish_feedback(feedback);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();
    path_pub_->publish(path);
    clearFootprintMarkers();
  }

  PlanAttempt planOnce(const geometry_msgs::msg::PoseStamped & raw_goal)
  {
    PlanAttempt attempt;
    attempt.path.header.frame_id = global_frame_;
    attempt.path.header.stamp = node_->now();

    if (require_map_ready_ && !map_->isMapReady()) {
      attempt.status = PlanToGoal::Feedback::NO_MAP;
      attempt.reason = "ROG map has not received point cloud data";
      return attempt;
    }

    geometry_msgs::msg::PoseStamped start_pose;
    if (!lookupRobotPose(start_pose)) {
      attempt.status = PlanToGoal::Feedback::NO_ROBOT_TF;
      attempt.reason = "could not lookup robot pose";
      return attempt;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    if (!transformGoal(raw_goal, goal_pose)) {
      attempt.status = PlanToGoal::Feedback::NO_GOAL_TF;
      attempt.reason = "could not transform local goal";
      return attempt;
    }

    Vec3 start = posePosition(start_pose);
    Vec3 goal = posePosition(goal_pose);
    const double goal_yaw = poseYaw(goal_pose.pose);
    double start_yaw = std::atan2(goal.y() - start.y(), goal.x() - start.x());
    double path_pitch = segmentPitch(start, goal);

    const bool start_center_inside = map_->insideLocalMapPublic(start);
    const FootprintCheck start_check = start_center_inside ?
      checkFootprintFree(start, start_yaw, path_pitch) :
      makeCenterOutsideFailure(start);
    if (!start_center_inside || !start_check.free) {
      Vec3 projected_start;
      std::string projection_failure;
      if (projectNearestFreePose(start, start_yaw, path_pitch, start, goal, projected_start, &projection_failure)) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "ROG projected invalid start: original=%s, projected=%s, offset=%.3f",
          vecToString(start).c_str(), vecToString(projected_start).c_str(),
          (projected_start - start).norm());
        start = projected_start;
        start_yaw = std::atan2(goal.y() - start.y(), goal.x() - start.x());
        path_pitch = segmentPitch(start, goal);
      } else {
        attempt.status = PlanToGoal::Feedback::INVALID_START;
        attempt.reason = makePlanFailureReason(
          "start is outside local ROG map or in collision",
          start,
          goal,
          start_yaw,
          goal_yaw,
          footprintFailureToString(start_check) + ", projection=" + projection_failure);
        return attempt;
      }
    }

    const bool goal_center_inside = map_->insideLocalMapPublic(goal);
    const FootprintCheck goal_check = goal_center_inside ?
      checkFootprintFree(goal, start_yaw, path_pitch) :
      makeCenterOutsideFailure(goal);
    if (!goal_center_inside || !goal_check.free) {
      Vec3 projected_goal;
      std::string projection_failure;
      if (projectNearestFreePose(goal, start_yaw, path_pitch, start, goal, projected_goal, &projection_failure)) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "ROG projected invalid goal: original=%s, projected=%s, offset=%.3f",
          vecToString(goal).c_str(), vecToString(projected_goal).c_str(),
          (projected_goal - goal).norm());
        goal = projected_goal;
        start_yaw = std::atan2(goal.y() - start.y(), goal.x() - start.x());
        path_pitch = segmentPitch(start, goal);
      } else {
        attempt.status = PlanToGoal::Feedback::INVALID_GOAL;
        attempt.reason = makePlanFailureReason(
          "goal is outside local ROG map or in collision",
          start,
          goal,
          start_yaw,
          goal_yaw,
          footprintFailureToString(goal_check) + ", projection=" + projection_failure);
        return attempt;
      }
    }

    std::vector<Vec3> points;
    if (isFootprintLineFree(start, goal)) {
      points = {start, goal};
    } else {
      std::string astar_failure;
      if (!runProjectedAStar(start, goal, points, &astar_failure)) {
        attempt.status = PlanToGoal::Feedback::NO_SOLUTION;
        attempt.reason = makePlanFailureReason(
          "projected A* found no collision-free path",
          start,
          goal,
          start_yaw,
          goal_yaw,
          astar_failure);
        return attempt;
      }
    }

    points = cleanPathGeometry(shortcutPath(points));
    points = projectPathToLinearHeight(resamplePath(points), start, goal);
    if (enable_path_smoothing_ && points.size() > 2) {
      const std::vector<Vec3> smoothed = smoothPath(points, start, goal);
      std::string smoothing_failure;
      if (pathCollisionFree(smoothed, &smoothing_failure, &points, true)) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "ROG path smoothing accepted: raw_points=%zu, smoothed_points=%zu",
          points.size(), smoothed.size());
        points = smoothed;
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          1000,
          "ROG path smoothing rejected: %s; publishing unsmoothed path",
          smoothing_failure.c_str());
      }
    }
    if (points.size() < 2) {
      attempt.status = PlanToGoal::Feedback::NO_SOLUTION;
      attempt.reason = makePlanFailureReason(
        "planned path has fewer than two points",
        start,
        goal,
        start_yaw,
        goal_yaw,
        "points=" + std::to_string(points.size()));
      return attempt;
    }
    std::string path_collision_failure;
    if (!pathCollisionFree(points, &path_collision_failure)) {
      attempt.status = PlanToGoal::Feedback::NO_SOLUTION;
      attempt.reason = makePlanFailureReason(
        "planned path failed final collision validation",
        start,
        goal,
        start_yaw,
        goal_yaw,
        path_collision_failure);
      return attempt;
    }

    attempt.path = makePath(points, goal_pose.pose.orientation);
    attempt.status = PlanToGoal::Feedback::FOUND_SOLUTION;
    return attempt;
  }

  bool lookupRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Robot TF lookup failed: %s", ex.what());
      return false;
    }
  }

  bool transformGoal(
    const geometry_msgs::msg::PoseStamped & raw_goal,
    geometry_msgs::msg::PoseStamped & goal)
  {
    if (raw_goal.header.frame_id == global_frame_) {
      goal = raw_goal;
      return true;
    }
    try {
      tf_buffer_.transform(raw_goal, goal, global_frame_, tf2::durationFromSec(0.1));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Goal TF transform failed: %s", ex.what());
      return false;
    }
  }

  bool runProjectedAStar(
    const Vec3 & start,
    const Vec3 & goal,
    std::vector<Vec3> & path,
    std::string * failure_reason = nullptr)
  {
    const double resolution = search_resolution_ > 0.0 ? search_resolution_ : map_->getInfResolution();
    if (!std::isfinite(resolution) || resolution <= 0.0) {
      setFailureReason(failure_reason, "invalid search resolution");
      return false;
    }

    rog_map::Vec3f map_min;
    rog_map::Vec3f map_max;
    map_->getLocalMapBounds(map_min, map_max);

    const double min_x = std::max(map_min.x(), std::min(start.x(), goal.x()) - search_margin_);
    const double max_x = std::min(map_max.x(), std::max(start.x(), goal.x()) + search_margin_);
    const double min_y = std::max(map_min.y(), std::min(start.y(), goal.y()) - search_margin_);
    const double max_y = std::min(map_max.y(), std::max(start.y(), goal.y()) + search_margin_);

    const int min_ix = static_cast<int>(std::floor(min_x / resolution));
    const int max_ix = static_cast<int>(std::floor(max_x / resolution));
    const int min_iy = static_cast<int>(std::floor(min_y / resolution));
    const int max_iy = static_cast<int>(std::floor(max_y / resolution));
    const int nx = max_ix - min_ix + 1;
    const int ny = max_iy - min_iy + 1;
    if (nx <= 1 || ny <= 1) {
      std::ostringstream out;
      out << "search window too small, nx=" << nx << ", ny=" << ny
          << ", resolution=" << resolution;
      setFailureReason(failure_reason, out.str());
      return false;
    }

    auto toIndex = [&](int ix, int iy) {
      return (ix - min_ix) * ny + (iy - min_iy);
    };
    auto inBounds = [&](int ix, int iy) {
      return ix >= min_ix && ix <= max_ix && iy >= min_iy && iy <= max_iy;
    };
    auto toGrid = [&](const Vec3 & p) {
      return std::pair<int, int>(
        static_cast<int>(std::floor(p.x() / resolution)),
        static_cast<int>(std::floor(p.y() / resolution)));
    };
    auto toPoint = [&](int ix, int iy) {
      const double x = (static_cast<double>(ix) + 0.5) * resolution;
      const double y = (static_cast<double>(iy) + 0.5) * resolution;
      return Vec3(x, y, interpolatedHeight(start, goal, x, y));
    };

    const auto [start_ix, start_iy] = toGrid(start);
    const auto [goal_ix, goal_iy] = toGrid(goal);
    if (!inBounds(start_ix, start_iy) || !inBounds(goal_ix, goal_iy)) {
      std::ostringstream out;
      out << "start/goal grid outside A* window, start_grid=("
          << start_ix << ", " << start_iy << "), goal_grid=("
          << goal_ix << ", " << goal_iy << "), window_x=["
          << min_ix << ", " << max_ix << "], window_y=["
          << min_iy << ", " << max_iy << ']';
      setFailureReason(failure_reason, out.str());
      return false;
    }

    std::vector<SearchNode> nodes(static_cast<size_t>(nx * ny));
    std::priority_queue<QueueItem> open_set;
    const int start_index = toIndex(start_ix, start_iy);
    const int goal_index = toIndex(goal_ix, goal_iy);
    nodes[start_index].seen = true;
    nodes[start_index].ix = start_ix;
    nodes[start_index].iy = start_iy;
    nodes[start_index].g = 0.0;
    nodes[start_index].f = heuristicCost(start, goal);
    open_set.push({nodes[start_index].f, start_index});

    const std::vector<std::pair<int, int>> neighbors = allow_diagonal_
      ? std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}
      : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    const auto start_time = std::chrono::steady_clock::now();
    size_t expanded_nodes = 0;
    size_t footprint_rejections = 0;
    FootprintCheck first_footprint_rejection;
    Vec3 first_rejection_edge_start = Vec3::Zero();
    Vec3 first_rejection_edge_end = Vec3::Zero();
    Vec3 first_rejection_center = Vec3::Zero();
    int first_rejection_sample_index = -1;
    int first_rejection_sample_count = 0;
    double first_rejection_yaw = 0.0;
    double first_rejection_pitch = 0.0;
    double first_rejection_length = 0.0;
    bool has_first_footprint_rejection = false;
    while (!open_set.empty()) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() > planning_timeout_) {
        std::ostringstream out;
        out << "A* timed out after " << planning_timeout_
            << " s, expanded=" << expanded_nodes
            << ", footprint_rejections=" << footprint_rejections;
        if (has_first_footprint_rejection) {
          out << ", first_edge_rejection=" << footprintEdgeFailureToString(
            first_rejection_edge_start, first_rejection_edge_end, first_footprint_rejection,
            first_rejection_sample_index, first_rejection_sample_count, first_rejection_center,
            first_rejection_yaw, first_rejection_pitch, first_rejection_length);
        }
        setFailureReason(failure_reason, out.str());
        return false;
      }

      const QueueItem item = open_set.top();
      open_set.pop();
      SearchNode & current = nodes[item.index];
      if (current.closed) {
        continue;
      }
      current.closed = true;
      ++expanded_nodes;
      if (item.index == goal_index) {
        path = reconstructPath(nodes, goal_index, start, goal, resolution, min_ix, min_iy, ny);
        return true;
      }

      const Vec3 current_point = item.index == start_index ?
        start :
        toPoint(current.ix, current.iy);
      for (const auto & step : neighbors) {
        const int nix = current.ix + step.first;
        const int niy = current.iy + step.second;
        if (!inBounds(nix, niy)) {
          continue;
        }
        const int nindex = toIndex(nix, niy);
        SearchNode & neighbor = nodes[nindex];
        if (neighbor.closed) {
          continue;
        }
        const Vec3 neighbor_point = toPoint(nix, niy);
        FootprintCheck footprint_check;
        Vec3 failure_center = Vec3::Zero();
        int failure_sample_index = -1;
        int sample_count = 0;
        double line_yaw = 0.0;
        double line_pitch = 0.0;
        double line_length = 0.0;
        if (!checkFootprintLineFree(
            current_point, neighbor_point, &footprint_check, &failure_sample_index,
            &sample_count, &failure_center, &line_yaw, &line_pitch, &line_length))
        {
          ++footprint_rejections;
          if (!has_first_footprint_rejection) {
            first_footprint_rejection = footprint_check;
            first_rejection_edge_start = current_point;
            first_rejection_edge_end = neighbor_point;
            first_rejection_center = failure_center;
            first_rejection_sample_index = failure_sample_index;
            first_rejection_sample_count = sample_count;
            first_rejection_yaw = line_yaw;
            first_rejection_pitch = line_pitch;
            first_rejection_length = line_length;
            has_first_footprint_rejection = true;
          }
          continue;
        }
        const double edge_cost = resolution * std::hypot(
          static_cast<double>(step.first), static_cast<double>(step.second));
        const double transition_cost = edge_cost + nodeTraversalCost(
          current, nodes, step.first, step.second, current_point, neighbor_point);
        const double tentative_cost = current.g + transition_cost;
        if (!neighbor.seen || tentative_cost < neighbor.g) {
          neighbor.seen = true;
          neighbor.ix = nix;
          neighbor.iy = niy;
          neighbor.step_x = step.first;
          neighbor.step_y = step.second;
          neighbor.parent = item.index;
          neighbor.g = tentative_cost;
          neighbor.f = tentative_cost + heuristicCost(neighbor_point, goal);
          open_set.push({neighbor.f, nindex});
        }
      }
    }

    std::ostringstream out;
    out << "A* exhausted open set, expanded=" << expanded_nodes
        << ", footprint_rejections=" << footprint_rejections;
    if (has_first_footprint_rejection) {
      out << ", first_edge_rejection=" << footprintEdgeFailureToString(
        first_rejection_edge_start, first_rejection_edge_end, first_footprint_rejection,
        first_rejection_sample_index, first_rejection_sample_count, first_rejection_center,
        first_rejection_yaw, first_rejection_pitch, first_rejection_length);
    }
    setFailureReason(failure_reason, out.str());
    return false;
  }

  std::vector<Vec3> reconstructPath(
    const std::vector<SearchNode> & nodes,
    int goal_index,
    const Vec3 & start,
    const Vec3 & goal,
    double resolution,
    int min_ix,
    int min_iy,
    int ny) const
  {
    (void)min_ix;
    (void)min_iy;
    (void)ny;
    std::vector<Vec3> reversed;
    int index = goal_index;
    while (index >= 0) {
      const auto & node = nodes[index];
      const double x = (static_cast<double>(node.ix) + 0.5) * resolution;
      const double y = (static_cast<double>(node.iy) + 0.5) * resolution;
      reversed.emplace_back(x, y, interpolatedHeight(start, goal, x, y));
      index = node.parent;
    }
    std::reverse(reversed.begin(), reversed.end());
    if (!reversed.empty()) {
      reversed.front() = start;
      reversed.back() = goal;
    }
    return reversed;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double interpolatedHeight(const Vec3 & start, const Vec3 & goal, double x, double y)
  {
    const Vec3 delta = goal - start;
    const double denom = delta.x() * delta.x() + delta.y() * delta.y();
    double beta = 0.0;
    if (denom > 1e-6) {
      beta = ((x - start.x()) * delta.x() + (y - start.y()) * delta.y()) / denom;
      beta = std::clamp(beta, 0.0, 1.0);
    }
    return start.z() + beta * (goal.z() - start.z());
  }

  Vec3 pointWithLinearHeight(const Vec3 & point, const Vec3 & start, const Vec3 & goal) const
  {
    return Vec3(point.x(), point.y(), interpolatedHeight(start, goal, point.x(), point.y()));
  }

  std::vector<Vec3> projectPathToLinearHeight(
    const std::vector<Vec3> & input,
    const Vec3 & start,
    const Vec3 & goal) const
  {
    std::vector<Vec3> output;
    output.reserve(input.size());
    for (const Vec3 & point : input) {
      output.push_back(pointWithLinearHeight(point, start, goal));
    }
    if (!output.empty()) {
      output.front() = start;
      output.back() = goal;
    }
    return output;
  }

  double heuristicCost(const Vec3 & point, const Vec3 & goal) const
  {
    return horizontalDistance(point, goal) +
      cost_height_weight_ * std::abs(goal.z() - point.z());
  }

  double nodeTraversalCost(
    const SearchNode & current,
    const std::vector<SearchNode> & nodes,
    int step_x,
    int step_y,
    const Vec3 & current_point,
    const Vec3 & neighbor_point) const
  {
    double cost = cost_height_weight_ * std::abs(neighbor_point.z() - current_point.z());
    cost += mapNeighborhoodCost(neighbor_point);

    if (current.parent >= 0) {
      const SearchNode & parent = nodes[current.parent];
      const double previous_yaw = std::atan2(
        static_cast<double>(current.iy - parent.iy),
        static_cast<double>(current.ix - parent.ix));
      const double next_yaw = std::atan2(
        static_cast<double>(step_y),
        static_cast<double>(step_x));
      cost += cost_turn_weight_ * std::abs(normalizeAngle(next_yaw - previous_yaw));
    }
    return cost;
  }

  double mapNeighborhoodCost(const Vec3 & center) const
  {
    const int steps = std::max(0, static_cast<int>(std::ceil(cost_sample_radius_ / cost_sample_step_)));
    size_t sample_count = 0;
    size_t obstacle_count = 0;
    size_t unknown_count = 0;
    size_t inflated_count = 0;

    for (int ix = -steps; ix <= steps; ++ix) {
      for (int iy = -steps; iy <= steps; ++iy) {
        const double dx = static_cast<double>(ix) * cost_sample_step_;
        const double dy = static_cast<double>(iy) * cost_sample_step_;
        if (std::hypot(dx, dy) > cost_sample_radius_ + 1e-9) {
          continue;
        }
        ++sample_count;
        const Vec3 sample(center.x() + dx, center.y() + dy, center.z());
        const auto raw_type = map_->getGridType(sample);
        if (raw_type == rog_map::OCCUPIED || raw_type == rog_map::OUT_OF_MAP) {
          ++obstacle_count;
        } else if (raw_type == rog_map::UNKNOWN) {
          ++unknown_count;
        }
        if (map_->getInflatedGridType(sample) == rog_map::OCCUPIED) {
          ++inflated_count;
        }
      }
    }

    if (sample_count == 0) {
      return 0.0;
    }
    const double inv_samples = 1.0 / static_cast<double>(sample_count);
    return
      cost_obstacle_weight_ * static_cast<double>(obstacle_count) * inv_samples +
      cost_unknown_penalty_ * static_cast<double>(unknown_count) * inv_samples +
      cost_inflation_weight_ * static_cast<double>(inflated_count) * inv_samples;
  }

  static double horizontalDistance(const Vec3 & a, const Vec3 & b)
  {
    return std::hypot(a.x() - b.x(), a.y() - b.y());
  }

  double clampFootprintPitch(double pitch) const
  {
    if (!enable_pitch_footprint_) {
      return 0.0;
    }
    return std::clamp(pitch, -max_footprint_pitch_, max_footprint_pitch_);
  }

  double segmentPitch(const Vec3 & start, const Vec3 & goal) const
  {
    const double horizontal = horizontalDistance(start, goal);
    if (horizontal <= 1e-6) {
      return 0.0;
    }
    return clampFootprintPitch(std::atan2(goal.z() - start.z(), horizontal));
  }

  Vec3 footprintForward(double yaw, double pitch) const
  {
    pitch = clampFootprintPitch(pitch);
    return Vec3(std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch));
  }

  static Vec3 footprintLeft(double yaw)
  {
    return Vec3(-std::sin(yaw), std::cos(yaw), 0.0);
  }

  Vec3 footprintUp(double yaw, double pitch) const
  {
    pitch = clampFootprintPitch(pitch);
    return Vec3(-std::cos(yaw) * std::sin(pitch), -std::sin(yaw) * std::sin(pitch), std::cos(pitch));
  }

  bool checkFootprintLineFree(
    const Vec3 & start,
    const Vec3 & goal,
    FootprintCheck * failure_check = nullptr,
    int * failure_sample_index = nullptr,
    int * sample_count = nullptr,
    Vec3 * failure_center = nullptr,
    double * line_yaw = nullptr,
    double * line_pitch = nullptr,
    double * line_length = nullptr)
  {
    const Vec3 delta = goal - start;
    const double length = delta.norm();
    if (line_length != nullptr) {
      *line_length = length;
    }
    if (length < 1e-6) {
      const Vec3 center = pointWithLinearHeight(start, start, goal);
      const FootprintCheck check = checkFootprintFree(center, 0.0, 0.0);
      if (!check.free) {
        if (failure_check != nullptr) {
          *failure_check = check;
        }
        if (failure_sample_index != nullptr) {
          *failure_sample_index = 0;
        }
        if (sample_count != nullptr) {
          *sample_count = 1;
        }
        if (failure_center != nullptr) {
          *failure_center = center;
        }
        if (line_yaw != nullptr) {
          *line_yaw = 0.0;
        }
        if (line_pitch != nullptr) {
          *line_pitch = 0.0;
        }
        return false;
      }
      return true;
    }
    const double yaw = std::atan2(delta.y(), delta.x());
    const double pitch = segmentPitch(start, goal);
    const int steps = std::max(2, static_cast<int>(std::ceil(length / output_spacing_)));
    if (sample_count != nullptr) {
      *sample_count = steps + 1;
    }
    if (line_yaw != nullptr) {
      *line_yaw = yaw;
    }
    if (line_pitch != nullptr) {
      *line_pitch = pitch;
    }
    for (int i = 0; i <= steps; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
      const Vec3 sample = start + ratio * delta;
      const Vec3 center = pointWithLinearHeight(sample, start, goal);
      const FootprintCheck check = checkFootprintFree(center, yaw, pitch);
      if (!check.free) {
        if (failure_check != nullptr) {
          *failure_check = check;
        }
        if (failure_sample_index != nullptr) {
          *failure_sample_index = i;
        }
        if (failure_center != nullptr) {
          *failure_center = center;
        }
        return false;
      }
    }
    return true;
  }

  bool isFootprintLineFree(const Vec3 & start, const Vec3 & goal)
  {
    return checkFootprintLineFree(start, goal);
  }

  std::string footprintEdgeFailureToString(
    const Vec3 & edge_start,
    const Vec3 & edge_end,
    const FootprintCheck & check,
    int failure_sample_index,
    int sample_count,
    const Vec3 & failure_center,
    double line_yaw,
    double line_pitch,
    double line_length) const
  {
    std::ostringstream out;
    out << "edge_start=" << vecToString(edge_start)
        << ", edge_end=" << vecToString(edge_end)
        << ", line_length=" << std::fixed << std::setprecision(3) << line_length
        << ", yaw=" << line_yaw
        << ", pitch=" << line_pitch
        << ", failed_sample=" << failure_sample_index << "/" << std::max(0, sample_count - 1)
        << ", failed_center=" << vecToString(failure_center)
        << ", footprint=" << footprintFailureToString(check);
    return out.str();
  }

  bool isFootprintFree(const Vec3 & center, double yaw, double pitch)
  {
    return checkFootprintFree(center, yaw, pitch).free;
  }

  FootprintCheck checkFootprintFree(const Vec3 & center, double yaw, double pitch)
  {
    pitch = clampFootprintPitch(pitch);
    const std::array<double, 2> longitudinal_offsets{-cylinder_offset_, cylinder_offset_};
    const Vec3 forward = footprintForward(yaw, pitch);
    const Vec3 left = footprintLeft(yaw);
    const Vec3 up = footprintUp(yaw, pitch);
    const Vec3 adjusted_center = center + up * footprint_z_offset_;

    std::vector<std::pair<double, double>> radial_offsets;
    radial_offsets.emplace_back(0.0, 0.0);
    if (cylinder_radius_ > 1e-6) {
      for (int i = 0; i < radial_samples_; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(radial_samples_);
        radial_offsets.emplace_back(cylinder_radius_ * std::cos(angle), cylinder_radius_ * std::sin(angle));
      }
    }

    for (const double offset : longitudinal_offsets) {
      const Vec3 cylinder_center = adjusted_center + forward * offset;
      int radial_index = 0;
      for (double dz = -clearance_down_; dz <= clearance_up_ + 1e-6; dz += vertical_step_) {
        for (const auto & radial : radial_offsets) {
          const Vec3 query = cylinder_center + forward * radial.first + left * radial.second + up * dz;
          FootprintCheck check;
          check.query = query;
          check.longitudinal_offset = offset;
          check.pitch = pitch;
          check.dz = dz;
          check.radial_index = radial_index;
          if (!isMapPointFree(query, &check)) {
            return check;
          }
          ++radial_index;
        }
      }
    }
    return FootprintCheck{};
  }

  bool isMapPointFree(const Vec3 & point, FootprintCheck * debug = nullptr)
  {
    if (!map_->insideLocalMapPublic(point)) {
      if (debug != nullptr) {
        debug->free = false;
        debug->query = point;
        debug->grid_type = rog_map::OUT_OF_MAP;
        debug->reason = "sample outside local map bounds";
      }
      return false;
    }
    const auto grid_type = map_->getGridType(point);
    if (debug != nullptr) {
      debug->grid_type = grid_type;
    }
    if (grid_type == rog_map::OUT_OF_MAP || grid_type == rog_map::OCCUPIED) {
      if (debug != nullptr) {
        debug->free = false;
        debug->query = point;
        debug->reason = grid_type == rog_map::OUT_OF_MAP ?
          "raw grid reports OUT_OF_MAP" :
          "raw grid reports OCCUPIED";
      }
      return false;
    }
    if (unknown_as_occupied_ && grid_type == rog_map::UNKNOWN) {
      if (debug != nullptr) {
        debug->free = false;
        debug->query = point;
        debug->reason = "raw grid reports UNKNOWN and unknown_as_occupied is true";
      }
      return false;
    }
    return true;
  }

  FootprintCheck makeCenterOutsideFailure(const Vec3 & point)
  {
    FootprintCheck check;
    check.free = false;
    check.query = point;
    check.grid_type = rog_map::OUT_OF_MAP;
    check.reason = "center outside local map bounds";
    return check;
  }

  std::string localMapBoundsString()
  {
    rog_map::Vec3f map_min;
    rog_map::Vec3f map_max;
    map_->getLocalMapBounds(map_min, map_max);
    return "[" + vecToString(map_min.cast<double>()) + ", " +
      vecToString(map_max.cast<double>()) + "]";
  }

  std::string makePlanFailureReason(
    const std::string & summary,
    const Vec3 & start,
    const Vec3 & goal,
    double start_yaw,
    double goal_yaw,
    const std::string & detail)
  {
    std::ostringstream out;
    out << summary
        << "; start=" << vecToString(start)
        << ", goal=" << vecToString(goal)
        << ", horizontal_distance=" << std::fixed << std::setprecision(3)
        << horizontalDistance(start, goal)
        << ", dz=" << goal.z() - start.z()
        << ", start_yaw=" << start_yaw
        << ", goal_yaw=" << goal_yaw
        << ", local_map_bounds=" << localMapBoundsString();
    if (!detail.empty()) {
      out << ", detail=" << detail;
    }
    return out.str();
  }

  static void setFailureReason(std::string * target, const std::string & reason)
  {
    if (target != nullptr) {
      *target = reason;
    }
  }

  bool projectNearestFreePose(
    const Vec3 & original,
    double yaw,
    double pitch,
    const Vec3 & start,
    const Vec3 & goal,
    Vec3 & projected,
    std::string * failure_reason = nullptr)
  {
    if (!projection_enable_) {
      setFailureReason(failure_reason, "projection disabled");
      return false;
    }

    const int steps = std::max(1, static_cast<int>(std::ceil(projection_radius_ / projection_step_)));
    double best_distance = std::numeric_limits<double>::infinity();
    Vec3 best = original;
    FootprintCheck first_rejection;
    bool has_first_rejection = false;

    for (int ix = -steps; ix <= steps; ++ix) {
      for (int iy = -steps; iy <= steps; ++iy) {
        const double dx = static_cast<double>(ix) * projection_step_;
        const double dy = static_cast<double>(iy) * projection_step_;
        const double horizontal_distance = std::hypot(dx, dy);
        if (horizontal_distance > projection_radius_ || horizontal_distance >= best_distance) {
          continue;
        }

        const Vec3 candidate_xy(original.x() + dx, original.y() + dy, original.z());
        const Vec3 candidate = pointWithLinearHeight(candidate_xy, start, goal);
        if (!map_->insideLocalMapPublic(candidate)) {
          continue;
        }
        const FootprintCheck check = checkFootprintFree(candidate, yaw, pitch);
        if (check.free) {
          best_distance = horizontal_distance;
          best = candidate;
        } else if (!has_first_rejection) {
          first_rejection = check;
          has_first_rejection = true;
        }
      }
    }

    if (!std::isfinite(best_distance)) {
      std::ostringstream out;
      out << "no free pose within " << projection_radius_ << " m";
      if (has_first_rejection) {
        out << ", first_rejection=" << footprintFailureToString(first_rejection);
      }
      setFailureReason(failure_reason, out.str());
      return false;
    }

    projected = best;
    return true;
  }

  static Vec3 closestPointOnSegment(const Vec3 & point, const Vec3 & start, const Vec3 & end)
  {
    const Vec3 segment = end - start;
    const double length_sq = segment.squaredNorm();
    if (length_sq <= 1e-12) {
      return start;
    }
    const double ratio = std::clamp((point - start).dot(segment) / length_sq, 0.0, 1.0);
    return start + ratio * segment;
  }

  static Vec3 closestPointOnPath(const Vec3 & point, const std::vector<Vec3> & path)
  {
    if (path.empty()) {
      return point;
    }
    if (path.size() == 1) {
      return path.front();
    }

    double best_distance = std::numeric_limits<double>::infinity();
    Vec3 best_point = path.front();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const Vec3 candidate = closestPointOnSegment(point, path[i], path[i + 1]);
      const double candidate_distance = (point - candidate).norm();
      if (candidate_distance < best_distance) {
        best_distance = candidate_distance;
        best_point = candidate;
      }
    }
    return best_point;
  }

  Vec3 limitDeviationToPath(const Vec3 & point, const std::vector<Vec3> & reference_path) const
  {
    if (path_smoothing_max_deviation_ <= 0.0) {
      return closestPointOnPath(point, reference_path);
    }
    const Vec3 closest = closestPointOnPath(point, reference_path);
    const Vec3 delta = point - closest;
    const double deviation = delta.norm();
    if (deviation <= path_smoothing_max_deviation_ || deviation <= 1e-12) {
      return point;
    }
    return closest + (path_smoothing_max_deviation_ / deviation) * delta;
  }

  std::vector<Vec3> removeShortSegments(const std::vector<Vec3> & input) const
  {
    if (input.size() <= 2 || smoothing_min_point_spacing_ <= 0.0) {
      return input;
    }

    std::vector<Vec3> output;
    output.reserve(input.size());
    output.push_back(input.front());
    for (size_t i = 1; i + 1 < input.size(); ++i) {
      if (horizontalDistance(output.back(), input[i]) >= smoothing_min_point_spacing_) {
        output.push_back(input[i]);
      }
    }
    if (horizontalDistance(output.back(), input.back()) > 1e-9) {
      output.push_back(input.back());
    } else {
      output.back() = input.back();
    }
    return output;
  }

  std::vector<Vec3> removeCollinearPoints(const std::vector<Vec3> & input) const
  {
    if (input.size() <= 2 || smoothing_collinear_angle_threshold_ <= 0.0) {
      return input;
    }

    std::vector<Vec3> output;
    output.reserve(input.size());
    output.push_back(input.front());
    for (size_t i = 1; i + 1 < input.size(); ++i) {
      const Vec3 & previous = output.back();
      const Vec3 & current = input[i];
      const Vec3 & following = input[i + 1];
      if (horizontalDistance(previous, current) <= 1e-9 ||
        horizontalDistance(current, following) <= 1e-9)
      {
        continue;
      }
      const double previous_yaw = std::atan2(current.y() - previous.y(), current.x() - previous.x());
      const double next_yaw = std::atan2(following.y() - current.y(), following.x() - current.x());
      if (std::abs(normalizeAngle(next_yaw - previous_yaw)) > smoothing_collinear_angle_threshold_) {
        output.push_back(current);
      }
    }
    if (horizontalDistance(output.back(), input.back()) > 1e-9) {
      output.push_back(input.back());
    } else {
      output.back() = input.back();
    }
    return output;
  }

  std::vector<Vec3> cleanPathGeometry(const std::vector<Vec3> & input) const
  {
    return removeCollinearPoints(removeShortSegments(input));
  }

  std::vector<Vec3> smoothPath(
    const std::vector<Vec3> & input,
    const Vec3 & start,
    const Vec3 & goal) const
  {
    if (input.size() <= 2 || path_smoothing_iterations_ <= 0) {
      return input;
    }

    const std::vector<Vec3> original = projectPathToLinearHeight(
      resamplePath(cleanPathGeometry(input)), start, goal);
    std::vector<Vec3> smoothed = original;
    for (int iteration = 0; iteration < path_smoothing_iterations_; ++iteration) {
      std::vector<Vec3> updated = smoothed;
      for (size_t i = 1; i + 1 < smoothed.size(); ++i) {
        Vec3 candidate = smoothed[i];
        candidate.x() +=
          path_smoothing_data_weight_ * (original[i].x() - smoothed[i].x()) +
          path_smoothing_smooth_weight_ *
          (smoothed[i - 1].x() + smoothed[i + 1].x() - 2.0 * smoothed[i].x());
        candidate.y() +=
          path_smoothing_data_weight_ * (original[i].y() - smoothed[i].y()) +
          path_smoothing_smooth_weight_ *
          (smoothed[i - 1].y() + smoothed[i + 1].y() - 2.0 * smoothed[i].y());
        candidate.z() = interpolatedHeight(start, goal, candidate.x(), candidate.y());
        updated[i] = limitDeviationToPath(candidate, original);
      }
      smoothed = updated;
    }
    smoothed.front() = original.front();
    smoothed.back() = original.back();
    return projectPathToLinearHeight(resamplePath(smoothed), start, goal);
  }

  bool pathCollisionFree(
    const std::vector<Vec3> & points,
    std::string * failure_reason = nullptr,
    const std::vector<Vec3> * reference_path = nullptr,
    bool enforce_smoothing_constraints = false)
  {
    if (points.size() < 2) {
      setFailureReason(failure_reason, "path has fewer than two points");
      return false;
    }
    if (enforce_smoothing_constraints && reference_path != nullptr && !reference_path->empty()) {
      for (size_t i = 0; i < points.size(); ++i) {
        const double deviation = (points[i] - closestPointOnPath(points[i], *reference_path)).norm();
        if (deviation > path_smoothing_max_deviation_ + 1e-9) {
          std::ostringstream out;
          out << "point " << i << " deviates " << std::fixed << std::setprecision(3)
              << deviation << " m from original path";
          setFailureReason(failure_reason, out.str());
          return false;
        }
      }
    }

    for (size_t i = 0; i + 1 < points.size(); ++i) {
      const double horizontal = horizontalDistance(points[i], points[i + 1]);
      const double dz = std::abs(points[i + 1].z() - points[i].z());
      if (
        enforce_smoothing_constraints &&
        smoothing_max_segment_slope_ > 0.0 &&
        ((horizontal <= 1e-6 && dz > 1e-6) ||
        (horizontal > 1e-6 && dz / horizontal > smoothing_max_segment_slope_)))
      {
        std::ostringstream out;
        out << "segment " << i << " -> " << (i + 1)
            << " slope " << std::fixed << std::setprecision(3)
            << (horizontal > 1e-6 ? dz / horizontal : std::numeric_limits<double>::infinity())
            << " exceeds " << smoothing_max_segment_slope_;
        setFailureReason(failure_reason, out.str());
        return false;
      }
      FootprintCheck footprint_failure;
      int failure_sample_index = -1;
      int sample_count = 0;
      Vec3 failure_center = Vec3::Zero();
      double line_yaw = 0.0;
      double line_pitch = 0.0;
      double line_length = 0.0;
      if (!checkFootprintLineFree(
          points[i], points[i + 1], &footprint_failure, &failure_sample_index,
          &sample_count, &failure_center, &line_yaw, &line_pitch, &line_length))
      {
        std::ostringstream out;
        out << "segment " << i << " -> " << (i + 1)
            << " is not collision-free, start=" << vecToString(points[i])
            << ", end=" << vecToString(points[i + 1])
            << ", line_length=" << std::fixed << std::setprecision(3) << line_length
            << ", horizontal_distance=" << horizontal
            << ", segment_dz=" << (points[i + 1].z() - points[i].z())
            << ", yaw=" << line_yaw
            << ", pitch=" << line_pitch
            << ", failed_sample=" << failure_sample_index << "/" << std::max(0, sample_count - 1)
            << ", failed_center=" << vecToString(failure_center)
            << ", footprint=" << footprintFailureToString(footprint_failure);
        setFailureReason(failure_reason, out.str());
        return false;
      }
    }
    if (enforce_smoothing_constraints && smoothing_max_turn_angle_ > 0.0 && points.size() > 2) {
      for (size_t i = 1; i + 1 < points.size(); ++i) {
        const double previous_yaw = std::atan2(
          points[i].y() - points[i - 1].y(),
          points[i].x() - points[i - 1].x());
        const double next_yaw = std::atan2(
          points[i + 1].y() - points[i].y(),
          points[i + 1].x() - points[i].x());
        const double turn = std::abs(normalizeAngle(next_yaw - previous_yaw));
        if (turn > smoothing_max_turn_angle_) {
          std::ostringstream out;
          out << "turn at point " << i << " is " << std::fixed << std::setprecision(3)
              << turn << " rad, exceeds " << smoothing_max_turn_angle_;
          setFailureReason(failure_reason, out.str());
          return false;
        }
      }
    }
    return true;
  }

  std::vector<Vec3> shortcutPath(const std::vector<Vec3> & input)
  {
    if (input.size() <= 2) {
      return input;
    }
    std::vector<Vec3> output;
    size_t i = 0;
    output.push_back(input.front());
    while (i + 1 < input.size()) {
      size_t best = i + 1;
      for (size_t j = input.size() - 1; j > i + 1; --j) {
        if (isFootprintLineFree(input[i], input[j])) {
          best = j;
          break;
        }
      }
      output.push_back(input[best]);
      i = best;
    }
    return output;
  }

  std::vector<Vec3> resamplePath(const std::vector<Vec3> & input) const
  {
    if (input.size() <= 1) {
      return input;
    }

    std::vector<double> cumulative;
    cumulative.reserve(input.size());
    cumulative.push_back(0.0);
    for (size_t i = 1; i < input.size(); ++i) {
      cumulative.push_back(cumulative.back() + horizontalDistance(input[i - 1], input[i]));
    }

    const double total_length = cumulative.back();
    if (total_length <= 1e-9) {
      return {input.front(), input.back()};
    }

    std::vector<Vec3> output;
    output.push_back(input.front());
    size_t segment_index = 1;
    for (double target = output_spacing_; target < total_length; target += output_spacing_) {
      while (segment_index + 1 < cumulative.size() && cumulative[segment_index] < target) {
        ++segment_index;
      }
      const double segment_start = cumulative[segment_index - 1];
      const double segment_end = cumulative[segment_index];
      const double segment_length = segment_end - segment_start;
      if (segment_length <= 1e-9) {
        continue;
      }
      const double ratio = (target - segment_start) / segment_length;
      output.push_back(input[segment_index - 1] + ratio * (input[segment_index] - input[segment_index - 1]));
    }
    if ((output.back() - input.back()).norm() > 1e-9) {
      output.push_back(input.back());
    } else {
      output.back() = input.back();
    }
    return output;
  }

  nav_msgs::msg::Path makePath(
    const std::vector<Vec3> & points,
    const geometry_msgs::msg::Quaternion & final_orientation) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();
    path.poses.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x();
      pose.pose.position.y = points[i].y();
      pose.pose.position.z = points[i].z();
      if (i + 1 < points.size()) {
        const Vec3 delta = points[i + 1] - points[i];
        pose.pose.orientation = yawToQuaternion(std::atan2(delta.y(), delta.x()));
      } else {
        pose.pose.orientation = final_orientation;
      }
      path.poses.push_back(pose);
    }
    return path;
  }

  void clearFootprintMarkers() const
  {
    if (!footprint_marker_pub_) {
      return;
    }
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = global_frame_;
    clear_marker.header.stamp = node_->now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    footprint_marker_pub_->publish(markers);
  }

  static double pathPoseYaw(const nav_msgs::msg::Path & path, size_t index)
  {
    const auto pointAt = [&](size_t i) {
      return Vec3(
        path.poses[i].pose.position.x,
        path.poses[i].pose.position.y,
        path.poses[i].pose.position.z);
    };

    if (index + 1 < path.poses.size()) {
      const Vec3 delta = pointAt(index + 1) - pointAt(index);
      return std::atan2(delta.y(), delta.x());
    }
    if (index > 0) {
      const Vec3 delta = pointAt(index) - pointAt(index - 1);
      return std::atan2(delta.y(), delta.x());
    }
    return poseYaw(path.poses[index].pose);
  }

  double pathPosePitch(const nav_msgs::msg::Path & path, size_t index) const
  {
    const auto pointAt = [&](size_t i) {
      return Vec3(
        path.poses[i].pose.position.x,
        path.poses[i].pose.position.y,
        path.poses[i].pose.position.z);
    };

    if (index + 1 < path.poses.size()) {
      return segmentPitch(pointAt(index), pointAt(index + 1));
    }
    if (index > 0) {
      return segmentPitch(pointAt(index - 1), pointAt(index));
    }
    return 0.0;
  }

  void colorFootprintMarker(
    visualization_msgs::msg::Marker & marker,
    size_t path_index,
    size_t path_size) const
  {
    marker.color.a = visualization_footprint_marker_alpha_;
    if (path_index == 0) {
      marker.color.r = 0.10;
      marker.color.g = 0.85;
      marker.color.b = 0.25;
    } else if (path_index + 1 >= path_size) {
      marker.color.r = 1.00;
      marker.color.g = 0.80;
      marker.color.b = 0.10;
    } else {
      marker.color.r = 0.10;
      marker.color.g = 0.45;
      marker.color.b = 1.00;
    }
  }

  void publishFootprintMarkers(const nav_msgs::msg::Path & path) const
  {
    if (!footprint_marker_pub_) {
      return;
    }
    if (path.poses.empty()) {
      clearFootprintMarkers();
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = path.header;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    int marker_id = 0;
    const double height = std::max(0.01, clearance_up_ + clearance_down_);
    const double z_offset = footprint_z_offset_ + 0.5 * (clearance_up_ - clearance_down_);
    for (size_t i = 0; i < path.poses.size(); i += static_cast<size_t>(visualization_footprint_marker_stride_)) {
      appendFootprintMarkers(path, i, marker_id, height, z_offset, markers);
    }
    if ((path.poses.size() - 1) % static_cast<size_t>(visualization_footprint_marker_stride_) != 0) {
      appendFootprintMarkers(path, path.poses.size() - 1, marker_id, height, z_offset, markers);
    }

    footprint_marker_pub_->publish(markers);
  }

  void appendFootprintMarkers(
    const nav_msgs::msg::Path & path,
    size_t path_index,
    int & marker_id,
    double height,
    double z_offset,
    visualization_msgs::msg::MarkerArray & markers) const
  {
    const auto & pose = path.poses[path_index].pose;
    const double yaw = pathPoseYaw(path, path_index);
    const double pitch = pathPosePitch(path, path_index);
    const Vec3 forward = footprintForward(yaw, pitch);
    const Vec3 up = footprintUp(yaw, pitch);
    const std::array<double, 2> longitudinal_offsets{-cylinder_offset_, cylinder_offset_};

    for (const double offset : longitudinal_offsets) {
      visualization_msgs::msg::Marker marker;
      marker.header = path.header;
      marker.ns = "footprint_cylinders";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      const Vec3 center(
        pose.position.x + forward.x() * offset + up.x() * z_offset,
        pose.position.y + forward.y() * offset + up.y() * z_offset,
        pose.position.z + forward.z() * offset + up.z() * z_offset);
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker.pose.orientation = yawPitchToQuaternion(yaw, pitch);
      marker.scale.x = 2.0 * cylinder_radius_;
      marker.scale.y = 2.0 * cylinder_radius_;
      marker.scale.z = height;
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 0;
      colorFootprintMarker(marker, path_index, path.poses.size());
      markers.markers.push_back(marker);
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rog_map::ROGMap> map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<PlanToGoal>::SharedPtr action_server_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_marker_pub_;

  std::string global_frame_;
  std::string robot_frame_;
  std::string action_name_;
  std::string path_topic_;
  double replan_rate_{2.0};
  double planning_timeout_{0.2};
  double search_resolution_{0.0};
  double search_margin_{3.0};
  double output_spacing_{0.15};
  bool unknown_as_occupied_{false};
  bool require_map_ready_{true};
  bool allow_diagonal_{true};
  bool enable_path_smoothing_{false};
  int path_smoothing_iterations_{20};
  double path_smoothing_data_weight_{0.35};
  double path_smoothing_smooth_weight_{0.20};
  double path_smoothing_max_deviation_{0.20};
  double smoothing_min_point_spacing_{0.10};
  double smoothing_collinear_angle_threshold_{0.08};
  double smoothing_max_segment_slope_{0.40};
  double smoothing_max_turn_angle_{1.20};
  double cost_height_weight_{1.0};
  double cost_unknown_penalty_{1.0};
  double cost_obstacle_weight_{4.0};
  double cost_inflation_weight_{2.0};
  double cost_sample_radius_{0.35};
  double cost_sample_step_{0.10};
  double cost_turn_weight_{0.2};
  bool projection_enable_{true};
  double projection_radius_{0.45};
  double projection_step_{0.10};
  double cylinder_radius_{0.28};
  double cylinder_offset_{0.23};
  double clearance_up_{0.45};
  double clearance_down_{0.35};
  double vertical_step_{0.15};
  int radial_samples_{8};
  bool enable_pitch_footprint_{true};
  double max_footprint_pitch_{0.70};
  double footprint_z_offset_{0.0};
  bool visualization_enable_footprint_markers_{true};
  std::string visualization_footprint_marker_topic_{"/rog_local_planner/footprint_markers"};
  int visualization_footprint_marker_stride_{2};
  double visualization_footprint_marker_alpha_{0.28};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rclcpp::Node>("rog_local_planner");
    StdoutRosLogger rog_map_stdout_logger(node);
    auto planner = std::make_shared<RogLocalPlanner>(node);
    (void)planner;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}
