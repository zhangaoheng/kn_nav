// ============================================================================
// waypoint_utils.cpp
// ----------------------------------------------------------------------------
// PCT 全局路径 -> SCAN 局部规划 waypoint 路线的采样工具实现。
//
// 职责：
//   * 把 PCT 发布的稠密全局路径（/pct_path）按三维弧长等间距重采样，
//     生成 SCAN-Planner 可跟踪的 waypoint 序列（/scan_planner/waypoints）。
//   * 计算路径内容签名，供 pct_scan_coordinator 判断新路径与当前路线
//     是否相同，避免重复发布。
//
// 数据流：pct_scan_coordinator::pathCallback -> sampleWaypoints()
//       -> 重采样路径 + 签名 -> waypoints_pub_ 发布。
//
// 关键约定：
//   * 输入路径必须在全局坐标系（默认 map）下，且所有点坐标有限。
//   * 采样间距 spacing 必须为正；z_offset 用于统一抬高/压低 waypoint 高度。
//   * 输出路径末尾始终保留输入路径的终点（含其朝向），保证导航终点唯一。
// ============================================================================

#include "pct_scan_navigation/waypoint_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace
{

constexpr double kEpsilon = 1e-9;

// 检查三维点坐标是否全部有限（NaN/Inf 会导致采样结果无效）。
bool finitePoint(const geometry_msgs::msg::Point &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// 两点三维欧氏距离（XY 用 hypot 减少溢出，Z 直接相减）。
double distance3D(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

// 把单个 64 位值混入哈希（FNV-1a 风格：异或后乘以素数），
// 用于把路径各点坐标/朝向/点数折叠成一个内容签名。
void mixHash(std::uint64_t &hash, std::int64_t value)
{
  hash ^= static_cast<std::uint64_t>(value);
  hash *= 1099511628211ULL;
}

}  // namespace

namespace pct_scan_navigation
{

// 将全局路径按三维弧长等间距重采样为 waypoint 路线。
// 输入：input(全局路径)、spacing(采样间距)、z_offset(统一高度偏移)。
// 输出：output(重采样路径，末尾保留原终点)、signature(路径内容签名)、
//       reason(失败原因)。
// 返回：路径有效且采样成功时为 true。
bool sampleWaypoints(
    const nav_msgs::msg::Path &input,
    const std::string &global_frame,
    double spacing,
    double z_offset,
    nav_msgs::msg::Path &output,
    std::uint64_t &signature,
    std::string &reason)
{
  output = nav_msgs::msg::Path();
  signature = 0;
  reason.clear();

  if (input.poses.empty())
  {
    reason = "path is empty";
    return false;
  }
  if (input.header.frame_id != global_frame)
  {
    reason = "path frame must be " + global_frame;
    return false;
  }
  if (!std::isfinite(spacing) || spacing <= 0.0 || !std::isfinite(z_offset))
  {
    reason = "spacing must be positive and z offset must be finite";
    return false;
  }

  // 签名初值采用 FNV offset basis；随后对每个点的 xyz(毫米精度)、
  // 终点四元数、点数依次 mixHash，保证"内容变则签名变"（见测试）。
  signature = 1469598103934665603ULL;
  for (const auto &pose : input.poses)
  {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != global_frame) ||
        !finitePoint(pose.pose.position))
    {
      reason = "path contains an invalid pose or frame";
      return false;
    }
    mixHash(signature, std::llround(pose.pose.position.x * 1000.0));
    mixHash(signature, std::llround(pose.pose.position.y * 1000.0));
    mixHash(signature, std::llround(pose.pose.position.z * 1000.0));
  }
  const auto &final_orientation = input.poses.back().pose.orientation;
  if (!std::isfinite(final_orientation.x) || !std::isfinite(final_orientation.y) ||
      !std::isfinite(final_orientation.z) || !std::isfinite(final_orientation.w))
  {
    reason = "final orientation is invalid";
    return false;
  }
  mixHash(signature, std::llround(final_orientation.x * 1000.0));
  mixHash(signature, std::llround(final_orientation.y * 1000.0));
  mixHash(signature, std::llround(final_orientation.z * 1000.0));
  mixHash(signature, std::llround(final_orientation.w * 1000.0));
  mixHash(signature, static_cast<std::int64_t>(input.poses.size()));

  std::vector<double> segment_lengths;
  segment_lengths.reserve(input.poses.size() > 1 ? input.poses.size() - 1 : 0);
  double total_length = 0.0;
  for (std::size_t i = 1; i < input.poses.size(); ++i)
  {
    const double length = distance3D(
        input.poses[i - 1].pose.position, input.poses[i].pose.position);
    segment_lengths.push_back(length);
    total_length += length;
  }

  output.header = input.header;
  output.header.frame_id = global_frame;
  std::size_t segment_index = 0;
  double segment_start_distance = 0.0;
  // 弧长等间距采样主循环：从 spacing 起每隔 spacing 取一个采样点，
  // 用"段起点距离 + 段内比例"在折线路径上定位；循环条件留出 epsilon
  // 余量，使恰好落在整米处的终点不会被重复采样。
  for (double target_distance = spacing;
       target_distance < total_length - kEpsilon;
       target_distance += spacing)
  {
    while (segment_index < segment_lengths.size() &&
           segment_start_distance + segment_lengths[segment_index] < target_distance - kEpsilon)
    {
      segment_start_distance += segment_lengths[segment_index];
      ++segment_index;
    }
    // 跳过零长度段（相邻重复点），避免除零；采样点始终落在有效段内。
    while (segment_index < segment_lengths.size() && segment_lengths[segment_index] <= kEpsilon)
    {
      segment_start_distance += segment_lengths[segment_index];
      ++segment_index;
    }
    if (segment_index >= segment_lengths.size())
      break;

    const auto &start = input.poses[segment_index].pose.position;
    const auto &end = input.poses[segment_index + 1].pose.position;
    const double ratio = std::clamp(
        (target_distance - segment_start_distance) / segment_lengths[segment_index], 0.0, 1.0);

    geometry_msgs::msg::PoseStamped sample;
    sample.header = output.header;
    sample.pose.position.x = start.x + ratio * (end.x - start.x);
    sample.pose.position.y = start.y + ratio * (end.y - start.y);
    sample.pose.position.z = start.z + ratio * (end.z - start.z) + z_offset;
    sample.pose.orientation.w = 1.0;
    output.poses.push_back(sample);
  }

  // 末尾始终追加输入路径的原始终点（含其朝向，仅叠加 z_offset），
  // 保证 SCAN 的最终目标位姿与全局路径一致。
  geometry_msgs::msg::PoseStamped final_pose = input.poses.back();
  final_pose.header = output.header;
  final_pose.pose.position.z += z_offset;
  output.poses.push_back(final_pose);
  return true;
}

}  // namespace pct_scan_navigation
