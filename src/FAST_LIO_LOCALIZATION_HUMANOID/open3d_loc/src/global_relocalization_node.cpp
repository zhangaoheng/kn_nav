#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <fstream>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <yaml-cpp/yaml.h>

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_loc/srv/global_relocalize.hpp"
#include "pct_scan_navigation/msg/localization_status.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

bool finiteTransform(const Eigen::Matrix4d & transform)
{
  if (!transform.allFinite()) return false;

  if ((transform.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-6) {
    return false;
  }

  const Eigen::Matrix3d rotation =
    transform.block<3, 3>(0, 0);

  return std::abs(rotation.determinant() - 1.0) < 0.1 &&
         (rotation.transpose() * rotation -
         Eigen::Matrix3d::Identity()).norm() < 0.2;
}

double transformYaw(const Eigen::Matrix4d & transform)
{
  return std::atan2(transform(1, 0), transform(0, 0));
}

}  // namespace


class GlobalRelocalizationNode : public rclcpp::Node
{
public:
  using GlobalRelocalize =
    open3d_loc::srv::GlobalRelocalize;

  using LocalizationStatus =
    pct_scan_navigation::msg::LocalizationStatus;

  GlobalRelocalizationNode()
  : Node("global_relocalization_node")
  {
    enabled_ =
      declare_parameter<bool>(
      "enabled", false);

    map_topic_ =
      declare_parameter<std::string>(
      "map_topic", "/map_3d");

    scan_topic_ =
      declare_parameter<std::string>(
      "scan_topic", "/scan_base_link");

    status_topic_ =
      declare_parameter<std::string>(
      "status_topic", "/localization_status");

    initialpose_topic_ =
      declare_parameter<std::string>(
      "initialpose_topic", "/initialpose");

    keyframe_database_root_ =
      declare_parameter<std::string>(
      "keyframe_database_root", "");

    keyframe_fpfh_radius_ =
      declare_parameter<double>(
      "keyframe_fpfh_radius", 2.0);

    keyframe_ransac_distance_ =
      declare_parameter<double>(
      "keyframe_ransac_distance", 1.0);

    keyframe_gicp_distance_ =
      declare_parameter<double>(
      "keyframe_gicp_distance", 0.6);

    keyframe_ransac_iterations_ =
      declare_parameter<int>(
      "keyframe_ransac_iterations", 40000);

    max_keyframe_tilt_delta_deg_ =
      declare_parameter<double>(
      "max_keyframe_tilt_delta_deg", 20.0);

    descriptor_rings_ =
      declare_parameter<int>(
      "descriptor_rings", 20);

    descriptor_sectors_ =
      declare_parameter<int>(
      "descriptor_sectors", 60);

    descriptor_radius_ =
      declare_parameter<double>(
      "descriptor_radius", 20.0);

    tile_size_ =
      declare_parameter<double>(
      "tile_size", 24.0);

    tile_height_ =
      declare_parameter<double>(
      "tile_height", 3.0);

    min_tile_points_ =
      declare_parameter<int>(
      "min_tile_points", 500);

    max_candidates_ =
      declare_parameter<int>(
      "max_candidates", 5);

    voxel_size_ =
      declare_parameter<double>(
      "voxel_size", 0.4);

    min_scan_points_ =
      declare_parameter<int>(
      "min_scan_points", 300);

    min_target_points_ =
      declare_parameter<int>(
      "min_target_points", 800);

    min_descriptor_score_ =
      declare_parameter<double>(
      "min_descriptor_score", 0.15);

    min_fitness_ =
      declare_parameter<double>(
      "min_fitness", 0.70);

    max_inlier_rmse_ =
      declare_parameter<double>(
      "max_inlier_rmse", 0.4);

    max_abs_z_ =
      declare_parameter<double>(
      "max_abs_z", 5.0);

    max_scan_age_s_ =
      declare_parameter<double>(
      "max_scan_age_s", 2.0);

    target_half_xy_ =
      declare_parameter<double>(
      "target_half_xy", 12.0);

    target_half_z_ =
      declare_parameter<double>(
      "target_half_z", 2.0);

    ground_plane_distance_ =
      declare_parameter<double>(
      "ground_plane_distance", 0.15);

    min_fitness_margin_ =
      declare_parameter<double>(
      "min_fitness_margin", 0.05);

    ambiguity_translation_m_ =
      declare_parameter<double>(
      "ambiguity_translation_m", 1.0);

    ambiguity_yaw_deg_ =
      declare_parameter<double>(
      "ambiguity_yaw_deg", 10.0);

    if (
      descriptor_rings_ < 4 ||
      descriptor_sectors_ < 12 ||
      descriptor_radius_ <= 0.0 ||
      tile_size_ <= 0.0 ||
      tile_height_ <= 0.0 ||
      voxel_size_ <= 0.0 ||
      max_candidates_ < 1 ||
      target_half_xy_ <= 0.0 ||
      target_half_z_ <= 0.0 ||
      ground_plane_distance_ < 0.0 ||
      min_fitness_margin_ < 0.0 ||
      ambiguity_translation_m_ < 0.0 ||
      ambiguity_yaw_deg_ < 0.0 ||
      keyframe_fpfh_radius_ <= voxel_size_ ||
      keyframe_ransac_distance_ <= 0.0 ||
      keyframe_gicp_distance_ <= 0.0 ||
      keyframe_ransac_iterations_ < 100 ||
      max_keyframe_tilt_delta_deg_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid global relocalization parameters");
    }

    // These are safety floors, not tuning defaults.  An older external map
    // configuration must not silently restore the permissive values that
    // accepted visibly wrong poses.  YAML may still make either check stricter.
    if (min_fitness_ < 0.70) {
      RCLCPP_WARN(
        get_logger(), "min_fitness %.3f is unsafe; clamped to 0.700", min_fitness_);
      min_fitness_ = 0.70;
    }
    if (max_inlier_rmse_ > 0.40) {
      RCLCPP_WARN(
        get_logger(), "max_inlier_rmse %.3f is unsafe; clamped to 0.400", max_inlier_rmse_);
      max_inlier_rmse_ = 0.40;
    }

    auto map_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1)).
      reliable().
      transient_local();

    map_sub_ =
      create_subscription<
      sensor_msgs::msg::PointCloud2>(
      map_topic_,
      map_qos,
      std::bind(
        &GlobalRelocalizationNode::mapCallback,
        this,
        std::placeholders::_1));

    scan_sub_ =
      create_subscription<
      sensor_msgs::msg::PointCloud2>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &GlobalRelocalizationNode::scanCallback,
        this,
        std::placeholders::_1));

    status_sub_ =
      create_subscription<
      LocalizationStatus>(
      status_topic_,
      10,
      std::bind(
        &GlobalRelocalizationNode::statusCallback,
        this,
        std::placeholders::_1));

    initialpose_pub_ =
      create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_,
      10);

    candidate_pose_pub_ =
      create_publisher<
      geometry_msgs::msg::PoseStamped>(
      "~/candidate_pose",
      10);

    aligned_cloud_pub_ =
      create_publisher<
      sensor_msgs::msg::PointCloud2>(
      "~/aligned_cloud",
      1);

    service_ =
      create_service<
      GlobalRelocalize>(
      "~/trigger",
      std::bind(
        &GlobalRelocalizationNode::handleTrigger,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Global relocalization ready (enabled=%s). "
      "It never controls the robot and only applies a pose when explicitly requested.",
      enabled_ ? "true" : "false");
  }


private:
  struct Descriptor
  {
    std::vector<double> cells;
    std::vector<double> ring_key;
  };


  struct Tile
  {
    Eigen::Vector3d center{
      Eigen::Vector3d::Zero()};

    std::shared_ptr<
      open3d::geometry::PointCloud> cloud;

    Descriptor descriptor;

    Eigen::Matrix4d pose{
      Eigen::Matrix4d::Identity()};

    bool is_keyframe{false};
  };


  struct Candidate
  {
    size_t tile_index{0};
    double score{-1.0};
    int sector_shift{0};
  };


  struct Match
  {
    bool valid{false};

    Eigen::Matrix4d transform{
      Eigen::Matrix4d::Identity()};

    double descriptor_score{0.0};
    double fitness{0.0};

    double rmse{
      std::numeric_limits<double>::infinity()};

    Eigen::Matrix4d second_transform{
      Eigen::Matrix4d::Identity()};

    double second_fitness{-1.0};

    double second_rmse{
      std::numeric_limits<double>::infinity()};

    std::shared_ptr<
      open3d::geometry::PointCloud> aligned;
  };


  Descriptor makeDescriptor(
    const open3d::geometry::PointCloud & cloud,
    const Eigen::Vector3d & origin) const
  {
    Descriptor result;

    result.cells.assign(
      descriptor_rings_ *
      descriptor_sectors_,
      0.0);

    result.ring_key.assign(
      descriptor_rings_,
      0.0);

    std::vector<int> counts(
      result.cells.size(),
      0);

    for (const auto & point : cloud.points_) {
      const double x =
        point.x() - origin.x();

      const double y =
        point.y() - origin.y();

      const double radius =
        std::hypot(x, y);

      if (
        !std::isfinite(radius) ||
        radius >= descriptor_radius_)
      {
        continue;
      }

      const int ring =
        std::min(
        descriptor_rings_ - 1,
        static_cast<int>(
          radius /
          descriptor_radius_ *
          descriptor_rings_));

      double angle =
        std::atan2(y, x);

      if (angle < 0.0) {
        angle +=
          2.0 * kPi;
      }

      const int sector =
        std::min(
        descriptor_sectors_ - 1,
        static_cast<int>(
          angle /
          (2.0 * kPi) *
          descriptor_sectors_));

      const size_t index =
        static_cast<size_t>(
        ring *
        descriptor_sectors_ +
        sector);

      result.cells[index] =
        std::max(
        result.cells[index],
        1.0 +
        std::min(
          5.0,
          std::abs(
            point.z() -
            origin.z())));

      counts[index]++;
    }

    for (
      int ring = 0;
      ring < descriptor_rings_;
      ++ring)
    {
      double sum = 0.0;
      int occupied = 0;

      for (
        int sector = 0;
        sector < descriptor_sectors_;
        ++sector)
      {
        const size_t index =
          static_cast<size_t>(
          ring *
          descriptor_sectors_ +
          sector);

        if (counts[index] > 0) {
          sum +=
            result.cells[index];

          occupied++;
        }
      }

      result.ring_key[ring] =
        occupied > 0 ?
        sum / occupied :
        0.0;
    }

    return result;
  }


  Candidate compareDescriptor(
    const Descriptor & query,
    const Descriptor & target) const
  {
    Candidate best;

    double query_norm = 0.0;

    for (double value : query.cells) {
      query_norm +=
        value * value;
    }

    if (query_norm <= 1e-9) {
      return best;
    }

    for (
      int shift = 0;
      shift < descriptor_sectors_;
      ++shift)
    {
      double dot = 0.0;
      double target_norm = 0.0;

      for (
        int ring = 0;
        ring < descriptor_rings_;
        ++ring)
      {
        for (
          int sector = 0;
          sector < descriptor_sectors_;
          ++sector)
        {
          const double q =
            query.cells[
            ring *
            descriptor_sectors_ +
            sector];

          const int shifted_sector =
            (sector + shift) %
            descriptor_sectors_;

          const double t =
            target.cells[
            ring *
            descriptor_sectors_ +
            shifted_sector];

          dot +=
            q * t;

          target_norm +=
            t * t;
        }
      }

      const double denominator =
        std::sqrt(
        query_norm *
        target_norm);

      const double score =
        denominator > 1e-9 ?
        dot / denominator :
        0.0;

      if (score > best.score) {
        best.score =
          score;

        best.sector_shift =
          shift;
      }
    }

    return best;
  }


  bool loadKeyframeDatabase(const std::string & map_name)
  {
    if (keyframe_database_root_.empty() || map_name.empty()) return false;
    if (map_name.find('/') != std::string::npos || map_name.find("..") != std::string::npos) {
      RCLCPP_ERROR(get_logger(), "invalid map name for keyframe database: %s", map_name.c_str());
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (loaded_database_map_ == map_name && !tiles_.empty() && !map_dirty_) return true;
    }

    const std::filesystem::path database_dir =
      std::filesystem::path(keyframe_database_root_) / map_name;
    const std::filesystem::path metadata_path = database_dir / "metadata.yaml";
    try {
      const YAML::Node metadata = YAML::LoadFile(metadata_path.string());
      const std::string metadata_map = metadata["map_name"].as<std::string>("");
      const std::string frame_id = metadata["frame_id"].as<std::string>("");
      const std::string scan_frame_id = metadata["scan_frame_id"].as<std::string>("");
      const bool confirmed = metadata["confirmed_map_aligned"].as<bool>(false);
      if (metadata_map != map_name || frame_id != "map" ||
        scan_frame_id != "base_link" || !confirmed)
      {
        RCLCPP_ERROR(
          get_logger(),
          "reject keyframe database metadata: requested=%s metadata_map=%s "
          "frame=%s scan_frame=%s confirmed=%s",
          map_name.c_str(), metadata_map.c_str(), frame_id.c_str(),
          scan_frame_id.c_str(), confirmed ? "true" : "false");
        return false;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "cannot read keyframe metadata %s: %s",
        metadata_path.c_str(), error.what());
      return false;
    }
    std::ifstream stream(database_dir / "keyframes.csv");
    if (!stream.is_open()) {
      RCLCPP_WARN(
        get_logger(), "keyframe database is unavailable for map=%s path=%s",
        map_name.c_str(), database_dir.c_str());
      return false;
    }

    std::vector<Tile> keyframes;
    std::string line;
    std::getline(stream, line);
    while (std::getline(stream, line)) {
      if (line.empty()) continue;
      std::vector<std::string> fields;
      std::stringstream row(line);
      std::string field;
      while (std::getline(row, field, ',')) fields.push_back(field);
      if (fields.size() != 20) {
        RCLCPP_WARN(get_logger(), "skip malformed keyframe row with %zu fields", fields.size());
        continue;
      }
      try {
        Tile keyframe;
        const std::filesystem::path cloud_path = database_dir / fields[2];
        keyframe.cloud = open3d::io::CreatePointCloudFromFile(cloud_path.string());
        if (!keyframe.cloud || keyframe.cloud->IsEmpty()) {
          RCLCPP_WARN(get_logger(), "skip empty keyframe cloud: %s", cloud_path.c_str());
          continue;
        }
        for (int index = 0; index < 16; ++index) {
          keyframe.pose(index / 4, index % 4) = std::stod(fields[4 + index]);
        }
        if (!finiteTransform(keyframe.pose)) {
          RCLCPP_WARN(get_logger(), "skip keyframe with invalid pose: %s", cloud_path.c_str());
          continue;
        }
        keyframe.center = keyframe.pose.block<3, 1>(0, 3);
        keyframe.descriptor = makeDescriptor(*keyframe.cloud, Eigen::Vector3d::Zero());
        keyframe.is_keyframe = true;
        keyframes.push_back(std::move(keyframe));
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "skip invalid keyframe row: %s", error.what());
      }
    }
    if (keyframes.empty()) {
      RCLCPP_ERROR(get_logger(), "keyframe database contains no usable frames: %s", database_dir.c_str());
      return false;
    }
    const size_t count = keyframes.size();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (map_name_ != map_name) {
        RCLCPP_WARN(
          get_logger(),
          "discard keyframe database loaded during map switch: loaded=%s active=%s",
          map_name.c_str(), map_name_.c_str());
        return false;
      }
      tiles_ = std::move(keyframes);
      descriptor_map_.reset();
      loaded_database_map_ = map_name;
      map_dirty_ = false;
    }
    RCLCPP_INFO(
      get_logger(), "Loaded relocalization keyframe database: map=%s frames=%zu path=%s",
      map_name.c_str(), count, database_dir.c_str());
    return true;
  }


  void mapCallback(
    const sensor_msgs::msg::
    PointCloud2::ConstSharedPtr message)
  {
    if (!keyframe_database_root_.empty()) return;
    uint64_t signature =
      1469598103934665603ULL;

    const auto mix =
      [&signature](uint64_t value)
      {
        signature ^= value;
        signature *=
          1099511628211ULL;
      };

    mix(message->width);
    mix(message->height);
    mix(message->point_step);
    mix(message->row_step);
    mix(message->data.size());

    const size_t stride =
      std::max<size_t>(
      1,
      message->data.size() /
      4096);

    for (
      size_t index = 0;
      index < message->data.size();
      index += stride)
    {
      mix(
        message->data[index]);
    }

    {
      std::lock_guard<std::mutex>
        lock(data_mutex_);

      if (
        !map_dirty_ &&
        signature ==
        map_signature_)
      {
        return;
      }
    }

    auto map_cloud =
      std::make_shared<
      open3d::geometry::PointCloud>();

    open3d_conversions::rosToOpen3d(
      message,
      *map_cloud,
      true);

    *map_cloud =
      map_cloud->
      RemoveNonFinitePoints(
        true,
        true);

    if (map_cloud->IsEmpty()) {
      return;
    }

    auto descriptor_map =
      map_cloud->
      VoxelDownSample(
        voxel_size_);

    std::vector<Tile>
      next_tiles;

    const std::vector<
      std::pair<double, double>>
      offsets =
    {
      {0.0, 0.0},
      {tile_size_ / 2.0, 0.0},
      {0.0, tile_size_ / 2.0},
      {
        tile_size_ / 2.0,
        tile_size_ / 2.0
      }
    };

    for (
      const auto & offset :
      offsets)
    {
      std::map<
        std::tuple<int, int, int>,
        std::shared_ptr<
          open3d::geometry::PointCloud>>
        buckets;

      for (
        const auto & point :
        descriptor_map->points_)
      {
        const int ix =
          static_cast<int>(
          std::floor(
            (
              point.x() -
              offset.first
            ) /
            tile_size_));

        const int iy =
          static_cast<int>(
          std::floor(
            (
              point.y() -
              offset.second
            ) /
            tile_size_));

        const int iz =
          static_cast<int>(
          std::floor(
            point.z() /
            tile_height_));

        auto & bucket =
          buckets[
          {ix, iy, iz}];

        if (!bucket) {
          bucket =
            std::make_shared<
            open3d::geometry::
            PointCloud>();
        }

        bucket->
          points_.
          push_back(point);
      }

      for (
        auto & entry :
        buckets)
      {
        if (
          entry.second->
          points_.size() <
          static_cast<size_t>(
            min_tile_points_))
        {
          continue;
        }

        Tile tile;

        tile.cloud =
          entry.second;

        tile.center =
          tile.cloud->
          GetCenter();

        tile.descriptor =
          makeDescriptor(
          *tile.cloud,
          tile.center);

        next_tiles.
          push_back(
          std::move(tile));
      }
    }

    const size_t tile_count =
      next_tiles.size();

    {
      std::lock_guard<std::mutex>
        lock(data_mutex_);

      tiles_ =
        std::move(
        next_tiles);

      // IMPORTANT:
      // Keep the complete downsampled map.
      // Descriptor search still uses tiles,
      // but ICP verification will crop
      // a larger local map around the
      // candidate tile.
      descriptor_map_ =
        descriptor_map;

      map_signature_ =
        signature;

      map_dirty_ =
        false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Built global descriptor database: "
      "map_points=%zu descriptor_map_points=%zu tiles=%zu",
      map_cloud->points_.size(),
      descriptor_map->points_.size(),
      tile_count);
  }


  void scanCallback(
    const sensor_msgs::msg::
    PointCloud2::ConstSharedPtr message)
  {
    auto scan =
      std::make_shared<
      open3d::geometry::
      PointCloud>();

    open3d_conversions::
    rosToOpen3d(
      message,
      *scan,
      true);

    *scan =
      scan->
      RemoveNonFinitePoints(
        true,
        true);

    std::lock_guard<std::mutex>
      lock(data_mutex_);

    latest_scan_ =
      scan;

    latest_scan_stamp_ =
      rclcpp::Time(
      message->header.stamp);
  }


  void statusCallback(
    const LocalizationStatus::
    ConstSharedPtr message)
  {
    std::lock_guard<std::mutex>
      lock(data_mutex_);

    localization_state_ =
      message->state;

    if (
      !message->map_name.empty() &&
      message->map_name !=
      map_name_)
    {
      map_name_ =
        message->map_name;

      map_dirty_ =
        true;
    }
  }


  std::vector<Candidate>
  findCandidates(
    const open3d::geometry::
    PointCloud & scan,
    const std::vector<Tile> &
    tiles) const
  {
    // Keyframe clouds and the live scan are both expressed in base_link.
    // Their descriptor origin must therefore be the sensor/base origin.  The
    // legacy map-tile fallback contains map-frame points and remains centred
    // on the tile/scan centroid as before.
    const bool keyframe_mode =
      !tiles.empty() && tiles.front().is_keyframe;
    const Descriptor query =
      makeDescriptor(
      scan,
      keyframe_mode ? Eigen::Vector3d::Zero() : scan.GetCenter());

    std::vector<Candidate>
      candidates;

    candidates.reserve(
      tiles.size());

    for (
      size_t i = 0;
      i < tiles.size();
      ++i)
    {
      Candidate candidate =
        compareDescriptor(
        query,
        tiles[i].
        descriptor);

      candidate.tile_index =
        i;

      if (
        candidate.score >=
        min_descriptor_score_)
      {
        candidates.
          push_back(
          candidate);
      }
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [](
        const Candidate & lhs,
        const Candidate & rhs)
      {
        return
          lhs.score >
          rhs.score;
      });

    if (
      candidates.size() >
      static_cast<size_t>(
        max_candidates_))
    {
      candidates.resize(
        static_cast<size_t>(
          max_candidates_));
    }

    return candidates;
  }


  std::shared_ptr<open3d::geometry::PointCloud>
  removeDominantHorizontalPlane(
    const std::shared_ptr<open3d::geometry::PointCloud> & cloud,
    size_t minimum_remaining_points,
    const char * label) const
  {
    if (!cloud || cloud->points_.size() < minimum_remaining_points + 3 ||
      ground_plane_distance_ <= 0.0)
    {
      return cloud;
    }

    const auto segmentation = cloud->SegmentPlane(ground_plane_distance_, 3, 100, 0);
    const Eigen::Vector4d & plane = std::get<0>(segmentation);
    const std::vector<size_t> & inliers = std::get<1>(segmentation);
    const double normal_norm = plane.head<3>().norm();
    const double vertical_normal =
      normal_norm > 1e-9 ? std::abs(plane.z()) / normal_norm : 0.0;

    // Only remove a floor/ceiling-like plane.  Never remove a dominant wall,
    // because vertical structure is what constrains global x/y/yaw.
    if (vertical_normal < 0.85 || inliers.empty()) {
      return cloud;
    }

    auto structural = cloud->SelectByIndex(inliers, true);
    if (!structural || structural->points_.size() < minimum_remaining_points) {
      RCLCPP_WARN(
        get_logger(),
        "%s horizontal-plane removal skipped: remaining=%zu required=%zu",
        label, structural ? structural->points_.size() : 0UL, minimum_remaining_points);
      return cloud;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s horizontal plane removed: input=%zu plane=%zu structural=%zu normal_z=%.3f",
      label, cloud->points_.size(), inliers.size(), structural->points_.size(), vertical_normal);
    return structural;
  }


  Match matchKeyframeCandidate(
    const open3d::geometry::PointCloud & scan,
    const Tile & keyframe,
    const Candidate & candidate) const
  {
    Match match;
    match.descriptor_score = candidate.score;

    auto source = std::make_shared<open3d::geometry::PointCloud>(scan);
    auto target = std::make_shared<open3d::geometry::PointCloud>(*keyframe.cloud);
    source = source->VoxelDownSample(voxel_size_);
    target = target->VoxelDownSample(voxel_size_);
    source = removeDominantHorizontalPlane(
      source, static_cast<size_t>(min_scan_points_), "keyframe source");
    target = removeDominantHorizontalPlane(
      target, static_cast<size_t>(min_target_points_), "keyframe target");

    if (!source || !target ||
      source->points_.size() < static_cast<size_t>(min_scan_points_) ||
      target->points_.size() < static_cast<size_t>(min_target_points_))
    {
      RCLCPP_WARN(
        get_logger(),
        "keyframe candidate rejected before coarse registration: frame=%zu source=%zu target=%zu",
        candidate.tile_index,
        source ? source->points_.size() : 0UL,
        target ? target->points_.size() : 0UL);
      return match;
    }

    const open3d::geometry::KDTreeSearchParamHybrid normal_search(
      std::max(voxel_size_ * 3.0, keyframe_fpfh_radius_ * 0.5), 50);
    source->EstimateNormals(normal_search);
    target->EstimateNormals(normal_search);

    const open3d::geometry::KDTreeSearchParamHybrid feature_search(
      keyframe_fpfh_radius_, 100);
    auto source_feature =
      open3d::pipelines::registration::ComputeFPFHFeature(*source, feature_search);
    auto target_feature =
      open3d::pipelines::registration::ComputeFPFHFeature(*target, feature_search);
    if (!source_feature || !target_feature ||
      source_feature->Num() != source->points_.size() ||
      target_feature->Num() != target->points_.size())
    {
      RCLCPP_WARN(
        get_logger(), "keyframe candidate has invalid FPFH features: frame=%zu",
        candidate.tile_index);
      return match;
    }

    const open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength
      edge_checker(0.9);
    const open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance
      distance_checker(keyframe_ransac_distance_);
    const std::vector<std::reference_wrapper<
      const open3d::pipelines::registration::CorrespondenceChecker>> checkers = {
      std::cref(edge_checker), std::cref(distance_checker)};

    const auto coarse =
      open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
      *source, *target, *source_feature, *target_feature, true,
      keyframe_ransac_distance_,
      open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
      3, checkers,
      open3d::pipelines::registration::RANSACConvergenceCriteria(
        keyframe_ransac_iterations_, 0.999));

    if (coarse.fitness_ <= 0.0 || !std::isfinite(coarse.inlier_rmse_) ||
      !finiteTransform(coarse.transformation_))
    {
      RCLCPP_INFO(
        get_logger(),
        "keyframe coarse registration failed: frame=%zu descriptor=%.3f fitness=%.6f rmse=%.6f",
        candidate.tile_index, candidate.score, coarse.fitness_, coarse.inlier_rmse_);
      return match;
    }

    // The local registration maps current base_link points into the stored
    // keyframe's base_link frame. Compose that with the verified keyframe pose
    // to obtain T_map_current_base.
    const Eigen::Matrix4d global_initial =
      keyframe.pose * coarse.transformation_;
    auto target_map =
      std::make_shared<open3d::geometry::PointCloud>(*target);
    target_map->Transform(keyframe.pose);

    const auto refined =
      open3d::pipelines::registration::RegistrationGeneralizedICP(
      *source, *target_map, keyframe_gicp_distance_, global_initial,
      open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(),
      open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 64));

    match.transform = refined.transformation_;
    match.fitness = refined.fitness_;
    match.rmse = refined.inlier_rmse_;

    const Eigen::Matrix3d local_rotation =
      coarse.transformation_.block<3, 3>(0, 0);
    const double local_tilt_deg =
      std::acos(std::max(-1.0, std::min(1.0, local_rotation(2, 2)))) * 180.0 / kPi;
    const bool finite_ok = finiteTransform(match.transform);
    const bool z_ok = std::abs(match.transform(2, 3)) <= max_abs_z_;
    const bool tilt_ok = local_tilt_deg <= max_keyframe_tilt_delta_deg_;
    const bool fitness_ok = match.fitness >= min_fitness_;
    const bool rmse_ok = match.rmse <= max_inlier_rmse_;
    match.valid = finite_ok && z_ok && tilt_ok && fitness_ok && rmse_ok;

    RCLCPP_INFO(
      get_logger(),
      "keyframe registration: frame=%zu descriptor=%.3f coarse_fit=%.6f "
      "coarse_rmse=%.6f local_tilt=%.2fdeg fine_fit=%.6f fine_rmse=%.6f "
      "xyz=(%.3f %.3f %.3f) valid=%s",
      candidate.tile_index, candidate.score, coarse.fitness_, coarse.inlier_rmse_,
      local_tilt_deg, match.fitness, match.rmse,
      match.transform(0, 3), match.transform(1, 3), match.transform(2, 3),
      match.valid ? "true" : "false");

    match.aligned = std::make_shared<open3d::geometry::PointCloud>(*source);
    match.aligned->Transform(match.transform);
    return match;
  }


  Match matchCandidate(
    const open3d::geometry::
    PointCloud & scan,
    const open3d::geometry::
    PointCloud & map,
    const Tile & tile,
    const Candidate &
    candidate) const
  {
    Match match;

    match.descriptor_score =
      candidate.score;

    auto source =
      std::make_shared<
      open3d::geometry::
      PointCloud>(scan);

    source =
      source->
      VoxelDownSample(
        voxel_size_);

    const Eigen::Vector3d source_pose_center = source->GetCenter();
    source = removeDominantHorizontalPlane(
      source, static_cast<size_t>(min_scan_points_), "source");

    /*
     * IMPORTANT CHANGE
     *
     * Previously:
     *
     *   target = tile.cloud
     *
     * That means ICP only saw one
     * 24m x 24m x 3m descriptor tile.
     *
     * Now:
     *
     *   descriptor tile selects an
     *   approximate location only.
     *
     *   ICP gets a much larger local
     *   crop from the complete map.
     */
    const Eigen::Vector3d
      min_bound(
      tile.center.x() -
      target_half_xy_,

      tile.center.y() -
      target_half_xy_,

      tile.center.z() -
      target_half_z_);

    const Eigen::Vector3d
      max_bound(
      tile.center.x() +
      target_half_xy_,

      tile.center.y() +
      target_half_xy_,

      tile.center.z() +
      target_half_z_);

    const open3d::geometry::
      AxisAlignedBoundingBox
      bbox(
      min_bound,
      max_bound);

    auto target =
      map.Crop(
      bbox);

    if (!target) {
      RCLCPP_WARN(
        get_logger(),
        "candidate rejected: "
        "map crop returned null "
        "tile=%zu",
        candidate.tile_index);

      return match;
    }

    target =
      target->
      VoxelDownSample(
        voxel_size_);

    target = removeDominantHorizontalPlane(
      target, static_cast<size_t>(min_target_points_), "target");

    const Eigen::Vector3d
      source_center =
      source->
      GetCenter();

    const Eigen::Vector3d
      target_center =
      target->
      GetCenter();

    RCLCPP_INFO(
      get_logger(),
      "candidate detail: "
      "tile=%zu descriptor=%.3f shift=%d "
      "source_points=%zu target_points=%zu "
      "source_center=(%.3f %.3f %.3f) "
      "tile_center=(%.3f %.3f %.3f) "
      "target_center=(%.3f %.3f %.3f) "
      "crop_xy=%.1f crop_z=%.1f",
      candidate.tile_index,
      candidate.score,
      candidate.sector_shift,
      source->points_.size(),
      target->points_.size(),

      source_center.x(),
      source_center.y(),
      source_center.z(),

      tile.center.x(),
      tile.center.y(),
      tile.center.z(),

      target_center.x(),
      target_center.y(),
      target_center.z(),

      target_half_xy_ * 2.0,
      target_half_z_ * 2.0);

    if (
      source->
      points_.size() <
      static_cast<size_t>(
        min_scan_points_) ||
      target->
      points_.size() <
      static_cast<size_t>(
        min_target_points_))
    {
      RCLCPP_WARN(
        get_logger(),
        "candidate rejected before ICP: "
        "tile=%zu "
        "source=%zu(min=%d) "
        "target=%zu(min=%d)",
        candidate.tile_index,

        source->
        points_.size(),

        min_scan_points_,

        target->
        points_.size(),

        min_target_points_);

      return match;
    }

    target->
      EstimateNormals(
      open3d::geometry::
      KDTreeSearchParamHybrid(
        voxel_size_ *
        3.0,
        30));

    const double
      descriptor_yaw =
      normalizeAngle(
      static_cast<double>(
        candidate.
        sector_shift) *
      2.0 *
      kPi /
      descriptor_sectors_);

    for (
      const double yaw :
      {
        descriptor_yaw,
        -descriptor_yaw
      })
    {
      Eigen::Matrix4d
        transform =
        Eigen::Matrix4d::
        Identity();

      const Eigen::Matrix3d
        rotation =
        Eigen::AngleAxisd(
        yaw,
        Eigen::Vector3d::
        UnitZ()).
        toRotationMatrix();

      transform.
        block<3, 3>(
        0,
        0) =
        rotation;

      /*
       * Keep the original coarse
       * translation initialization.
       *
       * We are changing only the
       * ICP target for this test.
       */
      transform.
        block<3, 1>(
        0,
        3) =
        tile.center -
        rotation *
        source_pose_center;

      RCLCPP_INFO(
        get_logger(),
        "candidate initial: "
        "tile=%zu "
        "descriptor=%.3f "
        "yaw=%.2f deg "
        "xyz=(%.3f %.3f %.3f)",
        candidate.tile_index,
        candidate.score,
        yaw *
        180.0 /
        kPi,
        transform(0, 3),
        transform(1, 3),
        transform(2, 3));

      const std::vector<double>
        scales =
      {
        4.0,
        2.0,
        1.0
      };

      for (
        double scale :
        scales)
      {
        const double
          resolution =
          voxel_size_ *
          scale;

        const double
          max_correspondence =
          resolution *
          1.5;

        auto src_scale =
          source->
          VoxelDownSample(
            resolution);

        auto dst_scale =
          target->
          VoxelDownSample(
            resolution);

        if (
          src_scale->
          IsEmpty() ||
          dst_scale->
          IsEmpty())
        {
          RCLCPP_WARN(
            get_logger(),
            "ICP skipped: "
            "tile=%zu "
            "scale=%.1f "
            "src_empty=%s "
            "dst_empty=%s",
            candidate.tile_index,
            scale,

            src_scale->
            IsEmpty() ?
            "true" :
            "false",

            dst_scale->
            IsEmpty() ?
            "true" :
            "false");

          continue;
        }

        dst_scale->
          EstimateNormals(
          open3d::geometry::
          KDTreeSearchParamHybrid(
            resolution *
            3.0,
            30));

        // Point-to-plane is unstable on the very sparse coarse clouds used
        // by global relocalization.  A degenerate plane fit previously moved
        // candidates by tens or even thousands of metres while reporting
        // fitness=0, and that corrupt transform was then fed to every finer
        // ICP level.  Use point-to-point for coarse capture and reserve
        // point-to-plane for the final refinement.
        const auto result =
          scale > 1.0 ?
          open3d::pipelines::registration::RegistrationICP(
            *src_scale,
            *dst_scale,
            max_correspondence,
            transform,
            open3d::pipelines::registration::
            TransformationEstimationPointToPoint(false),
            open3d::pipelines::registration::ICPConvergenceCriteria(
              1e-6, 1e-6, 40)) :
          open3d::pipelines::registration::RegistrationICP(
            *src_scale,
            *dst_scale,
            max_correspondence,
            transform,
            open3d::pipelines::registration::
            TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(
              1e-6, 1e-6, 40));

        // Keep the complete rigid transform. Real map poses can contain
        // roll/pitch, so projecting every result to yaw-only creates a
        // systematic error. Horizontal-plane removal and the geometric gates
        // below protect the fallback matcher from floor-dominated solutions.
        const Eigen::Matrix4d constrained_result = result.transformation_;
        const double step_translation =
          (constrained_result.block<3, 1>(0, 3) -
          transform.block<3, 1>(0, 3)).norm();
        const double max_step_translation =
          std::max(tile_size_, descriptor_radius_);
        const bool result_usable =
          result.fitness_ > 0.0 &&
          std::isfinite(result.inlier_rmse_) &&
          finiteTransform(constrained_result) &&
          step_translation <= max_step_translation;

        if (result_usable) {
          transform = constrained_result;
        } else {
          RCLCPP_WARN(
            get_logger(),
            "discard degenerate ICP step: tile=%zu scale=%.1f "
            "fitness=%.6f rmse=%.6f step_translation=%.3f(max=%.3f)",
            candidate.tile_index,
            scale,
            result.fitness_,
            result.inlier_rmse_,
            step_translation,
            max_step_translation);
        }

        RCLCPP_INFO(
          get_logger(),
          "ICP result: "
          "tile=%zu "
          "scale=%.1f "
          "resolution=%.3f "
          "max_corr=%.3f "
          "src=%zu "
          "dst=%zu "
          "fitness=%.6f "
          "rmse=%.6f "
          "xyz=(%.3f %.3f %.3f)",
          candidate.tile_index,
          scale,
          resolution,
          max_correspondence,

          src_scale->
          points_.size(),

          dst_scale->
          points_.size(),

          result.
          fitness_,

          result.
          inlier_rmse_,

          transform(0, 3),
          transform(1, 3),
          transform(2, 3));
      }

      const auto evaluation =
        open3d::pipelines::
        registration::
        EvaluateRegistration(
        *source,
        *target,
        voxel_size_ *
        1.5,
        transform);

      RCLCPP_INFO(
        get_logger(),
        "candidate evaluation: "
        "tile=%zu "
        "descriptor=%.3f "
        "fitness=%.6f "
        "rmse=%.6f "
        "xyz=(%.3f %.3f %.3f)",
        candidate.tile_index,
        candidate.score,

        evaluation.
        fitness_,

        evaluation.
        inlier_rmse_,

        transform(0, 3),
        transform(1, 3),
        transform(2, 3));

      if (
        evaluation.
        fitness_ >
        match.fitness ||
        (
          evaluation.
          fitness_ ==
          match.fitness &&
          evaluation.
          inlier_rmse_ <
          match.rmse
        ))
      {
        match.second_transform = match.transform;
        match.second_fitness = match.fitness;
        match.second_rmse = match.rmse;
        match.transform =
          transform;

        match.fitness =
          evaluation.
          fitness_;

        match.rmse =
          evaluation.
          inlier_rmse_;
      } else if (
        evaluation.fitness_ > match.second_fitness ||
        (evaluation.fitness_ == match.second_fitness &&
        evaluation.inlier_rmse_ < match.second_rmse))
      {
        match.second_transform = transform;
        match.second_fitness = evaluation.fitness_;
        match.second_rmse = evaluation.inlier_rmse_;
      }
    }

    const Eigen::Matrix4d &
      transform =
      match.transform;

    const bool finite_ok =
      finiteTransform(
      transform);

    const bool z_ok =
      std::abs(
      transform(2, 3)) <=
      max_abs_z_;

    const bool fitness_ok =
      match.fitness >=
      min_fitness_;

    const bool rmse_ok =
      match.rmse <=
      max_inlier_rmse_;

    match.valid =
      finite_ok &&
      z_ok &&
      fitness_ok &&
      rmse_ok;

    RCLCPP_INFO(
      get_logger(),
      "candidate verification: "
      "tile=%zu "
      "valid=%s "
      "finite=%s "
      "z_ok=%s "
      "fitness_ok=%s "
      "rmse_ok=%s "
      "fitness=%.6f(min=%.3f) "
      "rmse=%.6f(max=%.3f) "
      "z=%.3f(max_abs=%.3f)",
      candidate.tile_index,

      match.valid ?
      "true" :
      "false",

      finite_ok ?
      "true" :
      "false",

      z_ok ?
      "true" :
      "false",

      fitness_ok ?
      "true" :
      "false",

      rmse_ok ?
      "true" :
      "false",

      match.fitness,
      min_fitness_,
      match.rmse,
      max_inlier_rmse_,
      transform(2, 3),
      max_abs_z_);

    match.aligned =
      std::make_shared<
      open3d::geometry::
      PointCloud>(
      *source);

    match.aligned->
      Transform(
      transform);

    return match;
  }


  void handleTrigger(
    const std::shared_ptr<
      GlobalRelocalize::
      Request> request,

    std::shared_ptr<
      GlobalRelocalize::
      Response> response)
  {
    if (!enabled_) {
      response->message =
        "global relocalization is disabled by configuration";

      return;
    }

    if (
      running_.
      exchange(true))
    {
      response->message =
        "global relocalization is already running";

      return;
    }

    struct RunningGuard
    {
      std::atomic_bool &
        flag;

      ~RunningGuard()
      {
        flag.store(false);
      }
    } guard{running_};

    if (!keyframe_database_root_.empty()) {
      std::string active_map;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        active_map = map_name_;
      }
      if (!loadKeyframeDatabase(active_map)) {
        response->message =
          "keyframe database unavailable for active map '" + active_map + "'";
        return;
      }
    }

    std::shared_ptr<
      open3d::geometry::
      PointCloud> scan;

    std::shared_ptr<
      open3d::geometry::
      PointCloud> descriptor_map;

    std::vector<Tile>
      tiles;

    rclcpp::Time scan_stamp(
      0,
      0,
      RCL_ROS_TIME);

    uint8_t
      localization_state =
      LocalizationStatus::
      UNINITIALIZED;

    bool map_dirty =
      true;

    {
      std::lock_guard<std::mutex>
        lock(data_mutex_);

      scan =
        latest_scan_;

      descriptor_map =
        descriptor_map_;

      tiles =
        tiles_;

      scan_stamp =
        latest_scan_stamp_;

      localization_state =
        localization_state_;

      map_dirty =
        map_dirty_;
    }

    const bool keyframe_mode =
      !tiles.empty() && tiles.front().is_keyframe;

    RCLCPP_INFO(
      get_logger(),
      "trigger received: "
      "apply=%s "
      "allow_while_tracking=%s "
      "state=%u "
      "tiles=%zu "
      "scan_points=%zu "
      "map_points=%zu "
      "database_mode=%s "
      "map_dirty=%s",

      request->apply ?
      "true" :
      "false",

      request->
      allow_while_tracking ?
      "true" :
      "false",

      static_cast<
        unsigned int>(
        localization_state),

      tiles.size(),

      scan ?
      scan->
      points_.size() :
      0UL,

      descriptor_map ?
      descriptor_map->
      points_.size() :
      0UL,

      keyframe_mode ? "keyframes" : "map_tiles",

      map_dirty ?
      "true" :
      "false");

    if (
      !scan ||
      scan->IsEmpty())
    {
      response->message =
        "no scan received from " +
        scan_topic_;

      return;
    }

    if (
      !keyframe_mode &&
      (!descriptor_map ||
      descriptor_map->
      IsEmpty()))
    {
      response->message =
        "descriptor map is empty; wait for " +
        map_topic_;

      return;
    }

    if (tiles.empty()) {
      response->message =
        "global descriptor database is empty; wait for " +
        map_topic_;

      return;
    }

    if (
      map_dirty ||
      localization_state ==
      LocalizationStatus::MAP_SWITCHING)
    {
      response->message =
        "map is switching or the global descriptor database is stale";

      return;
    }

    const double scan_age =
      (
        now() -
        scan_stamp
      ).
      seconds();

    RCLCPP_INFO(
      get_logger(),
      "trigger scan age: "
      "%.3f s "
      "(max=%.3f)",
      scan_age,
      max_scan_age_s_);

    if (
      scan_age < -0.1 ||
      scan_age >
      max_scan_age_s_)
    {
      response->message =
        "latest scan is stale";

      return;
    }

    if (
      request->apply &&
      !request->
      allow_while_tracking &&
      localization_state ==
      LocalizationStatus::TRACKING)
    {
      response->message =
        "refusing to overwrite a TRACKING pose; "
        "set allow_while_tracking only for supervised recovery";

      return;
    }

    auto candidates =
      findCandidates(
      *scan,
      tiles);

    RCLCPP_INFO(
      get_logger(),
      "descriptor search complete: "
      "candidate_count=%zu "
      "threshold=%.3f",
      candidates.size(),
      min_descriptor_score_);

    if (
      candidates.empty())
    {
      response->message =
        "no descriptor candidate passed the threshold";

      return;
    }

    Match best;

    std::vector<Match> match_hypotheses;

    size_t candidate_index =
      0;

    for (
      const auto &
      candidate :
      candidates)
    {
      RCLCPP_INFO(
        get_logger(),
        "testing candidate "
        "%zu/%zu: "
        "tile=%zu "
        "descriptor=%.6f "
        "sector_shift=%d",
        candidate_index + 1,
        candidates.size(),
        candidate.tile_index,
        candidate.score,
        candidate.sector_shift);

      Match current;
      if (keyframe_mode) {
        current = matchKeyframeCandidate(
          *scan, tiles[candidate.tile_index], candidate);
      } else {
        current = matchCandidate(
          *scan, *descriptor_map, tiles[candidate.tile_index], candidate);
      }

      RCLCPP_INFO(
        get_logger(),
        "candidate result: "
        "tile=%zu "
        "valid=%s "
        "descriptor=%.6f "
        "fitness=%.6f "
        "rmse=%.6f "
        "xyz=(%.3f %.3f %.3f)",
        candidate.tile_index,

        current.valid ?
        "true" :
        "false",

        current.
        descriptor_score,

        current.
        fitness,

        current.
        rmse,

        current.
        transform(0, 3),

        current.
        transform(1, 3),

        current.
        transform(2, 3));

      match_hypotheses.push_back(current);

      if (current.second_fitness > 0.0 && finiteTransform(current.second_transform)) {
        Match alternate;
        alternate.descriptor_score = current.descriptor_score;
        alternate.transform = current.second_transform;
        alternate.fitness = current.second_fitness;
        alternate.rmse = current.second_rmse;
        alternate.valid =
          std::abs(alternate.transform(2, 3)) <= max_abs_z_ &&
          alternate.fitness >= min_fitness_ &&
          alternate.rmse <= max_inlier_rmse_;
        match_hypotheses.push_back(std::move(alternate));
      }

      if (
        current.fitness >
        best.fitness ||
        (
          current.fitness ==
          best.fitness &&
          current.rmse <
          best.rmse
        ))
      {
        best =
          std::move(
          current);
      }

      candidate_index++;
    }

    std::sort(
      match_hypotheses.begin(), match_hypotheses.end(),
      [](const Match & lhs, const Match & rhs) {
        return lhs.fitness > rhs.fitness ||
          (lhs.fitness == rhs.fitness && lhs.rmse < rhs.rmse);
      });

    const auto best_valid = std::find_if(
      match_hypotheses.begin(), match_hypotheses.end(),
      [](const Match & match) {return match.valid;});
    if (best_valid != match_hypotheses.end()) {
      best = *best_valid;
    } else if (!match_hypotheses.empty()) {
      best = match_hypotheses.front();
    }

    // Rebuild the aligned cloud for the selected hypothesis.  This also
    // supports a runner-up yaw hypothesis becoming the final winner.
    best.aligned = std::make_shared<open3d::geometry::PointCloud>(*scan);
    best.aligned = best.aligned->VoxelDownSample(voxel_size_);
    best.aligned->Transform(best.transform);

    response->
      descriptor_score =
      best.
      descriptor_score;

    response->
      fitness =
      best.
      fitness;

    response->
      inlier_rmse =
      best.
      rmse;

    if (!best.valid) {
      RCLCPP_WARN(
        get_logger(),
        "global relocalization rejected: "
        "descriptor=%.6f "
        "fitness=%.6f "
        "rmse=%.6f "
        "xyz=(%.3f %.3f %.3f)",
        best.
        descriptor_score,
        best.
        fitness,
        best.
        rmse,
        best.
        transform(0, 3),
        best.
        transform(1, 3),
        best.
        transform(2, 3));

      response->message =
        "candidate rejected by geometric verification: "
        "fitness=" +
        std::to_string(
        best.fitness) +
        " rmse=" +
        std::to_string(
        best.rmse) +
        " z=" +
        std::to_string(
        best.transform(
          2,
          3));

      return;
    }

    const Match * second_distinct = nullptr;
    for (const auto & hypothesis : match_hypotheses) {
      if (!hypothesis.valid || &hypothesis == &(*best_valid)) {
        continue;
      }
      const double translation_separation =
        (hypothesis.transform.block<3, 1>(0, 3) -
        best.transform.block<3, 1>(0, 3)).norm();
      const double yaw_separation_deg =
        std::abs(normalizeAngle(
          transformYaw(hypothesis.transform) - transformYaw(best.transform))) *
        180.0 / kPi;
      if (translation_separation >= ambiguity_translation_m_ ||
        yaw_separation_deg >= ambiguity_yaw_deg_)
      {
        second_distinct = &hypothesis;
        break;
      }
    }

    if (second_distinct != nullptr) {
      const double fitness_margin = best.fitness - second_distinct->fitness;
      RCLCPP_INFO(
        get_logger(),
        "global candidate margin: best=%.6f second=%.6f margin=%.6f(min=%.3f)",
        best.fitness, second_distinct->fitness, fitness_margin, min_fitness_margin_);
      if (fitness_margin < min_fitness_margin_) {
        response->message =
          "ambiguous global relocalization candidates: fitness margin=" +
          std::to_string(fitness_margin) + " required=" +
          std::to_string(min_fitness_margin_);
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }
    }

    const Eigen::Matrix3d
      rotation =
      best.
      transform.
      block<3, 3>(
        0,
        0);

    Eigen::Quaterniond
      quaternion(
      rotation);

    quaternion.
      normalize();

    response->
      pose.
      position.x =
      best.
      transform(0, 3);

    response->
      pose.
      position.y =
      best.
      transform(1, 3);

    response->
      pose.
      position.z =
      best.
      transform(2, 3);

    response->
      pose.
      orientation.x =
      quaternion.x();

    response->
      pose.
      orientation.y =
      quaternion.y();

    response->
      pose.
      orientation.z =
      quaternion.z();

    response->
      pose.
      orientation.w =
      quaternion.w();

    geometry_msgs::msg::
      PoseStamped
      candidate_pose;

    candidate_pose.
      header.
      frame_id =
      "map";

    candidate_pose.
      header.
      stamp =
      now();

    candidate_pose.
      pose =
      response->pose;

    candidate_pose_pub_->
      publish(
      candidate_pose);

    if (best.aligned) {
      sensor_msgs::msg::
        PointCloud2
        aligned_message;

      open3d_conversions::
      open3dToRos(
        *best.aligned,
        aligned_message,
        "map");

      aligned_message.
        header.
        stamp =
        candidate_pose.
        header.
        stamp;

      aligned_cloud_pub_->
        publish(
        aligned_message);
    }

    if (request->apply) {
      geometry_msgs::msg::
        PoseWithCovarianceStamped
        initialpose;

      initialpose.header =
        candidate_pose.header;

      initialpose.pose.pose =
        response->pose;

      initialpose.
        pose.
        covariance[0] =
        0.25;

      initialpose.
        pose.
        covariance[7] =
        0.25;

      initialpose.
        pose.
        covariance[14] =
        0.25;

      initialpose.
        pose.
        covariance[35] =
        0.0685;

      initialpose_pub_->
        publish(
        initialpose);

      RCLCPP_INFO(
        get_logger(),
        "verified global relocalization "
        "published to %s",
        initialpose_topic_.
        c_str());
    }

    response->
      success =
      true;

    response->
      message =
      request->apply ?
      "verified candidate published to /initialpose; wait for localization_status" :
      "verified candidate computed; pose was not applied";
  }


  bool enabled_{false};

  std::string map_topic_;
  std::string scan_topic_;
  std::string status_topic_;
  std::string initialpose_topic_;
  std::string keyframe_database_root_;
  std::string loaded_database_map_;

  double keyframe_fpfh_radius_{2.0};
  double keyframe_ransac_distance_{1.0};
  double keyframe_gicp_distance_{0.6};
  int keyframe_ransac_iterations_{40000};
  double max_keyframe_tilt_delta_deg_{20.0};

  int descriptor_rings_{20};
  int descriptor_sectors_{60};

  double descriptor_radius_{20.0};
  double tile_size_{24.0};
  double tile_height_{3.0};

  int min_tile_points_{500};
  int max_candidates_{5};

  double voxel_size_{0.4};

  int min_scan_points_{300};
  int min_target_points_{800};

  double min_descriptor_score_{0.15};
  double min_fitness_{0.70};
  double max_inlier_rmse_{0.4};
  double max_abs_z_{5.0};
  double max_scan_age_s_{2.0};
  double target_half_xy_{12.0};
  double target_half_z_{2.0};
  double ground_plane_distance_{0.15};
  double min_fitness_margin_{0.05};
  double ambiguity_translation_m_{1.0};
  double ambiguity_yaw_deg_{10.0};

  std::mutex data_mutex_;

  std::shared_ptr<
    open3d::geometry::
    PointCloud>
    latest_scan_;

  // Complete downsampled map used
  // to crop a larger ICP target.
  std::shared_ptr<
    open3d::geometry::
    PointCloud>
    descriptor_map_;

  std::vector<Tile>
    tiles_;

  rclcpp::Time
    latest_scan_stamp_{
    0,
    0,
    RCL_ROS_TIME};

  uint64_t
    map_signature_{0};

  bool
    map_dirty_{true};

  std::string
    map_name_;

  uint8_t
    localization_state_{
    LocalizationStatus::
    UNINITIALIZED};

  std::atomic_bool
    running_{false};

  rclcpp::Subscription<
    sensor_msgs::msg::
    PointCloud2>::
    SharedPtr
    map_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::
    PointCloud2>::
    SharedPtr
    scan_sub_;

  rclcpp::Subscription<
    LocalizationStatus>::
    SharedPtr
    status_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::
    PoseWithCovarianceStamped>::
    SharedPtr
    initialpose_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::
    PoseStamped>::
    SharedPtr
    candidate_pose_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::
    PointCloud2>::
    SharedPtr
    aligned_cloud_pub_;

  rclcpp::Service<
    GlobalRelocalize>::
    SharedPtr
    service_;
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  rclcpp::spin(
    std::make_shared<
      GlobalRelocalizationNode>());

  rclcpp::shutdown();

  return 0;
}
