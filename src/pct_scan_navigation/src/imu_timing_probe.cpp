#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
using SteadyClock = std::chrono::steady_clock;

double secondsBetween(const SteadyClock::time_point &newer,
                      const SteadyClock::time_point &older)
{
  return std::chrono::duration<double>(newer - older).count();
}

int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

class ImuTimingProbe : public rclcpp::Node
{
public:
  ImuTimingProbe() : Node("imu_timing_probe")
  {
    const std::string topic =
        declare_parameter<std::string>("imu_topic", "/livox/imu");
    report_period_s_ = std::max(
        0.2, declare_parameter<double>("report_period_s", 1.0));
    gap_warn_s_ = std::max(
        0.001, declare_parameter<double>("header_gap_warn_s", 0.02));
    arrival_gap_warn_s_ = std::max(
        0.001, declare_parameter<double>("arrival_gap_warn_s", 0.02));
    accel_axis_warn_ = std::max(
        0.0, declare_parameter<double>("accel_axis_warn", 3.8));
    const auto configured_queue_depth =
        declare_parameter<int64_t>("queue_depth", 1000);
    const int queue_depth = static_cast<int>(
        std::clamp<int64_t>(configured_queue_depth, 10, 100000));

    // Match FAST-LIO's reliable/volatile subscription while using a deeper
    // queue so this probe remains an independent reference under short stalls.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth))
                   .reliable()
                   .durability_volatile();
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        topic, qos,
        std::bind(&ImuTimingProbe::imuCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::duration<double>(report_period_s_),
        std::bind(&ImuTimingProbe::report, this));
    window_start_ = SteadyClock::now();

    RCLCPP_INFO(
        get_logger(),
        "IMU timing probe started: topic=%s queue_depth=%d gap_warn=%.3f s",
        topic.c_str(), queue_depth, gap_warn_s_);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto arrival = SteadyClock::now();
    const int64_t stamp_ns = stampNanoseconds(msg->header.stamp);

    ++window_count_;
    ++total_count_;
    if (have_previous_)
    {
      const double header_dt = static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
      const double arrival_dt = secondsBetween(arrival, last_arrival_);
      last_header_dt_s_ = header_dt;
      max_header_dt_s_ = std::max(max_header_dt_s_, header_dt);
      max_arrival_dt_s_ = std::max(max_arrival_dt_s_, arrival_dt);
      if (header_dt <= 0.0)
        ++backward_count_;
      else if (header_dt > gap_warn_s_)
      {
        ++header_gap_count_;
        if (header_dt >= worst_header_dt_s_)
        {
          worst_header_dt_s_ = header_dt;
          worst_header_gap_stamp_ns_ = stamp_ns;
        }
      }
      if (arrival_dt > arrival_gap_warn_s_)
        ++arrival_gap_count_;
    }

    const auto &accel = msg->linear_acceleration;
    if (std::max({std::abs(accel.x), std::abs(accel.y), std::abs(accel.z)}) >=
        accel_axis_warn_)
      ++clip_count_;

    last_stamp_ns_ = stamp_ns;
    last_arrival_ = arrival;
    have_previous_ = true;
  }

  void report()
  {
    const auto now = SteadyClock::now();
    const double elapsed = std::max(1e-9, secondsBetween(now, window_start_));
    const double rate = static_cast<double>(window_count_) / elapsed;
    const bool warning = header_gap_count_ > 0 || backward_count_ > 0 ||
                         arrival_gap_count_ > 0;
    if (warning)
    {
      RCLCPP_WARN(
          get_logger(),
          "[IMU_INPUT_DIAG] period=%.3fs count=%lu total=%lu rate=%.2fHz "
          "header_dt_last=%.3fms header_dt_max=%.3fms gaps=%lu backward=%lu "
          "worst_gap_stamp_ns=%ld arrival_dt_max=%.3fms arrival_gaps=%lu clip=%lu",
          elapsed, static_cast<unsigned long>(window_count_),
          static_cast<unsigned long>(total_count_), rate,
          last_header_dt_s_ * 1000.0, max_header_dt_s_ * 1000.0,
          static_cast<unsigned long>(header_gap_count_),
          static_cast<unsigned long>(backward_count_),
          static_cast<long>(worst_header_gap_stamp_ns_),
          max_arrival_dt_s_ * 1000.0,
          static_cast<unsigned long>(arrival_gap_count_),
          static_cast<unsigned long>(clip_count_));
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "[IMU_INPUT_DIAG] period=%.3fs count=%lu total=%lu rate=%.2fHz "
          "header_dt_last=%.3fms header_dt_max=%.3fms gaps=0 backward=0 "
          "arrival_dt_max=%.3fms arrival_gaps=0 clip=%lu",
          elapsed, static_cast<unsigned long>(window_count_),
          static_cast<unsigned long>(total_count_), rate,
          last_header_dt_s_ * 1000.0, max_header_dt_s_ * 1000.0,
          max_arrival_dt_s_ * 1000.0,
          static_cast<unsigned long>(clip_count_));
    }

    window_start_ = now;
    window_count_ = 0;
    header_gap_count_ = 0;
    backward_count_ = 0;
    arrival_gap_count_ = 0;
    clip_count_ = 0;
    max_header_dt_s_ = 0.0;
    max_arrival_dt_s_ = 0.0;
    worst_header_dt_s_ = 0.0;
    worst_header_gap_stamp_ns_ = 0;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  double report_period_s_{1.0};
  double gap_warn_s_{0.02};
  double arrival_gap_warn_s_{0.02};
  double accel_axis_warn_{3.8};
  bool have_previous_{false};
  int64_t last_stamp_ns_{0};
  SteadyClock::time_point last_arrival_{};
  SteadyClock::time_point window_start_{};
  uint64_t total_count_{0};
  uint64_t window_count_{0};
  uint64_t header_gap_count_{0};
  uint64_t backward_count_{0};
  uint64_t arrival_gap_count_{0};
  uint64_t clip_count_{0};
  double last_header_dt_s_{0.0};
  double max_header_dt_s_{0.0};
  double max_arrival_dt_s_{0.0};
  double worst_header_dt_s_{0.0};
  int64_t worst_header_gap_stamp_ns_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTimingProbe>());
  rclcpp::shutdown();
  return 0;
}
