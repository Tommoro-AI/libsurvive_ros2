// Copyright 2022 Andrew Symington
// Copyright 2026 Sunghyun Kwon
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#define SURVIVE_ENABLE_FULL_API

// C system
#include <os_generic.h>

// C++ system
#include <array>
#include <cstdint>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

// Other
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "libsurvive/survive.h"
#include "libsurvive/survive_api.h"
#include "libsurvive_ros2/msg/occlusion_status.hpp"
#include "libsurvive_ros2/tracking_calibration.hpp"
#include "libsurvive_ros2/optical_diagnostic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace libsurvive_ros2
{

class Component : public rclcpp::Node
{
public:
  explicit Component(const rclcpp::NodeOptions & options);
  virtual ~Component();
  void publish_imu(const sensor_msgs::msg::Imu & msg);
  void publish_velocity(const geometry_msgs::msg::TwistStamped & msg);
  void publish_battery(const sensor_msgs::msg::BatteryState & msg);
  void publish_device_battery(
    const SurviveSimpleObject *object, const rclcpp::Time & stamp);
  void update_occlusion_state(
    const SurviveSimpleObject *object, const rclcpp::Time & stamp);
  void publish_device_occlusion(
    const std::string & serial, bool occluded, const rclcpp::Time & stamp);

private:
  void work();
  void log_optical_diagnostics();
  OpticalDiagnosticMonitor::Clock::time_point last_optical_log_{};
  std::unordered_map<const SurviveSimpleObject *, OpticalDiagnosticMonitor> optical_monitors_;
  bool tracking_ready();
  bool start_driver(bool recalibrate = false);
  void set_ready(bool ready);
  TrackingCalibration calibration_;
  std::atomic_bool tracking_ready_{false};
  std::atomic_bool calibration_valid_{false};
  std::atomic_bool recalibrate_requested_{false};
  std::mutex readiness_mutex_;
  bool tracking_started_{false};
  struct StationObservation
  {
    std::uint64_t samples{0};
    TrackingCalibration::Clock::time_point last_seen{};
    std::array<FLT, 3> up_direction{};
  };
  std::array<StationObservation, NUM_GEN2_LIGHTHOUSES> station_observations_{};
  std::string driver_args_;
  SurviveSimpleContext *actx_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<libsurvive_ros2::msg::OcclusionStatus>::SharedPtr occlusion_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr cfg_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recalibrate_service_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  std::thread worker_thread_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_base_station_update_{0, 0, RCL_STEADY_TIME};
  std::unordered_map<std::string, rclcpp::Time> last_battery_publish_by_device_;
  std::string tracking_frame_;
  std::string occlusion_topic_base_;
  std::unordered_map<std::string, bool> occlusion_by_device_;
  std::unordered_map<std::string, int> occlusion_enter_count_by_device_;
  std::unordered_map<std::string, int> occlusion_exit_count_by_device_;
  double lighthouse_rate_;
};

}  // namespace libsurvive_ros2
