// Copyright 2022 Andrew Symington
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
#include <cstdint>
#include <memory>
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
#include "libsurvive_ros2/msg/pose_confidence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
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
  void update_confidence_state(
    const SurviveSimpleObject * object, const rclcpp::Time & stamp);
  void publish_device_confidence(
    const std::string & serial, float confidence, const rclcpp::Time & stamp);
  void publish_device_occlusion(
    const std::string & serial, bool occluded, const rclcpp::Time & stamp);

private:
  void work();
  SurviveSimpleContext *actx_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<libsurvive_ros2::msg::OcclusionStatus>::SharedPtr occlusion_publisher_;
  rclcpp::Publisher<libsurvive_ros2::msg::PoseConfidence>::SharedPtr confidence_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr cfg_publisher_;
  std::thread worker_thread_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_base_station_update_{0, 0, RCL_STEADY_TIME};
  std::unordered_map<std::string, rclcpp::Time> last_battery_publish_by_device_;
  std::string parent_frame_;
  std::string occlusion_topic_base_;
  std::unordered_map<std::string, bool> occlusion_by_device_;
  std::unordered_map<std::string, int> occlusion_enter_count_by_device_;
  std::unordered_map<std::string, int> occlusion_exit_count_by_device_;
  double lighthouse_rate_;
};

}  // namespace libsurvive_ros2
