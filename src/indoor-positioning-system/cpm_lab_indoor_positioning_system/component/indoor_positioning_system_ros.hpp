// Copyright 2025 Cyber-Physical Mobility Group
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


#ifndef CPM_LAB_INDOOR_POSITIONING_SYSTEM__COMPONENT__INDOOR_POSITIONING_SYSTEM_ROS_HPP_
#define CPM_LAB_INDOOR_POSITIONING_SYSTEM__COMPONENT__INDOOR_POSITIONING_SYSTEM_ROS_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/time.hpp>

#include <cpm_lab_lab_msgs/msg/vehicle_command.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_observation.hpp>

#include <indoor_positioning_system/Definitions.hpp>

namespace indoor_positioning_system_ros
{

// Convert from ROS quaternion to yaw angle
inline double convertQuaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quaternion, tf_quat);
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  tf2::Matrix3x3(tf_quat).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

// Convert yaw angle to ROS quaternion
inline geometry_msgs::msg::Quaternion convertYawToQuaternion(double yaw)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(tf_q);
}

// Build a ROS timestamp from a floating-point seconds value
inline rclcpp::Time convertSecondsToRosTime(double seconds)
{
  int32_t sec = static_cast<int32_t>(std::floor(seconds));
  uint32_t nsec = static_cast<uint32_t>((seconds - sec) * 1e9);
  return rclcpp::Time(sec, nsec);
}

//-----------------------------------------------------------------------------
// Conversion: ROS msg → Internal types
//-----------------------------------------------------------------------------

inline indoor_positioning_system::VehicleState convertStateFromMsg(
  const cpm_lab_lab_msgs::msg::VehicleState & state_msg)
{
  indoor_positioning_system::VehicleState state;
  state.vehicle_id = state_msg.vehicle_id;
  state.pose.x = state_msg.pose.position.x;
  state.pose.y = state_msg.pose.position.y;
  state.pose.yaw = convertQuaternionToYaw(state_msg.pose.orientation);
  state.seconds = rclcpp::Time(state_msg.header.stamp).seconds();
  state.speed = state_msg.twist.linear.x;
  state.imu_yaw_rate = state_msg.twist.angular.z;
  return state;
}

//-----------------------------------------------------------------------------
// Conversion: Internal types → ROS msg
//-----------------------------------------------------------------------------

inline cpm_lab_lab_msgs::msg::VehicleObservation
convertObservationToMsg(const indoor_positioning_system::VehicleObservation & obs)
{
  cpm_lab_lab_msgs::msg::VehicleObservation msg;
  msg.vehicle_id = obs.vehicle_id;
  msg.header.stamp = convertSecondsToRosTime(obs.seconds);
  msg.valid_after_stamp = msg.header.stamp;
  msg.pose.position.x = obs.pose.x;
  msg.pose.position.y = obs.pose.y;
  msg.pose.orientation = convertYawToQuaternion(obs.pose.yaw);
  return msg;
}

}  // namespace indoor_positioning_system_ros

#endif  // CPM_LAB_INDOOR_POSITIONING_SYSTEM__COMPONENT__INDOOR_POSITIONING_SYSTEM_ROS_HPP_
