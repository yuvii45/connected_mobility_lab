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

#ifndef INDOOR_POSITIONING_SYSTEM__DEFINITIONS_HPP__
#define INDOOR_POSITIONING_SYSTEM__DEFINITIONS_HPP__

#if defined __has_include && __has_include ("cv_bridge/cv_bridge.h")
#include <cv_bridge/cv_bridge.h>
#endif

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <list>

#if defined __has_include && __has_include ("cv_bridge/cv_bridge.hpp")
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace indoor_positioning_system
{

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

struct VehicleObservation
{
  int vehicle_id;
  double seconds;
  Pose2D pose;
  double delta_pose;
};
using VehicleObservations = std::vector<VehicleObservation>;

struct VehicleState
{
  int vehicle_id;
  double seconds;
  Pose2D pose;
  double speed;
  double imu_yaw_rate;
  double ips_update_age;
};
using VehicleStates = std::vector<VehicleState>;

using ImagePoint = cv::Point2d;
using ImagePoints = std::vector<ImagePoint>;

using FloorPoint = Eigen::Vector2d;
using FloorPoints = std::vector<FloorPoint>;


struct VehiclePointSet
{
  int id = 0;
  bool center_present = false;

  FloorPoint front;
  FloorPoint center;
  FloorPoint back_left;
  FloorPoint back_right;
};

using VehiclePointSets = std::vector<VehiclePointSet>;
using VehiclePoints = VehiclePointSets;
using VehiclePointTimeseries = std::vector<VehiclePointSets>;

}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__DEFINITIONS_HPP__
