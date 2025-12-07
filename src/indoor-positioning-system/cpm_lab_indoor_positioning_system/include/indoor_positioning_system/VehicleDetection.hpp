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


#ifndef INDOOR_POSITIONING_SYSTEM__VEHICLEDETECTION_HPP_
#define INDOOR_POSITIONING_SYSTEM__VEHICLEDETECTION_HPP_

#include <array>
#include <vector>
#include <optional>

#include "indoor_positioning_system/Definitions.hpp"
#include "indoor_positioning_system/IndoorPositioningSystemParameter.hpp"

namespace Eigen
{
using MatrixXb = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
}  // namespace Eigen

namespace indoor_positioning_system
{

  // ---------------------------
// Helper struct
// ---------------------------
struct VehicleCandidate
{
  std::size_t front_idx;
  std::size_t back_left_idx;
  std::size_t back_right_idx;
  std::optional<std::size_t> id_led_idx;  // empty if none
};

class VehicleDetection
{
private:
  const double PERCENTAGE_TOLERANCE = 0.2;
  const double distance_front_to_rear_;
  const double distance_rear_to_rear_;
  const double angle_front_to_rear_;

  [[nodiscard]] Eigen::MatrixXd calc_point_distances(
    const FloorPoints & points) const;

  [[nodiscard]] std::vector<VehicleCandidate> find_vehicle_candidates(
    const FloorPoints & points,
    const Eigen::MatrixXd & point_distances) const;

  [[nodiscard]] std::vector<VehicleCandidate> resolve_conflicts(
    const std::vector<VehicleCandidate> & candidates) const;

  [[nodiscard]] VehiclePointSet assign_vehicle_points(
    const FloorPoints & floor_points,
    const VehicleCandidate & cand) const;

public:
  explicit VehicleDetection(const IndoorPositioningSystemParameter & parameters);

  [[nodiscard]] VehiclePointSets apply(const FloorPoints & floor_points) const;
};
}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__VEHICLEDETECTION_HPP_
