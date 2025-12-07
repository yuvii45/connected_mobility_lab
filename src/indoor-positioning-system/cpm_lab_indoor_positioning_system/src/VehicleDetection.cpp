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

#include "indoor_positioning_system/VehicleDetection.hpp"
#include <iostream>
#include <set>

namespace indoor_positioning_system
{

// ---------------------------
// Constructor
// ---------------------------
VehicleDetection::VehicleDetection(const IndoorPositioningSystemParameter & parameters)
: distance_front_to_rear_(parameters.vehicleDetectionDistanceFrontToRear()),
  distance_rear_to_rear_(parameters.vehicleDetectionDistanceRearToRear()),
  angle_front_to_rear_(parameters.vehicleDetectionAngleFrontToRear())
{
}

// ---------------------------
// Public API
// ---------------------------
VehiclePointSets VehicleDetection::apply(const FloorPoints & floor_points) const
{
  // compute pairwise distances once
  const Eigen::MatrixXd point_distances = calc_point_distances(floor_points);

  // find vehicle candidates
  std::vector<VehicleCandidate> candidates =
    find_vehicle_candidates(floor_points, point_distances);

  // resolve conflicts (overlapping points)
  std::vector<VehicleCandidate> vehicles = resolve_conflicts(candidates);

  // turn candidates into VehiclePointSets
  VehiclePointSets vehicle_points;
  for (const auto & cand : vehicles) {
    vehicle_points.push_back(assign_vehicle_points(floor_points, cand));
  }

  return vehicle_points;
}

// ---------------------------
// Distance matrix
// ---------------------------
Eigen::MatrixXd VehicleDetection::calc_point_distances(const FloorPoints & points) const
{
  Eigen::MatrixXd dist(points.size(), points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      const double d = (points[i] - points[j]).norm();
      dist(i, j) = dist(j, i) = d;
    }
  }
  return dist;
}

// ---------------------------
// Candidate detection
// ---------------------------
std::vector<VehicleCandidate> VehicleDetection::find_vehicle_candidates(
  const FloorPoints & points,
  const Eigen::MatrixXd & point_distances) const
{
  std::vector<VehicleCandidate> candidates;
  if (points.size() < 3) {return {};}

  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = 0; j < points.size(); ++j) {
      for (std::size_t k = 0; k < points.size(); ++k) {
        if(i == j || j == k || i == k) {continue;}

        const double rear_tol = distance_rear_to_rear_ * PERCENTAGE_TOLERANCE;
        const double front_tol = distance_front_to_rear_ * PERCENTAGE_TOLERANCE;

        if(std::abs(distance_rear_to_rear_ - point_distances(j, k)) > rear_tol) {
          continue;
        }
        if(std::abs(distance_front_to_rear_ - point_distances(i, k)) > front_tol) {
          continue;
        }
        if(std::abs(distance_front_to_rear_ - point_distances(i, j)) > front_tol) {
          continue;
        }

        // Valid canidate found distance wise now check angles
        VehicleCandidate cand{i, j, k, std::nullopt};
        VehiclePointSet vps = assign_vehicle_points(points, cand);
        // angle check at front point
        Eigen::Vector2d v1 = vps.back_left - vps.front;
        Eigen::Vector2d v2 = vps.back_right - vps.front;

        double cos_angle = v1.dot(v2) / (v1.norm() * v2.norm());
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);

        const double angle_deg = std::acos(cos_angle) * 180.0 / M_PI;
        const double tol = angle_front_to_rear_ * PERCENTAGE_TOLERANCE;
        const double angle_difference = std::abs(angle_deg - angle_front_to_rear_);

        if (angle_difference > tol) {
          continue;  // invalid orientation
        }

        // ID LED detection near weighted center
        FloorPoint center_est =
          0.270491803 * (vps.back_left + vps.back_right) +
          0.459016393 * vps.front;

        for (std::size_t l = 0; l < points.size(); ++l) {
          if ((center_est - points[l]).norm() < 0.05) {
            cand.id_led_idx = l;
            break;
          }
        }

        candidates.push_back(cand);
      }
    }
  }
  return candidates;
}

// ---------------------------
// Conflict resolution
// ---------------------------
static bool sharesPoint(const VehicleCandidate & a, const VehicleCandidate & b)
{
  std::set<std::size_t> pa = {a.front_idx, a.back_left_idx, a.back_right_idx};
  if (a.id_led_idx) {pa.insert(*a.id_led_idx);}

  std::set<std::size_t> pb = {b.front_idx, b.back_left_idx, b.back_right_idx};
  if (b.id_led_idx) {pb.insert(*b.id_led_idx);}

  for (auto idx : pa) {
    if (pb.count(idx)) {return true;}
  }
  return false;
}

std::vector<VehicleCandidate> VehicleDetection::resolve_conflicts(
  const std::vector<VehicleCandidate> & candidates) const
{
  if (candidates.size() <= 1) {return candidates;}

    // Copy candidates so we can remove elements
  std::vector<VehicleCandidate> unresolved = candidates;
  std::vector<VehicleCandidate> resolved;

  while (!unresolved.empty()) {
        // Compute conflict count for each candidate
    std::vector<size_t> conflict_counts(unresolved.size(), 0);

    for (size_t i = 0; i < unresolved.size(); ++i) {
      for (size_t j = 0; j < unresolved.size(); ++j) {
        if (i != j && sharesPoint(unresolved[i], unresolved[j])) {
          conflict_counts[i]++;
        }
      }
    }

        // Find candidate with minimal conflicts (best candidate)
    auto best_it = std::min_element(conflict_counts.begin(), conflict_counts.end());
    size_t best_idx = std::distance(conflict_counts.begin(), best_it);

        // Add best candidate to resolved
    resolved.push_back(unresolved[best_idx]);

        // Remove best candidate and all candidates that share points with it
    std::vector<VehicleCandidate> next_unresolved;
    for (size_t i = 0; i < unresolved.size(); ++i) {
      if (i == best_idx) {continue;}      // skip already added
      if (!sharesPoint(unresolved[best_idx], unresolved[i])) {
        next_unresolved.push_back(unresolved[i]);
      }
    }

    unresolved = std::move(next_unresolved);
  }

  return resolved;
}


// ---------------------------
// Candidate → VehiclePointSet
// ---------------------------
VehiclePointSet VehicleDetection::assign_vehicle_points(
  const FloorPoints & floor_points,
  const VehicleCandidate & cand) const
{
  VehiclePointSet vps;
  const FloorPoint & front = floor_points[cand.front_idx];
  const FloorPoint & r0 = floor_points[cand.back_left_idx];
  const FloorPoint & r1 = floor_points[cand.back_right_idx];

  auto cross2d = [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      return a.x() * b.y() - a.y() * b.x();
    };

  if (cross2d(r0 - front, r1 - front) > 0) {
    vps.back_left = r0;
    vps.back_right = r1;
  } else {
    vps.back_left = r1;
    vps.back_right = r0;
  }

  vps.front = front;

  if (cand.id_led_idx) {
    vps.center_present = true;
    vps.center = floor_points[*cand.id_led_idx];
  } else {
    vps.center_present = false;
  }

  return vps;
}

}  // namespace indoor_positioning_system
