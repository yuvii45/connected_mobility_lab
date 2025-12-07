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


#include "indoor_positioning_system/VehicleTracking.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <numeric>
#include "third_party/association_functions.hpp"

namespace indoor_positioning_system
{

inline float angleDifference(float a, float b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}


VehicleTracking::VehicleTracking(const IndoorPositioningSystemParameter & parameters)
:max_association_cost_(parameters.vehicleTrackingMaxAssociationCost())
{
}


int VehicleTracking::getRequestedVehicleId() const
{
  return this->first_missed_vehicle_id_;
}

void VehicleTracking::assign(
  std::vector<VehicleObservation> & observations, const double current_time,
  const std::unordered_map<int, VehicleState> & latest_vehicle_states)
{
  this->first_missed_vehicle_id_ = 0;  // Reset the first missed vehicle ID

  std::vector<int> predicted_ids;
  std::vector<Pose2D> predicted_poses;

  for (const auto & [id, state] : latest_vehicle_states) {
    for (const auto & obs : observations) {
      if (obs.vehicle_id == id) {
        continue;  // Skip already assigned IDs
      }
    }
    const double state_time = state.seconds;
    const double dt = current_time - state_time;

    Pose2D predicted;
    predicted.x = state.pose.x + std::cos(state.pose.yaw) * state.speed * dt;
    predicted.y = state.pose.y + std::sin(state.pose.yaw) * state.speed * dt;
    predicted.yaw = state.pose.yaw + state.imu_yaw_rate * dt;

    predicted_ids.push_back(id);
    predicted_poses.push_back(predicted);
  }

  const int N = predicted_poses.size();
  const int M = observations.size();
  if (N == 0 || M == 0) {
    return;
  }

  std::vector<int> observation_indices;
  for (int j = 0; j < M; ++j) {
    if (observations[j].vehicle_id < 1 || observations[j].vehicle_id > 20) {
      observation_indices.push_back(j);
    }
  }

  const int U = observation_indices.size();
  if (U == 0) {
    return;
  }

  Eigen::MatrixXf cost_matrix(N, U);
  float theta_weight = 0.2f;
  for (int i = 0; i < N; ++i) {
    for (int uj = 0; uj < U; ++uj) {
      const int j = observation_indices[uj];
      const auto & pred = predicted_poses[i];
      const auto & obs = observations[j].pose;

      float dx = pred.x - obs.x;
      float dy = pred.y - obs.y;
      float dtheta = angleDifference(pred.yaw, obs.yaw);

      cost_matrix(i, uj) = std::sqrt(dx * dx + dy * dy) + theta_weight * std::abs(dtheta);
    }
  }

  AssignmentMatrix assignment_matrix(N, U);
  hungarian(cost_matrix, assignment_matrix);

  std::unordered_set<int> used_ids;

  for (int i = 0; i < N; ++i) {
    for (int uj = 0; uj < U; ++uj) {
      if (assignment_matrix(i, uj) == 1) {
        int id = predicted_ids[i];
        int obs_idx = observation_indices[uj];
        float cost = cost_matrix(i, uj);
        observations[obs_idx].delta_pose = cost;
        if (cost > max_association_cost_) {
          std::cerr << "Assigner reject: ID " << id << " → Obs " << obs_idx << " (cost=" << cost <<
            ")" << std::endl;
          continue;
        }

        if (used_ids.count(id)) {
          std::cerr << "Assigner conflict: ID " << id << "already used" << std::endl;
          continue;
        }

        observations[obs_idx].vehicle_id = id;
        used_ids.insert(id);

        // std::cout << "Assigner assigned: ID " << id << " → Obs "
        // << obs_idx << " (cost=" << cost <<
        //   ")" << std::endl;
      }
    }
  }

  size_t unassigned = std::count_if(observations.begin(), observations.end(),
      [](const auto & obs) {return obs.vehicle_id < 1 || obs.vehicle_id > 20;});

  // if (unassigned == 0) {
  //   std::cout << "✅ All " << observations.size() <<
  //     " observations received valid IDs via feedback." << std::endl;
  // } else {
  //   std::cout << "❌ " << unassigned << " of " << observations.size() <<
  //     " observations were unassigned after feedback."
  //             << std::endl;
  // }

  for (const auto & state : latest_vehicle_states) {
    if (std::none_of(observations.begin(), observations.end(),
      [&state](const VehicleObservation & obs) {return obs.vehicle_id == state.first;}))
    {
      this->first_missed_vehicle_id_ = state.first;
      // std::cout << "Requested vehicle ID updated to: " << this->first_missed_vehicle_id_ <<
      // std::endl;
      break;
    }
  }
}
}  // namespace indoor_positioning_system
