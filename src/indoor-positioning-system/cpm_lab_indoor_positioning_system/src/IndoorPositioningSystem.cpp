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


#include "indoor_positioning_system/IndoorPositioningSystem.hpp"

#include <iostream>

namespace indoor_positioning_system
{

IndoorPositioningSystem::IndoorPositioningSystem(
  const IndoorPositioningSystemParameter & parameters)
{
  led_detection_ = std::make_shared<LedDetection>(parameters);
  point_undistortion_ = std::make_shared<PointUndistortion>(parameters);
  point_projection_ = std::make_shared<PointProjection>(parameters);
  vehicle_detection_ = std::make_shared<VehicleDetection>(parameters);
  vehicle_identification_ = std::make_shared<VehicleIdentification>(parameters);
  pose_calculation_ = std::make_shared<PoseCalculation>(parameters);
  feedback_identifier_ = std::make_shared<VehicleTracking>(parameters);
}

bool IndoorPositioningSystem::detectionSucceeded() const
{
  return sussessful_;
}

const std::string & IndoorPositioningSystem::getErrorMessage() const
{
  return error_message_;
}

bool IndoorPositioningSystem::hasWarning() const
{
  return warning_;
}

const std::string & IndoorPositioningSystem::getWarningMessage() const
{
  return warning_message_;
}

void IndoorPositioningSystem::idRequestSendToVehicle(const int vehicle_id)
{
  vehicle_identification_->setRequestedVehicleId(vehicle_id);
}

int IndoorPositioningSystem::getRequestedVehicleId()
{
  return feedback_identifier_->getRequestedVehicleId();
}

const ImagePoints & IndoorPositioningSystem::getLedPoints() const
{
  return led_points_;
}

const FloorPoints & IndoorPositioningSystem::getFloorPoints() const
{
  return floor_points_;
}

const std::vector<VehicleObservation> & IndoorPositioningSystem::getVehicleObservations() const
{
  return vehicle_observations_;
}

void IndoorPositioningSystem::updateVehicleState(const VehicleState & vehicle_state)
{
  // Update the vehicle state in the map
  vehicle_states_[vehicle_state.vehicle_id] = vehicle_state;
}

void IndoorPositioningSystem::apply(const cv::Mat & image, const double frame_time)
{
  sussessful_ = true;
  error_message_ = "";
  warning_ = false;
  warning_message_ = "";


  led_points_ = led_detection_->apply(image);

  if (led_points_.empty()) {
    error_message_ = "No LED points detected.";
    sussessful_ = false;
    return;
  }

  if (led_points_.size() > 100) {
    error_message_ = "Too many LED points detected: " +
      std::to_string(led_points_.size());
    sussessful_ = false;
    return;
  }

  image_points_ = point_undistortion_->apply(led_points_);
  floor_points_ = point_projection_->apply(image_points_);
  possible_vehicle_points_ = vehicle_detection_->apply(floor_points_);
  vehicle_points_ = vehicle_identification_->apply(possible_vehicle_points_);
  vehicle_observations_ = pose_calculation_->apply(vehicle_points_, frame_time);
  feedback_identifier_->assign(vehicle_observations_, frame_time, vehicle_states_);

  const size_t unassigned = std::count_if(
  vehicle_observations_.begin(), vehicle_observations_.end(),
    [](const auto & obs) {return obs.vehicle_id < 1;});

  if (unassigned != 0) {
    warning_message_ = "Detected " + std::to_string(led_points_.size()) +
      " LED points resulting in " + std::to_string(vehicle_observations_.size()) +
      " observations but " + std::to_string(unassigned) + " of them could not be identified.";
    warning_ = true;
  }
}

void IndoorPositioningSystem::resetTracking()
{
  // TODO(dummy): implement reset tracking
}

}  // namespace indoor_positioning_system
