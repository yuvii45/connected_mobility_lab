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


#ifndef INDOOR_POSITIONING_SYSTEM__INDOORPOSITIONINGSYSTEM_HPP_
#define INDOOR_POSITIONING_SYSTEM__INDOORPOSITIONINGSYSTEM_HPP_

#include <unordered_map>
#include <memory>
#include <vector>
#include <string>

#include "indoor_positioning_system/Definitions.hpp"
#include "indoor_positioning_system/VehicleDetection.hpp"
#include "indoor_positioning_system/VehicleIdentification.hpp"
#include "indoor_positioning_system/PoseCalculation.hpp"
#include "indoor_positioning_system/PointUndistortion.hpp"
#include "indoor_positioning_system/LedDetection.hpp"
#include "indoor_positioning_system/VehicleTracking.hpp"

#include "indoor_positioning_system/IndoorPositioningSystemParameter.hpp"

namespace indoor_positioning_system
{

typedef std::shared_ptr<class IndoorPositioningSystem> IndoorPositioningSystemPtr;

class IndoorPositioningSystem
{
private:
  std::shared_ptr<LedDetection> led_detection_;
  std::shared_ptr<PointUndistortion> point_undistortion_;
  std::shared_ptr<VehicleDetection> vehicle_detection_;
  std::shared_ptr<VehicleIdentification> vehicle_identification_;
  std::shared_ptr<PoseCalculation> pose_calculation_;
  std::shared_ptr<VehicleTracking> feedback_identifier_;

  ImagePoints led_points_;
  ImagePoints image_points_;
  FloorPoints floor_points_;
  VehiclePointSets possible_vehicle_points_;
  VehiclePointSets vehicle_points_;
  VehiclePointTimeseries vehicle_point_history_;
  std::vector<VehicleObservation> vehicle_observations_;
  std::unordered_map<int, VehicleState> vehicle_states_;

  bool sussessful_ = true;
  std::string error_message_ = "";

  bool warning_ = true;
  std::string warning_message_ = "";

public:
  explicit IndoorPositioningSystem(const IndoorPositioningSystemParameter & parameters);

  [[nodiscard]] const std::vector<VehicleObservation> & getVehicleObservations() const;

  [[nodiscard]] const ImagePoints & getLedPoints() const;
  [[nodiscard]] const FloorPoints & getFloorPoints() const;

  bool detectionSucceeded() const;

  const std::string & getErrorMessage() const;

  bool hasWarning() const;

  const std::string & getWarningMessage() const;

  void idRequestSendToVehicle(const int vehicle_id);

  int getRequestedVehicleId();

  void updateVehicleState(const VehicleState & vehicle_state);

  void apply(const cv::Mat & image, const double frame_time);

  void resetTracking();
};

}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__INDOORPOSITIONINGSYSTEM_HPP_
