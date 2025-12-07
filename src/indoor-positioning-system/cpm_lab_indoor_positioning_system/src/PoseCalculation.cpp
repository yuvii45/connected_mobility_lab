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


#include "indoor_positioning_system/PoseCalculation.hpp"

namespace indoor_positioning_system
{
PoseCalculation::PoseCalculation(const IndoorPositioningSystemParameter & /*parameters*/)
{
}

VehicleObservations PoseCalculation::apply(
  const VehiclePointSets & vehicle_points,
  const double frame_time)
{
  VehicleObservations result;

  for (const auto & vehicle : vehicle_points) {
    const FloorPoint front = vehicle.front;
    const FloorPoint back = 0.5 * (vehicle.back_left + vehicle.back_right);
    const FloorPoint center = 0.5 * (front + back);
    const FloorPoint direction = (front - back).normalized();

    VehicleObservation vehicleObservation;
    vehicleObservation.pose.x = center.x();
    vehicleObservation.pose.y = center.y();
    vehicleObservation.pose.yaw = std::atan2(direction.y(), direction.x());
    vehicleObservation.vehicle_id = vehicle.id;
    vehicleObservation.seconds = frame_time;
    result.push_back(vehicleObservation);
  }

  return result;
}
}  // namespace indoor_positioning_system
