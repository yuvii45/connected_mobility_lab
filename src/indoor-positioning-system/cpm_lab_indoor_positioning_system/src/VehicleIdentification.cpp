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


#include <iostream>
#include <utility>

#include "indoor_positioning_system/VehicleIdentification.hpp"

namespace indoor_positioning_system
{
VehicleIdentification::VehicleIdentification(
  const IndoorPositioningSystemParameter & /*parameters*/)
{
}

void VehicleIdentification::reset()
{
  id_request_matched_ = true;
  requested_vehicle_id_ = 0;
}

void VehicleIdentification::setRequestedVehicleId(const int vehicle_id)
{
  id_request_matched_ = false;
  requested_vehicle_id_ = vehicle_id;
}

VehiclePointSets VehicleIdentification::apply(const VehiclePointSets & vehicle_points)
{
  VehiclePointSets result = vehicle_points;

  if (id_request_matched_) {
    return result;
  }


  for (auto & point_set : result) {
    if(point_set.id != 0) {continue;}
    if(!point_set.center_present) {continue;}

    point_set.id = requested_vehicle_id_;
    std::cout <<
      "Assign vehicle id " + std::to_string(requested_vehicle_id_) + " via center LED." <<
      std::endl;

    reset();
  }

  return result;
}
}  // namespace indoor_positioning_system
