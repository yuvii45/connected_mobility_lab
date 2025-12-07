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


#ifndef INDOOR_POSITIONING_SYSTEM__VEHICLETRACKING_HPP_
#define INDOOR_POSITIONING_SYSTEM__VEHICLETRACKING_HPP_

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include "indoor_positioning_system/Definitions.hpp"
#include "indoor_positioning_system/IndoorPositioningSystemParameter.hpp"

namespace indoor_positioning_system
{

class VehicleTracking
{
public:
  explicit VehicleTracking(const IndoorPositioningSystemParameter & parameters);

  void assign(
    std::vector<VehicleObservation> & observations, const double current_time,
    const std::unordered_map<int, VehicleState> & latest_vehicle_states);

  int getRequestedVehicleId() const;

private:
  int first_missed_vehicle_id_ = 0;
  const float max_association_cost_;
};

}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__VEHICLETRACKING_HPP_
