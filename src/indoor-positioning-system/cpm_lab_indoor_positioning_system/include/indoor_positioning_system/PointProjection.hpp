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

#ifndef INDOOR_POSITIONING_SYSTEM__POINTPROJECTION_HPP_
#define INDOOR_POSITIONING_SYSTEM__POINTPROJECTION_HPP_

#include <vector>

#include "indoor_positioning_system/Definitions.hpp"
#include "indoor_positioning_system/IndoorPositioningSystemParameter.hpp"

namespace indoor_positioning_system
{

class PointProjection
{
private:
  // Extrinsics
  Eigen::Vector3d rotation_vector_;     // Rodrigues form
  Eigen::Vector3d translation_vector_;  // Camera translation

  // Homography matrices (projective transforms)
  Eigen::Projective2d homography_;
  Eigen::Projective2d homographyinv_;

  // Final lab transformation
  Eigen::Projective2d final_transformation_;

  // Height of vehicle LEDs to correct for
  const double led_height_;

public:
  explicit PointProjection(const IndoorPositioningSystemParameter & parameters);

  FloorPoints apply(const ImagePoints & undistorted_points);
};

}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__POINTPROJECTION_HPP_
