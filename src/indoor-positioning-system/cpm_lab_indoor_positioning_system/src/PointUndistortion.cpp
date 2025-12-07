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

#include <opencv2/opencv.hpp>

#include "indoor_positioning_system/PointUndistortion.hpp"

namespace indoor_positioning_system
{

PointUndistortion::PointUndistortion(const IndoorPositioningSystemParameter & parameters)
{
  // --- Intrinsics ---
  const auto & camera_matrix = parameters.pointUndistortionCameraMatrix();
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    camera_matrix_.at<double>(i / 3, i % 3) = camera_matrix[i];
  }

  const auto & distortion_coeffs = parameters.pointUndistortionDistortionCoefficients();
  distortion_coeffs_ = cv::Mat(1, distortion_coeffs.size(), CV_64F);
  for (size_t i = 0; i < distortion_coeffs.size(); ++i) {
    distortion_coeffs_.at<double>(0, i) = distortion_coeffs[i];
  }
}

ImagePoints PointUndistortion::apply(const ImagePoints & led_points)
{
  // Quit early if possible
  if (led_points.empty()) {
    return {};
  }

  // Undistort image points -> normalized camera coordinates
  ImagePoints undistorted_points;
  cv::undistortPoints(led_points, undistorted_points, camera_matrix_, distortion_coeffs_);
  // NOTE: undistorted_points are already in normalized coordinates (x/z, y/z).

  return undistorted_points;
}

}  // namespace indoor_positioning_system
