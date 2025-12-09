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
  // --- PROJECTION CALIBRATION ---
  calibration_x_ = parameters.getProjectionCalibrationX();
  calibration_y_ = parameters.getProjectionCalibrationY();
  
  // --- IMAGE SCALE ---
  projection_image_scale_ = parameters.getProjectionImageScale();
  
  // --- CAMERA POSITION ---
  projection_camera_x_ = parameters.getProjectionCameraX();
  projection_camera_y_ = parameters.getProjectionCameraY();
  projection_camera_z_ = parameters.getProjectionCameraZ();
  
  // --- LED Z POSITION ---
  projection_led_z_ = parameters.getProjectionLedZ();
}

ImagePoints PointUndistortion::apply(const ImagePoints & led_points)
{
  ImagePoints result;

  for (const auto &image_point: led_points) {
      const double image_x = image_point.x / this->projection_image_scale_;
      const double image_y = image_point.y / this->projection_image_scale_;

      // Calculate monomials
      const double ix1 = image_x;
      const double ix2 = ix1 * image_x;
      const double ix3 = ix2 * image_x;
      const double ix4 = ix3 * image_x;

      const double iy1 = image_y;
      const double iy2 = iy1 * image_y;
      const double iy3 = iy2 * image_y;
      const double iy4 = iy3 * image_y;

      // Calibration based on two-dimensional, 4th order polynomial
      const double features[] = {
              1,
              ix1, iy1,
              ix2, ix1 * iy1, iy2,
              ix3, ix2 * iy1, ix1 * iy2, iy3,
              ix4, ix3 * iy1, ix2 * iy2, ix1 * iy3, iy4
      };

      double floor_x = 0;
      double floor_y = 0;
      for (size_t i = 0; i < calibration_x_.size(); ++i) {
          floor_x += features[i] * calibration_x_[i];
          floor_y += features[i] * calibration_y_[i];
      }

      // The following calculation corrects for the error that is introduced, because
      // the LEDs are not on the ground, but a small distance above it.
      // This calibration does not need to be very precise, since another measurement based
      // calibration for the poses will correct small errors.
      const double scale_factor = (this->projection_camera_z_ - this->projection_led_z_) / this->projection_camera_z_;

      floor_x = scale_factor * (floor_x - this->projection_camera_x_) + this->projection_camera_x_;
      floor_y = scale_factor * (floor_y - this->projection_camera_y_) + this->projection_camera_y_;

      result.emplace_back(floor_x, floor_y);
    }

    return result;
  }
}  
