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


#include "indoor_positioning_system/IndoorPositioningSystemParameter.hpp"

namespace indoor_positioning_system
{

int IndoorPositioningSystemParameter::numberVehicles() const
{
  return number_vehicles_;
}

int IndoorPositioningSystemParameter::ledDetectionMinBrightnessThreshold() const
{
  return led_detection_min_brightness_threshold_;
}

int IndoorPositioningSystemParameter::ledDetectionMaxBrightnessThreshold() const
{
  return led_detection_max_brightness_threshold_;
}

int IndoorPositioningSystemParameter::ledDetectionMinContourSize() const
{
  return led_detection_min_contour_size_;
}

int IndoorPositioningSystemParameter::ledDetectionMaxContourSize() const
{
  return led_detection_max_contour_size_;
}

double IndoorPositioningSystemParameter::pointProjectionLedHeight() const
{
  return point_projection_led_height_;
}

const std::vector<double> & IndoorPositioningSystemParameter::pointUndistortionCameraMatrix() const
{
  return point_undistortion_camera_matrix_;
}

const std::vector<double> & IndoorPositioningSystemParameter::
pointUndistortionDistortionCoefficients() const
{
  return point_undistortion_distortion_coefficients_;
}

const std::vector<double> & IndoorPositioningSystemParameter::pointProjectionRotationVector() const
{
  return point_projection_rotation_vector_;
}

const std::vector<double> & IndoorPositioningSystemParameter::pointProjectionTranslationVector()
const
{
  return point_projection_translation_vector_;
}

const std::vector<double> & IndoorPositioningSystemParameter::pointProjectionFinalTransformation()
const
{
  return point_projection_final_transformation_;
}

double IndoorPositioningSystemParameter::vehicleDetectionDistanceFrontToRear() const
{
  return vehicle_detection_distance_front_to_rear_;
}

double IndoorPositioningSystemParameter::vehicleDetectionDistanceRearToRear() const
{
  return vehicle_detection_distance_rear_to_rear_;
}

double IndoorPositioningSystemParameter::vehicleDetectionAngleFrontToRear() const
{
  return vehicle_detection_angle_front_to_rear_;
}

double IndoorPositioningSystemParameter::vehicleTrackingMaxAssociationCost() const
{
  return vehicle_tracking_max_association_cost_;
}

}  // namespace indoor_positioning_system
