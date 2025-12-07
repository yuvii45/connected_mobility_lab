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


#ifndef INDOOR_POSITIONING_SYSTEM__INDOORPOSITIONINGSYSTEMPARAMETER_HPP_
#define INDOOR_POSITIONING_SYSTEM__INDOORPOSITIONINGSYSTEMPARAMETER_HPP_

#include <vector>
#include <string>

namespace indoor_positioning_system
{

class IndoorPositioningSystemParameter
{
public:
  static constexpr int N_CALIBRATION_TERMS = 15;
  static constexpr int N_POSE_CALIBRATION_TERMS = 5;

protected:
  int number_vehicles_ = 0;

  int led_detection_min_brightness_threshold_ = 0;
  int led_detection_max_brightness_threshold_ = 255;
  int led_detection_min_contour_size_ = 0;
  int led_detection_max_contour_size_ = 1000;

  double point_projection_led_height_ = 0.0;

  std::vector<double> point_undistortion_camera_matrix_{};
  std::vector<double> point_undistortion_distortion_coefficients_{};
  std::vector<double> point_projection_rotation_vector_{};
  std::vector<double> point_projection_translation_vector_{};
  std::vector<double> point_projection_final_transformation_{};

  double vehicle_detection_distance_front_to_rear_ = 0.0;
  double vehicle_detection_distance_rear_to_rear_ = 0.0;
  double vehicle_detection_angle_front_to_rear_ = 0.0;

  double vehicle_tracking_max_association_cost_ = 0.5;

public:
  [[nodiscard]] int numberVehicles() const;

  [[nodiscard]] int ledDetectionMinBrightnessThreshold() const;

  [[nodiscard]] int ledDetectionMaxBrightnessThreshold() const;

  [[nodiscard]] int ledDetectionMinContourSize() const;

  [[nodiscard]] int ledDetectionMaxContourSize() const;

  [[nodiscard]] double pointProjectionLedHeight() const;

  [[nodiscard]] const std::vector<double> & pointUndistortionCameraMatrix() const;

  [[nodiscard]] const std::vector<double> & pointUndistortionDistortionCoefficients() const;

  [[nodiscard]] const std::vector<double> & pointProjectionRotationVector() const;

  [[nodiscard]] const std::vector<double> & pointProjectionTranslationVector() const;

  [[nodiscard]] const std::vector<double> & pointProjectionFinalTransformation() const;

  [[nodiscard]] double vehicleDetectionDistanceFrontToRear() const;

  [[nodiscard]] double vehicleDetectionDistanceRearToRear() const;

  [[nodiscard]] double vehicleDetectionAngleFrontToRear() const;

  [[nodiscard]] double vehicleTrackingMaxAssociationCost() const;
};

}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__INDOORPOSITIONINGSYSTEMPARAMETER_HPP_
