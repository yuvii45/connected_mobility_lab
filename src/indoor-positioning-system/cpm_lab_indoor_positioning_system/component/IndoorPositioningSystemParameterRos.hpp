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


#ifndef CPM_LAB_INDOOR_POSITIONING_SYSTEM__COMPONENT__INDOORPOSITIONINGSYSTEMPARAMETERROS_HPP_
#define CPM_LAB_INDOOR_POSITIONING_SYSTEM__COMPONENT__INDOORPOSITIONINGSYSTEMPARAMETERROS_HPP_

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <indoor_positioning_system/IndoorPositioningSystemParameter.hpp>

namespace indoor_positioning_system_ros
{

class IndoorPositioningSystemParameterRos : public indoor_positioning_system::
  IndoorPositioningSystemParameter
{
public:
  /**
   * Read the parameters from the parameter server.
   * If invalid parameters can be detected, the interface will reset them
   * to the default values.
   * @param nh the ros::NodeHandle to use
   */
  void readFromRosParameterServer(rclcpp::Node & nh)
  {
    auto logger = rclcpp::get_logger("IPS");

    RCLCPP_DEBUG(logger, "Reading parameters from ROS2 server");

    RCLCPP_DEBUG(logger, "---> number_vehicles");
    nh.declare_parameter("number_vehicles", rclcpp::PARAMETER_INTEGER);
    nh.get_parameter("number_vehicles", number_vehicles_);
    RCLCPP_INFO(logger, "number_vehicles: %i", number_vehicles_);

    RCLCPP_DEBUG(logger, "---> led_detection_min_contour_size");
    nh.declare_parameter("led_detection_min_contour_size", rclcpp::PARAMETER_INTEGER);
    nh.get_parameter("led_detection_min_contour_size", led_detection_min_contour_size_);
    RCLCPP_INFO(logger, "led_detection_min_contour_size: %i", led_detection_min_contour_size_);

    RCLCPP_DEBUG(logger, "---> led_detection_max_contour_size");
    nh.declare_parameter("led_detection_max_contour_size", rclcpp::PARAMETER_INTEGER);
    nh.get_parameter("led_detection_max_contour_size", led_detection_max_contour_size_);
    RCLCPP_INFO(logger, "led_detection_max_contour_size: %i", led_detection_max_contour_size_);

    RCLCPP_DEBUG(logger, "---> led_detection_min_brightness_threshold");
    nh.declare_parameter("led_detection_min_brightness_threshold", rclcpp::PARAMETER_INTEGER);
    nh.get_parameter("led_detection_min_brightness_threshold",
        led_detection_min_brightness_threshold_);
    RCLCPP_INFO(logger, "led_detection_min_brightness_threshold: %i",
        led_detection_min_brightness_threshold_);

    RCLCPP_DEBUG(logger, "---> led_detection_max_brightness_threshold");
    nh.declare_parameter("led_detection_max_brightness_threshold", rclcpp::PARAMETER_INTEGER);
    nh.get_parameter("led_detection_max_brightness_threshold",
        led_detection_max_brightness_threshold_);
    RCLCPP_INFO(logger, "led_detection_max_brightness_threshold: %i",
        led_detection_max_brightness_threshold_);

    RCLCPP_DEBUG(logger, "---> point_projection_led_height");
    nh.declare_parameter("point_projection_led_height", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("point_projection_led_height", point_projection_led_height_);
    RCLCPP_INFO(logger, "point_projection_led_height: %f", point_projection_led_height_);

    RCLCPP_DEBUG(logger, "---> vehicle_detection_distance_front_to_rear");
    nh.declare_parameter("vehicle_detection_distance_front_to_rear", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("vehicle_detection_distance_front_to_rear",
        vehicle_detection_distance_front_to_rear_);
    RCLCPP_INFO(logger, "vehicle_detection_distance_front_to_rear: %f",
        vehicle_detection_distance_front_to_rear_);

    RCLCPP_DEBUG(logger, "---> vehicle_detection_distance_rear_to_rear");
    nh.declare_parameter("vehicle_detection_distance_rear_to_rear", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("vehicle_detection_distance_rear_to_rear",
        vehicle_detection_distance_rear_to_rear_);
    RCLCPP_INFO(logger, "vehicle_detection_distance_rear_to_rear: %f",
        vehicle_detection_distance_rear_to_rear_);

    RCLCPP_DEBUG(logger, "---> vehicle_detection_angle_front_to_rear");
    nh.declare_parameter("vehicle_detection_angle_front_to_rear", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("vehicle_detection_angle_front_to_rear",
        vehicle_detection_angle_front_to_rear_);
    RCLCPP_INFO(logger, "vehicle_detection_angle_front_to_rear: %f",
        vehicle_detection_angle_front_to_rear_);

    RCLCPP_DEBUG(logger, "---> vehicle_tracking_max_association_cost");
    nh.declare_parameter("vehicle_tracking_max_association_cost", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("vehicle_tracking_max_association_cost",
        vehicle_tracking_max_association_cost_);
    RCLCPP_INFO(logger, "vehicle_tracking_max_association_cost: %f",
        vehicle_tracking_max_association_cost_);

    // Declare and load camera matrix
    RCLCPP_DEBUG(logger, "---> point_undistortion_camera_matrix");
    if (!nh.has_parameter("point_undistortion_camera_matrix")) {
      nh.declare_parameter("point_undistortion_camera_matrix", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("point_undistortion_camera_matrix", point_undistortion_camera_matrix_);
    RCLCPP_INFO(logger, "Camera Matrix: %zu elements", point_undistortion_camera_matrix_.size());
    if (!point_undistortion_camera_matrix_.empty()) {
      for (const auto & value : point_undistortion_camera_matrix_) {
        RCLCPP_INFO(logger, "Camera Matrix Value: %f", value);
      }
    }

    RCLCPP_DEBUG(logger, "---> point_undistortion_distortion_coefficients");
    if (!nh.has_parameter("point_undistortion_distortion_coefficients")) {
      nh.declare_parameter("point_undistortion_distortion_coefficients",
          rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("point_undistortion_distortion_coefficients",
        point_undistortion_distortion_coefficients_);
    RCLCPP_INFO(logger, "Distortion Coefficients: %zu elements",
        point_undistortion_distortion_coefficients_.size());
    if (!point_undistortion_distortion_coefficients_.empty()) {
      for (const auto & value : point_undistortion_distortion_coefficients_) {
        RCLCPP_INFO(logger, "Distortion Coefficient Value: %f", value);
      }
    }

    RCLCPP_DEBUG(logger, "---> point_projection_rotation_vector");
    if (!nh.has_parameter("point_projection_rotation_vector")) {
      nh.declare_parameter("point_projection_rotation_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("point_projection_rotation_vector", point_projection_rotation_vector_);
    RCLCPP_INFO(logger, "Rotation vector: %zu elements", point_projection_rotation_vector_.size());
    if (!point_projection_rotation_vector_.empty()) {
      for (const auto & value : point_projection_rotation_vector_) {
        RCLCPP_INFO(logger, "Rotation vector Value: %f", value);
      }
    }

    RCLCPP_DEBUG(logger, "---> point_projection_translation_vector");
    if (!nh.has_parameter("point_projection_translation_vector")) {
      nh.declare_parameter("point_projection_translation_vector", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("point_projection_translation_vector", point_projection_translation_vector_);
    RCLCPP_INFO(logger, "translation vector: %zu elements",
        point_projection_translation_vector_.size());
    if (!point_projection_translation_vector_.empty()) {
      for (const auto & value : point_projection_translation_vector_) {
        RCLCPP_INFO(logger, "Translation vector  Value: %f", value);
      }
    }

    RCLCPP_DEBUG(logger, "---> point_projection_final_transformation");
    if (!nh.has_parameter("point_projection_final_transformation")) {
      nh.declare_parameter("point_projection_final_transformation", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("point_projection_final_transformation",
        point_projection_final_transformation_);
    RCLCPP_INFO(logger, "lab transformation matrix: %zu elements",
        point_projection_final_transformation_.size());
    if (!point_projection_final_transformation_.empty()) {
      for (const auto & value : point_projection_final_transformation_) {
        RCLCPP_INFO(logger, "lab transformation matrix  Value: %f", value);
      }
    }

    // --- Projection calibration X ---
    RCLCPP_DEBUG(logger, "---> projection_calibration_x");
    if (!nh.has_parameter("projection_calibration_x")) {
      nh.declare_parameter("projection_calibration_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("projection_calibration_x", projection_calibration_x_);
    RCLCPP_INFO(logger, "projection_calibration_x: %zu elements", projection_calibration_x_.size());

    // --- Projection calibration Y ---
    RCLCPP_DEBUG(logger, "---> projection_calibration_y");
    if (!nh.has_parameter("projection_calibration_y")) {
      nh.declare_parameter("projection_calibration_y", rclcpp::PARAMETER_DOUBLE_ARRAY);
    }
    nh.get_parameter("projection_calibration_y", projection_calibration_y_);
    RCLCPP_INFO(logger, "projection_calibration_y: %zu elements", projection_calibration_y_.size());

    // --- Projection image scale ---
    RCLCPP_DEBUG(logger, "---> projection_image_scale");
    nh.declare_parameter("projection_image_scale", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("projection_image_scale", projection_image_scale_);
    RCLCPP_INFO(logger, "projection_image_scale: %f", projection_image_scale_);

    // --- Projection camera X ---
    RCLCPP_DEBUG(logger, "---> projection_camera_x");
    nh.declare_parameter("projection_camera_x", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("projection_camera_x", projection_camera_x_);
    RCLCPP_INFO(logger, "projection_camera_x: %f", projection_camera_x_);

    // --- Projection camera Y ---
    RCLCPP_DEBUG(logger, "---> projection_camera_y");
    nh.declare_parameter("projection_camera_y", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("projection_camera_y", projection_camera_y_);
    RCLCPP_INFO(logger, "projection_camera_y: %f", projection_camera_y_);

    // --- Projection camera Z ---
    RCLCPP_DEBUG(logger, "---> projection_camera_z");
    nh.declare_parameter("projection_camera_z", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("projection_camera_z", projection_camera_z_);
    RCLCPP_INFO(logger, "projection_camera_z: %f", projection_camera_z_);

    // --- Projection LED Z ---
    RCLCPP_DEBUG(logger, "---> projection_led_z");
    nh.declare_parameter("projection_led_z", rclcpp::PARAMETER_DOUBLE);
    nh.get_parameter("projection_led_z", projection_led_z_);
    RCLCPP_INFO(logger, "projection_led_z: %f", projection_led_z_);

    // Validate parameters
    validateParameterSet();
  }

  void validateParameterSet()
  {
    auto logger = rclcpp::get_logger("IPS");

    if (!(number_vehicles_ > 0 && number_vehicles_ <= 256)) {
      RCLCPP_WARN(logger,
          "The number of vehicles %i is not valid, resetting to default value!",
                  number_vehicles_);
      number_vehicles_ = 20;
    }

    if (point_undistortion_camera_matrix_.size() != 9) {
      RCLCPP_WARN(logger, "Invalid camera matrix size: %zu. Expected 9 elements.",
          point_undistortion_camera_matrix_.size());
      point_undistortion_camera_matrix_ = std::vector<double>(9, 0.0);
    }

    if (point_undistortion_distortion_coefficients_.size() != 5) {
      RCLCPP_WARN(logger, "Invalid distortion coefficients size: %zu. Expected 5 elements.",
                  point_undistortion_distortion_coefficients_.size());
      point_undistortion_distortion_coefficients_ = std::vector<double>(5, 0.0);
    }

    if (point_projection_rotation_vector_.size() != 3) {
      RCLCPP_WARN(logger, "Invalid rotation vector size: %zu. Expected 3 elements.",
          point_projection_rotation_vector_.size());
      point_projection_rotation_vector_ = std::vector<double>(3, 0.0);
    }

    if (point_projection_translation_vector_.size() != 3) {
      RCLCPP_WARN(logger, "Invalid translation matrix size: %zu. Expected 3 elements.",
                  point_projection_translation_vector_.size());
      point_projection_translation_vector_ = std::vector<double>(3, 0.0);
    }

        // Validate projection calibration
    if (projection_calibration_x_.empty()) {
      RCLCPP_WARN(logger, "projection_calibration_x is empty!");
      projection_calibration_x_ = std::vector<double>(10, 0.0);   // or any expected size
    }

    if (projection_calibration_y_.empty()) {
      RCLCPP_WARN(logger, "projection_calibration_y is empty!");
      projection_calibration_y_ = std::vector<double>(10, 0.0);   // or any expected size
    }

    if (projection_image_scale_ <= 0.0) {
      RCLCPP_WARN(logger, "projection_image_scale is invalid, resetting to 1.0!");
      projection_image_scale_ = 1.0;
    }
  }
};

}  // namespace indoor_positioning_system_ros

#endif  // CPM_LAB_INDOOR_POSITIONING_SYSTEM__COMPONENT__INDOORPOSITIONINGSYSTEMPARAMETERROS_HPP_
