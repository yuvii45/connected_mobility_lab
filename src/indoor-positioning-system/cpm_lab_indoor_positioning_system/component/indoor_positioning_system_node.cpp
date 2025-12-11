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

#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_observation.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <cpm_lab_lab_msgs/msg/system_trigger.hpp>

#include <indoor_positioning_system/IndoorPositioningSystem.hpp>
#include "IndoorPositioningSystemParameterRos.hpp"
#include "indoor_positioning_system_ros.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace indoor_positioning_system_ros
{
class VehicleDetectionNode : public rclcpp::Node
{
public:
  explicit VehicleDetectionNode(const rclcpp::NodeOptions & options)
  : Node("vehicle_detection_node", options)
  {
    // Read parameter from yaml file and ceate indoor positioning system
    indoor_positioning_system_parameters_.readFromRosParameterServer(*this);
    indoor_positioning_system_ =
      std::make_shared<indoor_positioning_system::IndoorPositioningSystem>(
      indoor_positioning_system_parameters_);

    // Update timings for time based triggers
    last_frame_count_time_ = now();
    time_of_last_led_request_ = now();

    initialise_camera();

    // Creating ROS2 communication channels
    system_trigger_publisher_ =
      create_publisher<cpm_lab_lab_msgs::msg::SystemTrigger>("system_trigger", 10);

    auto qos = rclcpp::QoS(10).reliability(rmw_qos_reliability_policy_from_str("best_effort"));

    for (int i = 1; i < indoor_positioning_system_parameters_.numberVehicles() + 1; i++) {
      std::string name = "vehicle_" + std::to_string(i) + "/vehicleObservation";
      auto publisher = create_publisher<cpm_lab_lab_msgs::msg::VehicleObservation>(name, qos);
      observation_publisher_.push_back(publisher);

      std::string topic_name = "vehicle_" + std::to_string(i) + "/vehicleState";
      auto subscriber = create_subscription<cpm_lab_lab_msgs::msg::VehicleState>(
          topic_name, qos, std::bind(&VehicleDetectionNode::on_vehicle_state_callback, this,
          std::placeholders::_1));
      vehicle_state_subscribers_.push_back(subscriber);
    }

    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20),
        std::bind(&VehicleDetectionNode::timer_callback, this));
  }

  ~VehicleDetectionNode()
  {
    uninitialize_camera();
  }


  void initialise_camera()
  {
    Pylon::PylonInitialize();

    try {
      Pylon::CDeviceInfo info;
      info.SetDeviceClass(Pylon::CBaslerUsbInstantCamera::DeviceClass());

      camera_ =
        std::make_unique<Pylon::CBaslerUsbInstantCamera>(
          Pylon::CTlFactory::GetInstance().CreateFirstDevice(info));
      RCLCPP_INFO(get_logger(), "Found camera %s opening..",
          camera_->GetDeviceInfo().GetModelName().c_str());

      camera_->Open();
      camera_->StaticChunkNodeMapPoolSize = camera_->MaxNumBuffer.GetValue();
      camera_->ChunkModeActive.SetValue(true);
      camera_->ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_Timestamp);
      camera_->ChunkEnable.SetValue(true);
      camera_->ChunkSelector.SetValue(Basler_UsbCameraParams::ChunkSelector_PayloadCRC16);
      camera_->ChunkEnable.SetValue(true);
      camera_->AcquisitionFrameRateEnable.SetValue(true);
      camera_->AcquisitionFrameRate.SetValue(50);
      camera_->ReverseY.SetValue(true);
      camera_->ReverseX.SetValue(true);
      camera_->PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_Mono8);
      camera_->DeviceLinkThroughputLimitMode.SetValue(
          Basler_UsbCameraParams::DeviceLinkThroughputLimitMode_Off);
      camera_->GainAuto.SetValue(Basler_UsbCameraParams::GainAuto_Off);
      camera_->Gain.SetValue(0);
      camera_->ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
      camera_->ExposureTime.SetValue(1000.0);

      int desired_width = 2048;
      int desired_height = 2048;  

      camera_->Width.SetValue(desired_width);
      camera_->Height.SetValue(desired_height);

      camera_->StartGrabbing(Pylon::EGrabStrategy::GrabStrategy_LatestImageOnly);
    } catch(const Pylon::GenericException & e) {
      RCLCPP_ERROR(get_logger(), "Error while opening camera %s", e.what());
    }
  }

  void uninitialize_camera()
  {
    Pylon::PylonTerminate();
  }

private:
  void updateFramerate(const rclcpp::Time & frame_time)
  {
    frame_count_++;

    if (frame_count_ > 100) {
      last_fps_ = 100.0 / ((frame_time - last_frame_count_time_).seconds());
      last_frame_count_time_ = frame_time;
      frame_count_ = 0;

      RCLCPP_INFO(get_logger(), "Framerate: %.1f", last_fps_);
    }
  }

  static bool inline IsVehicleIDValid(int vehicle_id)
  {
    return vehicle_id >= 1 && vehicle_id <= 20;
  }

  void on_vehicle_state_callback(const cpm_lab_lab_msgs::msg::VehicleState & state_msg)
  {
    auto state = indoor_positioning_system_ros::convertStateFromMsg(state_msg);
    indoor_positioning_system_->updateVehicleState(state);
  }

  void timer_callback()
  {
    // auto t_start = std::chrono::high_resolution_clock::now();

    // Grab image
    Pylon::CBaslerUsbGrabResultPtr ptrGrabResult;
    camera_->RetrieveResult(2, ptrGrabResult, Pylon::TimeoutHandling_Return);

    const auto frame_time = now();

    if (!ptrGrabResult || !ptrGrabResult->GrabSucceeded()) {
      RCLCPP_ERROR(get_logger(), "Image grabbing failed.");
      return;
    }

    // Get Image from buffer
    const cv::Mat image(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1,
      static_cast<uint8_t *>(ptrGrabResult->GetBuffer()));

    // Apply tracking pipeline
    indoor_positioning_system_->apply(image, frame_time.seconds());
    const auto & vehicle_observations = indoor_positioning_system_->getVehicleObservations();
    // const auto & led_points = indoor_positioning_system_->getLedPoints();

    // RCLCPP_INFO(get_logger(), "Found %zu points in image resulting in %zu observations.",
    //     led_points.size(), vehicle_observations.size());

    if (!indoor_positioning_system_->detectionSucceeded()) {
      RCLCPP_ERROR(get_logger(), "Detection failed: %s",
                   indoor_positioning_system_->getErrorMessage().c_str());
      return;
    }

    if (indoor_positioning_system_->hasWarning()) {
      RCLCPP_WARN(get_logger(), "Detection successful with warning: %s",
                   indoor_positioning_system_->getWarningMessage().c_str());
    }

    for (auto & observation : vehicle_observations) {
      auto vehicle_id = observation.vehicle_id;

      if (!IsVehicleIDValid(vehicle_id)) {
        continue;
      }

      cpm_lab_lab_msgs::msg::VehicleObservation observation_msg =
        indoor_positioning_system_ros::convertObservationToMsg(observation);
      observation_msg.header.frame_id = "map";
      observation_publisher_.at(vehicle_id - 1)->publish(observation_msg);
    }

    // Update framerate for diagnostics
    updateFramerate(frame_time);

    // Request vehicles to id if necesarry
    int requested_vehicle_id = indoor_positioning_system_->getRequestedVehicleId();
    requestLedOfVehicle(requested_vehicle_id);

    // auto t_end = std::chrono::high_resolution_clock::now();

    // double total_ms = ms(t_start, t_end);
    // RCLCPP_INFO(get_logger(), "Total timer_callback: %.2f ms", total_ms);
  }

  double ms(
    std::chrono::high_resolution_clock::time_point t1,
    std::chrono::high_resolution_clock::time_point t2) const
  {
    return std::chrono::duration<double, std::milli>(t2 - t1).count();
  }

  void requestLedOfVehicle(int vehicle_id)
  {
    if ((now() - time_of_last_led_request_).seconds() < 2.0) {
      return;
    }

    if (!IsVehicleIDValid(vehicle_id)) {
      return;
    }

    time_of_last_led_request_ = now();

    cpm_lab_lab_msgs::msg::SystemTrigger trigger_msg;
    trigger_msg.stamp = now();
    trigger_msg.source_id = "ips";
    trigger_msg.target_id = "vehicle_" + std::to_string(vehicle_id);
    trigger_msg.type = cpm_lab_lab_msgs::msg::SystemTrigger::LED_REQUEST;
    RCLCPP_INFO(get_logger(), "LED request sent to vehicle %d", vehicle_id);
    system_trigger_publisher_->publish(trigger_msg);

    indoor_positioning_system_->idRequestSendToVehicle(vehicle_id);
  }

  // rclcpp::Subscription<cpm_lab_lab_msgs::msg::LedPoints>::SharedPtr subscriber_;
  std::vector<rclcpp::Subscription<cpm_lab_lab_msgs::msg::VehicleState>::SharedPtr>
  vehicle_state_subscribers_;
  std::vector<rclcpp::Publisher<cpm_lab_lab_msgs::msg::VehicleObservation>::SharedPtr>
  observation_publisher_;
  rclcpp::Publisher<cpm_lab_lab_msgs::msg::SystemTrigger>::SharedPtr system_trigger_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Pylon::CBaslerUsbInstantCamera> camera_;

  IndoorPositioningSystemParameterRos indoor_positioning_system_parameters_;
  indoor_positioning_system::IndoorPositioningSystemPtr indoor_positioning_system_;

  rclcpp::Time time_of_last_led_request_{};

  // Frame time tracking
  rclcpp::Time last_frame_count_time_{};
  size_t frame_count_ = 0;
  float last_fps_ = 0.0;
};
// Register as a composable component
RCLCPP_COMPONENTS_REGISTER_NODE(indoor_positioning_system_ros::VehicleDetectionNode)

}  // namespace indoor_positioning_system_ros
