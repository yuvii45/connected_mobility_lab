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

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "indoor_positioning_system/LedDetection.hpp"

namespace indoor_positioning_system
{

LedDetection::LedDetection(const IndoorPositioningSystemParameter & parameters)
: min_brightness_threshold_(parameters.ledDetectionMinBrightnessThreshold()),
  max_brightness_threshold_(parameters.ledDetectionMaxBrightnessThreshold()),
  min_contour_size_(parameters.ledDetectionMinContourSize()),
  max_contour_size_(parameters.ledDetectionMaxContourSize())
{
}

std::vector<cv::Point2d> LedDetection::apply(const cv::Mat& image)
{
    std::vector<cv::Point2d> led_points;

    cv::Mat img_binary;
    cv::threshold(image, img_binary, 150, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    for (std::vector<cv::Point> contour : contours)
    {
      double size = cv::contourArea(contour);
      // RCLCPP_INFO(this->get_logger(), "size = %f", size);
      if (!(size < 60 && size > 3))
      {
        continue;
      }

      cv::Moments M = cv::moments(contour);
      float x = (M.m10 / (M.m00 + 1e-5));
      float y = (M.m01 / (M.m00 + 1e-5));
      led_points.push_back({ x, y });
      // RCLCPP_INFO(this->get_logger(), "x = %f, y = %f", x, y);
    }

    cv::namedWindow("LED Mask", cv::WINDOW_NORMAL);
    cv::resizeWindow("LED Mask", 1024, 1024);
    cv::imshow("LED Mask", img_binary);
    cv::waitKey(1);

    return led_points;
}
}