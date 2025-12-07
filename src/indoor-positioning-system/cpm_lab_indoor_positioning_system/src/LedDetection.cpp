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
     // Step 1: Blur to reduce noise
     cv::Mat blurred;
     cv::GaussianBlur(image, blurred, cv::Size(3, 3), 1);

     // Step 2: Find local maxima (dilate and compare)
     cv::Mat dilated, local_max;
     cv::dilate(blurred, dilated, cv::Mat());
     cv::compare(blurred, dilated, local_max, cv::CMP_EQ);

     // Step 3: Threshold bright points
     cv::Mat thresholded;
     cv::threshold(blurred, thresholded, min_brightness_threshold_, max_brightness_threshold_, cv::THRESH_BINARY);

     // Step 4: Combine local maxima with threshold mask
     cv::Mat local_max_thresh;
     cv::bitwise_and(local_max, thresholded, local_max_thresh);

     // Step 5: Extract bright spot locations efficiently
     std::vector<cv::Point> nonzero_points;
     cv::findNonZero(local_max_thresh, nonzero_points);

     // Step 6: Convert to Point2d (subpixel not needed for LEDs)
     std::vector<cv::Point2d> led_points;
     led_points.reserve(nonzero_points.size());
     for (const auto& pt : nonzero_points)
         led_points.emplace_back(pt);

     return led_points;
}