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


#ifndef INDOOR_POSITIONING_SYSTEM__LEDDETECTION_HPP_
#define INDOOR_POSITIONING_SYSTEM__LEDDETECTION_HPP_

#include "indoor_positioning_system/Definitions.hpp"
#include "indoor_positioning_system/IndoorPositioningSystemParameter.hpp"

namespace indoor_positioning_system
{

class LedDetection
{
private:
  const int min_brightness_threshold_;
  const int max_brightness_threshold_;
  const int min_contour_size_;
  const int max_contour_size_;

public:
  explicit LedDetection(const IndoorPositioningSystemParameter & parameters);
  ImagePoints apply(const cv::Mat & image);
};

}  // namespace indoor_positioning_system

#endif  // INDOOR_POSITIONING_SYSTEM__LEDDETECTION_HPP_
