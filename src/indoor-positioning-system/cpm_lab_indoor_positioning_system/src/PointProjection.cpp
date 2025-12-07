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

#include "indoor_positioning_system/PointProjection.hpp"

namespace indoor_positioning_system
{

PointProjection::PointProjection(const IndoorPositioningSystemParameter & parameters)
: led_height_(parameters.pointProjectionLedHeight())
{
  // --- Extrinsics ---
  Eigen::Vector3d rvec;
  for (int i = 0; i < 3; ++i) {
    rvec[i] = parameters.pointProjectionRotationVector()[i];
  }

  // Rodrigues equivalent in Eigen (rotation vector → rotation matrix)
  double theta = rvec.norm();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  if (theta > 1e-12) {
    Eigen::Vector3d axis = rvec.normalized();
    R = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
  }

  Eigen::Vector3d t;
  for (int i = 0; i < 3; ++i) {
    t[i] = parameters.pointProjectionTranslationVector()[i];
  }

  // Homography (Z=0 plane → image)
  Eigen::Matrix3d H;
  H.col(0) = R.col(0);
  H.col(1) = R.col(1);
  H.col(2) = t;

  // Store as projective transforms (nicer API than raw matrices)
  homography_ = Eigen::Projective2d(H);
  homographyinv_ = Eigen::Projective2d(H.inverse());

  // --- Final floor → lab transformation ---
  Eigen::Matrix3d F;
  const auto & final_transformation = parameters.pointProjectionFinalTransformation();
  for (int i = 0; i < 9; ++i) {
    F(i / 3, i % 3) = final_transformation[i];
  }
  final_transformation_ = Eigen::Projective2d(F);
}

FloorPoints PointProjection::apply(const ImagePoints & undistorted_points)
{
  if (undistorted_points.empty()) {
    return {};
  }

  // Reconstruct rotation matrix R from homography
  Eigen::Matrix3d R;
  R.col(0) = homography_.matrix().col(0);
  R.col(1) = homography_.matrix().col(1);
  R.col(2) = R.col(0).cross(R.col(1));  // third column orthogonal

  R.col(2).normalize();  // optional, if calibration is good

  // Translation vector
  Eigen::Vector3d t = homography_.matrix().col(2);

  // Camera center in world coordinates
  Eigen::Matrix3d R_inv = R.transpose();
  Eigen::Vector3d cam_center = -R_inv * t;

  FloorPoints floor_points;
  floor_points.reserve(undistorted_points.size());

  for (const auto & pt : undistorted_points) {
    // 1. Ray in camera coordinates (normalized)
    Eigen::Vector3d ray_cam(pt.x, pt.y, 1.0);

    // 2. Ray direction in world coordinates
    Eigen::Vector3d ray_world = R_inv * ray_cam;

    // 3. Intersect with LED plane (Z = 0.088)
    double scale = (cam_center.z() - led_height_) / ray_world.z();
    Eigen::Vector3d led_world = cam_center - scale * ray_world;

    // 4. Project vertically to floor (Z = 0)
    Eigen::Vector2d floor_world(led_world.x(), led_world.y());

    // 5. Transform to lab frame
    Eigen::Vector3d lab_pt_h = final_transformation_ * floor_world.homogeneous();
    lab_pt_h /= lab_pt_h.z();

    // 6. Store 2D result
    floor_points.emplace_back(lab_pt_h.x(), lab_pt_h.y());
  }

  return floor_points;
}


}  // namespace indoor_positioning_system
