// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "jsk_pcl_ros/viewpoint_sampler.h"
#include <angles/angles.h>
#include <opencv2/opencv.hpp>

namespace jsk_pcl_ros
{
  ViewpointSampler::ViewpointSampler(
    double angle_step, double angle_min, double angle_max,
    double radius_step, double radius_min, double radius_max, int n_points):
    angle_step_(angle_step), angle_min_(angle_min), angle_max_(angle_max),
    radius_step_(radius_step), radius_min_(radius_min), radius_max_(radius_max),
    index_(0),
    angle_(angle_min),
    radius_(radius_min),
    n_points_(n_points)
  {
    
  }

  void ViewpointSampler::reset()
  {
    index_ = 0;
    angle_ = angle_min_;
    radius_ = radius_min_;
  }

  void ViewpointSampler::next()
  {
    angle_ += angle_step_;
    if (angle_ > angle_max_ * 1.001) { // 1.001 for prevent rounding error
      angle_ = angle_min_;
      radius_ += radius_step_;
      if (radius_ > radius_max_ * 1.001) { // 1.001 for prevent rounding error
        radius_ = radius_min_;
        ++index_;
      }
    }
  }

  size_t ViewpointSampler::sampleNum()
  {
    return ((angle_max_ - angle_min_) / angle_step_ + 1) * n_points_ * ((radius_max_ - radius_min_) / radius_step_ + 1);
  }

  void ViewpointSampler::normalizeVector(
    double& x, double& y, double& z)
  {
    double norm = sqrt(x*x + y*y + z*z);
    x /= norm;
    y /= norm;
    z /= norm;
  }
  
  void ViewpointSampler::get(Eigen::Affine3f& transform)
  {
    // compute the Point(x, y ,z) on the sphere based on index_ and radius_ using Golden Spiral technique
    double angle_rad = angles::from_degrees(angle_);
    const double inc = CV_PI * (3 - sqrt(5));
    const double off = 2.0f / double(n_points_);
    double y = index_ * off - 1.0f + (off / 2.0f);
    double r = sqrt(1.0f - y * y);
    double phi = index_ * inc;
    double x = std::cos(phi) * r;
    double z = std::sin(phi) * r;
    double lat = std::acos(z), lon;
    if ((fabs(std::sin(lat)) < 1e-5) || (fabs(y / std::sin(lat)) > 1))
      lon = 0;
    else
      lon = std::asin(y / std::sin(lat));
    x *= radius_; // * cos(lon) * sin(lat);
    y *= radius_; //double y = radius * sin(lon) * sin(lat);
    z *= radius_; //double z = radius * cos(lat);

    cv::Vec3d T(x, y, z);

    // Figure out the up vector
    double x_up = radius_ * std::cos(lon) * std::sin(lat - 1e-5) - x;
    double y_up = radius_ * std::sin(lon) * std::sin(lat - 1e-5) - y;
    double z_up = radius_ * std::cos(lat - 1e-5) - z;
    normalizeVector(x_up, y_up, z_up);
    
    // Figure out the third vector of the basis
    double x_right = -y_up * z + z_up * y;
    double y_right = x_up * z - z_up * x;
    double z_right = -x_up * y + y_up * x;
    normalizeVector(x_right, y_right, z_right);

    // Rotate the up vector in that basis
    double x_new_up = x_up * std::cos(angle_rad) + x_right * std::sin(angle_rad);
    double y_new_up = y_up * std::cos(angle_rad) + y_right * std::sin(angle_rad);
    double z_new_up = z_up * std::cos(angle_rad) + z_right * std::sin(angle_rad);
    cv::Vec3d up(x_new_up, y_new_up, z_new_up);

    // compute the left vector
    cv::Vec3d l;
    l = up.cross(T);  // cross product
    normalizeVector(l(0),l(1),l(2));
    
    up = T.cross(l);  // cross product
    normalizeVector(up(0), up(1), up(2));

    // compute transformation
    Eigen::Vector3f ez = Eigen::Vector3f(-T[0], -T[1], -T[2]).normalized();
    Eigen::Vector3f ey = Eigen::Vector3f(up[0], up[1], up[2]).normalized();
    Eigen::Vector3f ex = ey.cross(ez).normalized();
    Eigen::Vector3f translation = Eigen::Vector3f(T[0], T[1], T[2]);
    Eigen::Matrix3f mat;
    mat.col(0) = ex;
    mat.col(1) = ey;
    mat.col(2) = ez;
    
    Eigen::Quaternionf q(mat);
    transform = Eigen::Translation3f(translation) * Eigen::AngleAxisf(q);
  }
}
