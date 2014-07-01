// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "jsk_pcl_ros/geo_util.h"
#include <cfloat>

namespace jsk_pcl_ros
{
  Line::Line(const Eigen::Vector3d& direction, const Eigen::Vector3d& origin)
    : direction_ (direction.normalized()), origin_(origin)
  {

  }

  void Line::getDirection(Eigen::Vector3d& output)
  {
    output = direction_;
  }

  void Line::foot(const Eigen::Vector3d& point, Eigen::Vector3d& output)
  {
    const double alpha = point.dot(direction_) - origin_.dot(direction_);
    output = alpha * direction_ + origin_;
  }

  double Line::distanceToPoint(const Eigen::Vector3d& from, Eigen::Vector3d& foot_point)
  {
    foot(from, foot_point);
    return (from - foot_point).norm();
  }
  
  double Line::distanceToPoint(const Eigen::Vector3d& from)
  {
    Eigen::Vector3d foot_point;
    return distanceToPoint(from, foot_point);
  }

  Segment::Segment(const Eigen::Vector3d& from, const Eigen::Vector3d to):
    Line(from - to, from), from_(from), to_(to)
  {
    
  }

  double Segment::dividingRatio(const Eigen::Vector3d& point)
  {
    if (to_[0] != from_[0]) {
      return (point[0] - from_[0]) / (to_[0] - from_[0]);
    }
    else if (to_[1] != from_[1]) {
      return (point[1] - from_[1]) / (to_[1] - from_[1]);
    }
    else {
      return (point[2] - from_[2]) / (to_[2] - from_[2]);
    }
  }
  
  void Segment::foot(const Eigen::Vector3d& from, Eigen::Vector3d& output)
  {
    Eigen::Vector3d foot_point;
    Line::foot(from, foot_point);
    double r = dividingRatio(foot_point);
    if (r < 0.0) {
      output = from_;
    }
    else if (r > 1.0) {
      output = to_;
    }
    else {
      output = foot_point;
    }
  }
  
  Plane::Plane(const std::vector<float>& coefficients)
  {
    normal_ = Eigen::Vector3d(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
  }

  Plane::Plane(Eigen::Vector3d normal, double d) :
    normal_(normal), d_(d)
  {
    
  }

  Plane::Plane(Eigen::Vector3d normal, Eigen::Vector3d p) :
    normal_(normal), d_(- normal.dot(p))
  {
    
  }
          
  
  Plane::~Plane()
  {

  }

  Plane Plane::flip()
  {
    return Plane(- normal_, - d_);
  }

  bool Plane::isSameDirection(const Plane& another)
  {
    return normal_.dot(another.normal_) > 0;
  }

  double Plane::signedDistanceToPoint(const Eigen::Vector3d p)
  {
    return (normal_.dot(p) + d_);
  }
  
  double Plane::signedDistanceToPoint(const Eigen::Vector4f p)
  {
    return signedDistanceToPoint(Eigen::Vector3d(p[0], p[1], p[2]));
  }
  
  double Plane::distanceToPoint(const Eigen::Vector4f p)
  {
    return fabs(signedDistanceToPoint(p));
  }

  double Plane::distanceToPoint(const Eigen::Vector3d p)
  {
    return fabs(signedDistanceToPoint(p));
  }
  
  double Plane::distance(const Plane& another)
  {
    return fabs(fabs(d_) - fabs(another.d_));
  }

  double Plane::angle(const Plane& another)
  {
    return acos(normal_.dot(another.normal_));
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    double alpha = - p.dot(normal_);
    output = p + alpha * normal_;
  }

  ConvexPolygon::ConvexPolygon(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& vertices,
                               const std::vector<float>& coefficients):
    Plane(coefficients), vertices_(vertices)
  {

  }
    
  
  ConvexPolygon::ConvexPolygon(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& vertices):
    Plane((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]), vertices[0]),
    vertices_(vertices)
  {

  }

  void ConvexPolygon::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3d point_on_plane;
    Plane::project(p, point_on_plane);
    // check point_ is inside or not
    if (isInside(point_on_plane)) {
      output = point_on_plane;
    }
    else {
      // find the minimum foot point
      double min_distance = DBL_MAX;
      Eigen::Vector3d min_point;
      for (size_t i = 0; i < vertices_.size() - 1; i++) {
        Segment seg(vertices_[i], vertices_[i + 1]);
        Eigen::Vector3d foot;
        double distance = seg.distanceToPoint(p, foot);
        if (distance < min_distance) {
          min_distance = distance;
          min_point = foot;
        }
      }
      output = min_point;
    }
  }

  bool ConvexPolygon::isInside(const Eigen::Vector3d& p)
  {
    for (size_t i = 0; i < vertices_.size() - 1; i++) {
      Eigen::Vector3d A = vertices_[i];
      Eigen::Vector3d B = vertices_[i + 1];
      Eigen::Vector3d direction = (B - A).normalized();
      Eigen::Vector3d direction2 = (p - A).normalized();
      if (direction.cross(direction2).dot(normal_) > 0) {
        continue;
      }
      else {
        return false;
      }
    }
  }
  
}
