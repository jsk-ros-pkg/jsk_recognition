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
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <algorithm>
#include <iterator>
#include <cfloat>

namespace jsk_pcl_ros
{
  void convertEigenVector(const Eigen::Vector3f& input,
                          Eigen::Vector3d& output)
  {
    output[0] = input[0];
    output[1] = input[1];
    output[2] = input[2];
  }
  void convertEigenVector(const Eigen::Vector3d& input,
                          Eigen::Vector3f& output)
  {
    output[0] = input[0];
    output[1] = input[1];
    output[2] = input[2];
  }

  
  Line::Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin)
    : direction_ (direction.normalized()), origin_(origin)
  {

  }

  void Line::getDirection(Eigen::Vector3f& output)
  {
    output = direction_;
  }

  void Line::foot(const Eigen::Vector3f& point, Eigen::Vector3f& output)
  {
    const double alpha = point.dot(direction_) - origin_.dot(direction_);
    output = alpha * direction_ + origin_;
  }

  double Line::distanceToPoint(const Eigen::Vector3f& from, Eigen::Vector3f& foot_point)
  {
    foot(from, foot_point);
    return (from - foot_point).norm();
  }
  
  double Line::distanceToPoint(const Eigen::Vector3f& from)
  {
    Eigen::Vector3f foot_point;
    return distanceToPoint(from, foot_point);
  }

  Segment::Segment(const Eigen::Vector3f& from, const Eigen::Vector3f to):
    Line(from - to, from), from_(from), to_(to)
  {
    
  }

  double Segment::dividingRatio(const Eigen::Vector3f& point)
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
  
  void Segment::foot(const Eigen::Vector3f& from, Eigen::Vector3f& output)
  {
    Eigen::Vector3f foot_point;
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
    normal_ = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
  }

  Plane::Plane(Eigen::Vector3f normal, double d) :
    normal_(normal.normalized()), d_(d / normal.norm())
  {
    
  }

  Plane::Plane(Eigen::Vector3f normal, Eigen::Vector3f p) :
    normal_(normal.normalized()), d_(- normal.dot(p) / normal.norm())
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
    return isSameDirection(another.normal_);
  }
  
  bool Plane::isSameDirection(const Eigen::Vector3f& another_normal)
  {
    return normal_.dot(another_normal) > 0;
  }
  
  double Plane::signedDistanceToPoint(const Eigen::Vector3f p)
  {
    return (normal_.dot(p) + d_);
  }
  
  double Plane::signedDistanceToPoint(const Eigen::Vector4f p)
  {
    return signedDistanceToPoint(Eigen::Vector3f(p[0], p[1], p[2]));
  }
  
  double Plane::distanceToPoint(const Eigen::Vector4f p)
  {
    return fabs(signedDistanceToPoint(p));
  }

  double Plane::distanceToPoint(const Eigen::Vector3f p)
  {
    return fabs(signedDistanceToPoint(p));
  }
  
  double Plane::distance(const Plane& another)
  {
    return fabs(fabs(d_) - fabs(another.d_));
  }

  double Plane::angle(const Plane& another)
  {
    double dot = normal_.dot(another.normal_);
    if (dot > 1.0) {
      dot = 1.0;
    }
    else if (dot < -1.0) {
      dot = -1.0;
    }
    double theta = acos(dot);
    if (theta > M_PI / 2.0) {
      return M_PI - theta;
    }

    return acos(dot);
  }

  void Plane::project(const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    // double alpha = - p.dot(normal_);
    // output = p + alpha * normal_;
    double alpha = p.dot(normal_) - d_;
    output = p - alpha * normal_;
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(Eigen::Vector3f(p[0], p[1], p[2]), output_f);
    convertEigenVector(output_f, output);
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3f& output)
  {
    project(Eigen::Vector3f(p[0], p[1], p[2]), output);
  }

  void Plane::project(const Eigen::Vector3f& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(p, output);
    convertEigenVector(output_f, output);
  }
  
  Plane Plane::transform(const Eigen::Affine3d& transform)
  {
    Eigen::Vector4d n;
    n[0] = normal_[0];
    n[1] = normal_[1];
    n[2] = normal_[2];
    n[3] = d_;
    Eigen::Matrix4d m = transform.matrix();
    Eigen::Vector4d n_d = m.transpose() * n;
    //Eigen::Vector4d n_dd = n_d.normalized();
    Eigen::Vector4d n_dd = n_d / sqrt(n_d[0] * n_d[0] + n_d[1] * n_d[1] + n_d[2] * n_d[2]);
    return Plane(Eigen::Vector3f(n_dd[0], n_dd[1], n_dd[2]), n_dd[3]);
  }
  
  std::vector<float> Plane::toCoefficients()
  {
    std::vector<float> ret;
    toCoefficients(ret);
    return ret;
  }

  void Plane::toCoefficients(std::vector<float>& output)
  {
    output.push_back(normal_[0]);
    output.push_back(normal_[1]);
    output.push_back(normal_[2]);
    output.push_back(d_);
  }

  Eigen::Vector3f Plane::getNormal()
  {
    return normal_;
  }

  double Plane::getD() 
  {
  return d_;
  }

  ConvexPolygon::ConvexPolygon(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& vertices,
                               const std::vector<float>& coefficients):
    Plane(coefficients), vertices_(vertices)
  {

  }
    
  
  ConvexPolygon::ConvexPolygon(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& vertices):
    Plane((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalized(), vertices[0]),
    vertices_(vertices)
  {

  }

  void ConvexPolygon::projectOnPlane(const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    Plane::project(p, output);
  }

  ConvexPolygon ConvexPolygon::flipConvex()
  {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >
      new_vertices;
    new_vertices.resize(vertices_.size());
    std::reverse_copy(vertices_.begin(), vertices_.end(), std::back_inserter(new_vertices));
    return ConvexPolygon(new_vertices);
  }
  
  void ConvexPolygon::project(const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    Eigen::Vector3f point_on_plane;
    Plane::project(p, point_on_plane);
    // check point_ is inside or not
    if (isInside(point_on_plane)) {
      output = point_on_plane;
    }
    else {
      // find the minimum foot point
      double min_distance = DBL_MAX;
      Eigen::Vector3f min_point;
      for (size_t i = 0; i < vertices_.size() - 1; i++) {
        Segment seg(vertices_[i], vertices_[i + 1]);
        Eigen::Vector3f foot;
        double distance = seg.distanceToPoint(p, foot);
        if (distance < min_distance) {
          min_distance = distance;
          min_point = foot;
        }
      }
      output = min_point;
    }
  }

  void ConvexPolygon::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    Eigen::Vector3f p_f(p[0], p[1], p[2]);
    project(p_f, output_f);
    convertEigenVector(output_f, output);
  }
  
  void ConvexPolygon::project(const Eigen::Vector3d& p, Eigen::Vector3f& output)
  {
    Eigen::Vector3f p_f(p[0], p[1], p[2]);
    project(p_f, output);
  }
  
  void ConvexPolygon::project(const Eigen::Vector3f& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(p, output_f);
    convertEigenVector(output_f, output);
  }
  

  bool ConvexPolygon::isInside(const Eigen::Vector3f& p)
  {
    Eigen::Vector3f A0 = vertices_[0];
    Eigen::Vector3f B0 = vertices_[0 + 1];
    Eigen::Vector3f direction0 = (B0 - A0).normalized();
    Eigen::Vector3f direction20 = (p - A0).normalized();
    bool direction_way = direction0.cross(direction20).dot(normal_) > 0;
    for (size_t i = 1; i < vertices_.size() - 1; i++) {
      Eigen::Vector3f A = vertices_[i];
      Eigen::Vector3f B = vertices_[i + 1];
      Eigen::Vector3f direction = (B - A).normalized();
      Eigen::Vector3f direction2 = (p - A).normalized();
      if (direction_way) {
        if (direction.cross(direction2).dot(normal_) >= 0) {
          continue;
        }
        else {
          return false;
        }
      }
      else {
        if (direction.cross(direction2).dot(normal_) <= 0) {
          continue;
        }
        else {
          return false;
        }
      }
    }
    return true;
  }

  Eigen::Vector3f ConvexPolygon::getCentroid()
  {
    Eigen::Vector3f ret(0, 0, 0);
    for (size_t i = 0; i < vertices_.size(); i++) {
      ret = ret + vertices_[i];
    }
    return ret / vertices_.size();
  }

  ConvexPolygon ConvexPolygon::fromROSMsg(const geometry_msgs::Polygon& polygon)
  {
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f p;
      pcl_conversions::fromMSGToEigen(polygon.points[i], p);
      vertices.push_back(p);
    }
    return ConvexPolygon(vertices);
  }
  
}
