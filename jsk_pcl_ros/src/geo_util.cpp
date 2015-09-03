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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <algorithm>
#include <iterator>
#include <cfloat>
#include <pcl/surface/ear_clipping.h>
#include <pcl/conversions.h>
#include <boost/tuple/tuple_comparison.hpp>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <boost/foreach.hpp>
#include <boost/range/irange.hpp>
#include <boost/math/special_functions/round.hpp>
#include <jsk_topic_tools/log_utils.h>


#include <pcl/point_types.h>
#include <pcl/surface/processing.h>
#include "jsk_pcl_ros/pcl/ear_clipping_patched.h"

// #define DEBUG_GEO_UTIL
namespace jsk_pcl_ros
{
  Eigen::Quaternionf rotFrom3Axis(const Eigen::Vector3f& ex,
                                  const Eigen::Vector3f& ey,
                                  const Eigen::Vector3f& ez)
  {
    Eigen::Matrix3f rot;
    rot.col(0) = ex.normalized();
    rot.col(1) = ey.normalized();
    rot.col(2) = ez.normalized();
    return Eigen::Quaternionf(rot);
  }
  
  Line::Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin)
    : direction_ (direction.normalized()), origin_(origin)
  {

  }

  void Line::getDirection(Eigen::Vector3f& output) const
  {
    output = direction_;
  }

  Eigen::Vector3f Line::getDirection() const
  {
    return direction_;
  }
  
  void Line::getOrigin(Eigen::Vector3f& output) const
  {
    output = origin_;
  }

  void Line::foot(const Eigen::Vector3f& point, Eigen::Vector3f& output) const
  {
    const double alpha = computeAlpha(point);
    output = alpha * direction_ + origin_;
  }

  double Line::distanceToPoint(
    const Eigen::Vector3f& from, Eigen::Vector3f& foot_point) const
  {
    foot(from, foot_point);
    return (from - foot_point).norm();
  }
  
  double Line::distanceToPoint(const Eigen::Vector3f& from) const
  {
    Eigen::Vector3f foot_point;
    return distanceToPoint(from, foot_point);
  }

  double Line::angle(const Line& other) const
  {
    double dot = fabs(direction_.dot(other.direction_));
    if (dot > 1.0) {
      return M_PI / 2.0;
    }
    else {
      double theta = acos(dot);
      if (theta > M_PI / 2.0) {
        return M_PI / 2.0 - theta;
      }
      else {
        return theta;
      }
    }
  }

  bool Line::isParallel(const Line& other, double angle_threshold) const
  {
    return angle(other) < angle_threshold;
  }

  bool Line::isPerpendicular(const Line& other, double angle_threshold) const
  {
    return (M_PI / 2.0 - angle(other)) < angle_threshold;
  }

  bool Line::isSameDirection(const Line& other) const
  {
    return direction_.dot(other.direction_) > 0;
  }

  Line::Ptr Line::flip()
  {
    Line::Ptr ret (new Line(-direction_, origin_));
    return ret;
  }
  
  Line::Ptr Line::midLine(const Line& other) const
  {
    Eigen::Vector3f new_directin = (direction_ + other.direction_).normalized();
    Eigen::Vector3f new_origin;
    other.foot(origin_, new_origin);
    Line::Ptr ret (new Line(new_directin, (new_origin + origin_) / 2.0));
    return ret;
  }

  void Line::parallelLineNormal(const Line& other, Eigen::Vector3f& output)
    const
  {
    Eigen::Vector3f foot_point;
    other.foot(origin_, foot_point);
    output = origin_ - foot_point;
  }
  
  Line::Ptr Line::fromCoefficients(const std::vector<float>& coefficients)
  {
    Eigen::Vector3f p(coefficients[0],
                      coefficients[1],
                      coefficients[2]);
    Eigen::Vector3f d(coefficients[3],
                      coefficients[4],
                      coefficients[5]);
    Line::Ptr ret(new Line(d, p));
    return ret;
  }

  double Line::distance(const Line& other) const
  {
    Eigen::Vector3f v12 = (other.origin_ - origin_);
    Eigen::Vector3f n = direction_.cross(other.direction_);
    return fabs(n.dot(v12)) / n.norm();
  }

  Line::Ptr Line::parallelLineOnAPoint(const Eigen::Vector3f& p) const
  {
    Line::Ptr ret (new Line(direction_, p));
    return ret;
  }
  
  double Line::computeAlpha(const Point& p) const
  {
    return p.dot(direction_) - origin_.dot(direction_);
  }
  
  PointPair Line::findEndPoints(const Vertices& points) const
  {
    double min_alpha = DBL_MAX;
    double max_alpha = - DBL_MAX;
    Point min_alpha_point, max_alpha_point;
    for (size_t i = 0; i < points.size(); i++) {
      Point p = points[i];
      double alpha = computeAlpha(p);
      if (alpha > max_alpha) {
        max_alpha_point = p;
        max_alpha = alpha;
      }
      if (alpha < min_alpha) {
        min_alpha_point = p;
        min_alpha = alpha;
      }
    }
    // JSK_ROS_INFO("min: %f", min_alpha);
    // JSK_ROS_INFO("max: %f", max_alpha);
    return boost::make_tuple<Point, Point>(min_alpha_point, max_alpha_point);
  }

  void Line::print()
  {
    JSK_ROS_INFO("d: [%f, %f, %f], p: [%f, %f, %f]", direction_[0], direction_[1], direction_[2],
             origin_[0], origin_[1], origin_[2]);
  }

  void Line::point(double alpha, Eigen::Vector3f& output)
  {
    output = alpha * direction_ + origin_;
  }
  
  Segment::Segment(const Eigen::Vector3f& from, const Eigen::Vector3f to):
    Line(from - to, from), from_(from), to_(to)
  {
    
  }

  double Segment::dividingRatio(const Eigen::Vector3f& point) const
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
  
  void Segment::foot(const Eigen::Vector3f& from, Eigen::Vector3f& output) const
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

  double Segment::distance(const Eigen::Vector3f& point) const
  {
    Eigen::Vector3f foot_point;
    return distance(point, foot_point);
  }

  double Segment::distance(const Eigen::Vector3f& point,
                           Eigen::Vector3f& foot_point) const
  {
    foot(point, foot_point);
    return (foot_point - point).norm();
  }

  bool Segment::intersect(Plane& plane, Eigen::Vector3f& point) const
  {
    double x = - (plane.getNormal().dot(origin_) + plane.getD()) / (plane.getNormal().dot(direction_));
    point = direction_ * x + origin_;
    double r = dividingRatio(point);
    return 0 <= r && r <= 1.0;
  }

  std::ostream& operator<<(std::ostream& os, const Segment& seg)
  {
    os << "[" << seg.from_[0] << ", " << seg.from_[1] << ", " << seg.from_[2] << "] -- "
       << "[" << seg.to_[0] << ", " << seg.to_[1] << ", " << seg.to_[2] << "]";
  }
    
  Plane::Plane(const std::vector<float>& coefficients)
  {
    normal_ = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
    initializeCoordinates();
  }

  Plane::Plane(const boost::array<float, 4>& coefficients)
  {
    normal_ = Eigen::Vector3f(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
    initializeCoordinates();
  }

  Plane::Plane(Eigen::Vector3f normal, double d) :
    normal_(normal.normalized()), d_(d / normal.norm())
  {
    initializeCoordinates();
  }
  
  Plane::Plane(Eigen::Vector3f normal, Eigen::Vector3f p) :
    normal_(normal.normalized()), d_(- normal.dot(p) / normal.norm())
  {
    initializeCoordinates();
  }
          
  
  Plane::~Plane()
  {

  }
  
  Eigen::Vector3f Plane::getPointOnPlane()
  {
    Eigen::Vector3f x = normal_ / (normal_.norm() * normal_.norm()) * (- d_);
    return x;
  }

  Plane Plane::flip()
  {
    return Plane(- normal_, - d_);
  }

  Plane::Ptr Plane::faceToOrigin()
  {
    Eigen::Vector3f p = getPointOnPlane();
    Eigen::Vector3f n = getNormal();
    
    if (p.dot(n) < 0) {
      return Plane::Ptr (new Plane(normal_, d_));
    }
    else {
      return Plane::Ptr (new Plane(- normal_, - d_));
    }
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

  double Plane::angle(const Eigen::Vector3f& vector)
  {
    double dot = normal_.dot(vector);
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
    double alpha = p.dot(normal_) + d_;
    //double alpha = p.dot(normal_) - d_;
    output = p - alpha * normal_;
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(Eigen::Vector3f(p[0], p[1], p[2]), output_f);
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
  }

  void Plane::project(const Eigen::Vector3d& p, Eigen::Vector3f& output)
  {
    project(Eigen::Vector3f(p[0], p[1], p[2]), output);
  }

  void Plane::project(const Eigen::Vector3f& p, Eigen::Vector3d& output)
  {
    Eigen::Vector3f output_f;
    project(p, output);
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
  }

  void Plane::project(const Eigen::Affine3f& pose, Eigen::Affine3f& output)
  {
    Eigen::Vector3f p(pose.translation());
    Eigen::Vector3f output_p;
    project(p, output_p);
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(pose.rotation() * Eigen::Vector3f::UnitZ(),
                          coordinates().rotation() * Eigen::Vector3f::UnitZ());
    output = Eigen::Affine3f::Identity() * Eigen::Translation3f(output_p) * rot;
  }

  Plane Plane::transform(const Eigen::Affine3f& transform)
  {
    Eigen::Affine3d transform_d;
    convertEigenAffine3(transform, transform_d);
    return this->transform(transform_d);
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

  void Plane::initializeCoordinates()
  {
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal_);
    double c = normal_[2];
    double z = 0.0;
    // ax + by + cz + d = 0
    // z = - d / c (when x = y = 0)
    if (c == 0.0) {             // its not good
      z = 0.0;
    }
    else {
      z = - d_ / c;
    }
    plane_coordinates_
      = Eigen::Affine3f::Identity() * Eigen::Translation3f(0, 0, z) * rot;
  }
  
  Eigen::Affine3f Plane::coordinates()
  {
    return plane_coordinates_;
  }
  
  Polygon Polygon::createPolygonWithSkip(const Vertices& vertices)
  {
    const double thr = 0.01;
    Polygon not_skipped_polygon(vertices);
    Vertices skipped_vertices;
    for (size_t i = 0; i < vertices.size(); i++) {
      size_t next_i = not_skipped_polygon.nextIndex(i);
      Eigen::Vector3f v0 = vertices[i];
      Eigen::Vector3f v1 = vertices[next_i];
      if ((v1 - v0).norm() > thr) {
        skipped_vertices.push_back(vertices[i]);
      }
    }
    return Polygon(skipped_vertices);
  }

  Eigen::Vector3f Polygon::centroid()
  {
    Eigen::Vector3f c(0, 0, 0);
    if (vertices_.size() == 0) {
      return c;
    }
    else {
      for (size_t i = 0; i < vertices_.size(); i++) {
        c = c + vertices_[i];
      }
      return c / vertices_.size();
    }
  }

  std::vector<Plane::Ptr> convertToPlanes(
    std::vector<pcl::ModelCoefficients::Ptr> coefficients)
  {
    std::vector<Plane::Ptr> ret;
    for (size_t i = 0; i < coefficients.size(); i++) {
      ret.push_back(Plane::Ptr (new Plane(coefficients[i]->values)));
    }
    return ret;
  }
  
  
  Polygon::Polygon(const Vertices& vertices):
    Plane((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalized(), vertices[0]),
    vertices_(vertices)
  {
    
  }

  Polygon::Polygon(const Vertices& vertices,
                   const std::vector<float>& coefficients):
    Plane(coefficients), vertices_(vertices)
  {
    
  }
  
  Polygon::~Polygon()
  {

  }

  size_t Polygon::getFarestPointIndex(const Eigen::Vector3f& O)
  {
    double max_distance = - DBL_MAX;
    size_t max_index = 0;
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f v = vertices_[i];
      double d = (O - v).norm();
      if (max_distance < d) {
        max_distance = d;
        max_index = i;
      }
    }
    return max_index;
  }

  PointIndexPair Polygon::getNeighborIndex(size_t index)
  {
    return boost::make_tuple<size_t, size_t>(
      previousIndex(index), nextIndex(index));
  }

  double Polygon::area()
  {
    if (isTriangle()) {
      return (vertices_[1] - vertices_[0]).cross(vertices_[2] - vertices_[0]).norm() / 2.0;
    }
    else {
      std::vector<Polygon::Ptr> triangles = decomposeToTriangles();
      double sum = 0;
      for (size_t i = 0; i < triangles.size(); i++) {
        sum += triangles[i]->area();
      }
      return sum;
    }
  }
  
  Eigen::Vector3f Polygon::directionAtPoint(size_t i)
  {
    Eigen::Vector3f O = vertices_[i];
    Eigen::Vector3f A = vertices_[previousIndex(i)];
    Eigen::Vector3f B = vertices_[nextIndex(i)];
    Eigen::Vector3f OA = A - O;
    Eigen::Vector3f OB = B - O;
    Eigen::Vector3f n = (OA.normalized()).cross(OB.normalized());
    if (n.norm() == 0) {
      // JSK_ROS_ERROR("normal is 0");
      // JSK_ROS_ERROR("O: [%f, %f, %f]", O[0], O[1], O[2]);
      // JSK_ROS_ERROR("A: [%f, %f, %f]", A[0], A[1], A[2]);
      // JSK_ROS_ERROR("B: [%f, %f, %f]", B[0], B[1], B[2]);
      // JSK_ROS_ERROR("OA: [%f, %f, %f]", OA[0], OA[1], OA[2]);
      // JSK_ROS_ERROR("OB: [%f, %f, %f]", OB[0], OB[1], OB[2]);
      //exit(1);
    }
    return n.normalized();
  }
  
  bool Polygon::isTriangle() {
    return vertices_.size() == 3;
  }

  void Polygon::getLocalMinMax(double& min_x, double& min_y,
                               double& max_x, double& max_y)
  {
    min_x = DBL_MAX;
    min_y = DBL_MAX;
    max_x = - DBL_MAX;
    max_y = - DBL_MAX;
    
    Eigen::Affine3f inv_coords = coordinates().inverse();
    for (size_t i = 0; i < vertices_.size(); i++) {
      // Convert vertices into local coordinates
      Eigen::Vector3f local_point = inv_coords * vertices_[i];
      min_x = ::fmin(local_point[0], min_x);
      min_y = ::fmin(local_point[1], min_y);
      max_x = ::fmax(local_point[0], max_x);
      max_y = ::fmax(local_point[1], max_y);
    }
  }
  
  Eigen::Vector3f Polygon::randomSampleLocalPoint(boost::mt19937& random_generator)
  {
    // Compute min/max point
    double min_x, min_y, max_x, max_y;
    getLocalMinMax(min_x, min_y, max_x, max_y);
    std::vector<Polygon::Ptr> triangles = decomposeToTriangles();
    while (true) {
      double x = randomUniform(min_x, max_x, random_generator);
      double y = randomUniform(min_y, max_y, random_generator);
      Eigen::Vector3f local_v = Eigen::Vector3f(x, y, 0);
      Eigen::Vector3f v = coordinates() * local_v;
      // ROS_INFO_THROTTLE(1.0, "x: %f -- %f", min_x, max_x);
      // ROS_INFO_THROTTLE(1.0, "y: %f -- %f", min_y, max_y);
      // ROS_INFO_THROTTLE(1.0, "sampled point: [%f, %f]", x, y);
      // for (size_t i = 0; i < vertices_.size(); i++) {
      //   Eigen::Vector3f v = coordinates().inverse() * vertices_[i];
      //   ROS_INFO("v: [%f, %f, %f]", v[0], v[1], v[2]);
      // }
      if (isInside(v)) {
        return local_v;
      }
      else {
        // ROS_INFO_THROTTLE(1.0, "outside");
      }
    }
  }

  std::vector<Segment::Ptr> Polygon::edges() const
  {
    std::vector<Segment::Ptr> ret;
    ret.reserve(vertices_.size());
    for (size_t i = 0; i < vertices_.size() - 1; i++) {
      // edge between i and i+1
      ret.push_back(Segment::Ptr(new Segment(vertices_[i], vertices_[i+1])));
    }
    // edge between [-1] and [0]
    ret.push_back(Segment::Ptr(new Segment(vertices_[vertices_.size() - 1], vertices_[0])));
    return ret;
  }
  
  Eigen::Vector3f Polygon::nearestPoint(const Eigen::Vector3f& p,
                                        double& distance)
  {
    Eigen::Vector3f projected_p;
    Plane::project(p, projected_p);
    if (isInside(projected_p)) {
      distance = (p - projected_p).norm();
      return projected_p;
    }
    else {
      std::vector<Segment::Ptr> boundary_edges = edges();
      double min_distnace = DBL_MAX;
      Eigen::Vector3f nearest_point;
      // brute-force searching
      for (size_t i = 0; i < boundary_edges.size(); i++) {
        Segment::Ptr edge = boundary_edges[i];
        Eigen::Vector3f foot;
        double d = edge->distance(p, foot);
        if (min_distnace > d) {
          nearest_point = foot;
          min_distnace = d;
        }
      }
      distance = min_distnace;
      return nearest_point;
    }
  }
  
  size_t Polygon::getNumVertices() {
    return vertices_.size();
  }
  
  Eigen::Vector3f Polygon::getVertex(size_t i) {
    return vertices_[i];
  }
  
  Polygon::PtrPair Polygon::separatePolygon(size_t index)
  {
    PointIndexPair neighbor_index = getNeighborIndex(index);
    Vertices triangle_vertices;
    triangle_vertices.push_back(vertices_[index]);
    triangle_vertices.push_back(vertices_[neighbor_index.get<1>()]);
    triangle_vertices.push_back(vertices_[neighbor_index.get<0>()]);
    Polygon::Ptr triangle(new Polygon(triangle_vertices));
    Vertices rest_vertices;
    // do not add the points on the line
    for (size_t i = neighbor_index.get<1>(); i != index;) {
      // check the points on the line
      if (i == neighbor_index.get<1>()) {
        rest_vertices.push_back(vertices_[i]);
      }
      else {
        if (directionAtPoint(i).norm() != 0.0) {
          rest_vertices.push_back(vertices_[i]);
        }
        else {
          JSK_ROS_ERROR("removed: %lu", i);
        }
      }
      i = nextIndex(i);
    }
    Polygon::Ptr rest(new Polygon(rest_vertices));
    return boost::make_tuple<Polygon::Ptr, Polygon::Ptr>(
      triangle, rest);
  }
  
  bool Polygon::isPossibleToRemoveTriangleAtIndex(
    size_t index,
    const Eigen::Vector3f& direction)
  {
    Polygon::PtrPair candidate = separatePolygon(index);
    Polygon::Ptr triangle_candidate = candidate.get<0>();
    Polygon::Ptr rest_candidate = candidate.get<1>();
    // first check direction
    Eigen::Vector3f the_direction = directionAtPoint(index);
    //JSK_ROS_INFO("direction: [%f, %f, %f]", the_direction[0], the_direction[1], the_direction[2]);
    if (the_direction.norm() == 0.0) {
      JSK_ROS_ERROR("malformed polygon");
      exit(1);
    }
    if (direction.dot(the_direction) < 0) {
#ifdef DEBUG_GEO_UTIL
      JSK_ROS_INFO("triangle is not same direction");
      JSK_ROS_INFO("direction: [%f, %f, %f]", direction[0], direction[1], direction[2]);
      JSK_ROS_INFO("the_direction: [%f, %f, %f]",
               the_direction[0],
               the_direction[1],
               the_direction[2]);
      for (size_t i = 0; i < vertices_.size(); i++) {
        Eigen::Vector3f v = directionAtPoint(i);
        JSK_ROS_INFO("the_direction[%lu]: [%f, %f, %f]",
                 i, v[0], v[1], v[2]);
      // other direction
      }
#endif
      return false;
    }
    else {
      //return true;
      // second, check the triangle includes the rest of points or not
      for (size_t i = 0; i < rest_candidate->vertices_.size(); i++) {
        if (i == 0 || i == rest_candidate->vertices_.size() - 1) {
          continue;       // do not check the first and the last point
        }
        else {
          Eigen::Vector3f P = rest_candidate->getVertex(i);
          Eigen::Vector3f A = triangle_candidate->getVertex(0);
          Eigen::Vector3f B = triangle_candidate->getVertex(1);
          Eigen::Vector3f C = triangle_candidate->getVertex(2);
          Eigen::Vector3f CA = A - C;
          Eigen::Vector3f BC = C - B;
          Eigen::Vector3f AB = B - A;
          Eigen::Vector3f AP = P - A;
          Eigen::Vector3f BP = P - B;
          Eigen::Vector3f CP = P - C;
          Eigen::Vector3f Across = CA.normalized().cross(AP.normalized()).normalized();
          Eigen::Vector3f Bcross = AB.normalized().cross(BP.normalized()).normalized();
          Eigen::Vector3f Ccross = BC.normalized().cross(CP.normalized()).normalized();
#ifdef DEBUG_GEO_UTIL
          JSK_ROS_INFO("P: [%f, %f, %f]", P[0], P[1], P[2]);
          JSK_ROS_INFO("A: [%f, %f, %f]", A[0], A[1], A[2]);
          JSK_ROS_INFO("B: [%f, %f, %f]", B[0], B[1], B[2]);
          JSK_ROS_INFO("C: [%f, %f, %f]", C[0], C[1], C[2]);
          JSK_ROS_INFO("Across: [%f, %f, %f]", Across[0], Across[1], Across[2]);
          JSK_ROS_INFO("Bcross: [%f, %f, %f]", Bcross[0], Bcross[1], Bcross[2]);
          JSK_ROS_INFO("Ccross: [%f, %f, %f]", Ccross[0], Ccross[1], Ccross[2]);
          JSK_ROS_INFO("Across-Bcross: %f", Across.dot(Bcross));
          JSK_ROS_INFO("Bcross-Ccross: %f", Bcross.dot(Ccross));
          JSK_ROS_INFO("Ccross-Across: %f", Ccross.dot(Across));
#endif
          if (((Across.dot(Bcross) > 0 &&
                Bcross.dot(Ccross) > 0 &&
                Ccross.dot(Across) > 0) ||
               (Across.dot(Bcross) < 0 &&
                Bcross.dot(Ccross) < 0 &&
                Ccross.dot(Across) < 0))) {
            // JSK_ROS_ERROR("%lu -- %lu is inside", index, i);
            return false;
          }
          // ConvexPolygon convex_triangle(triangle_candidate->vertices_);
          // if (convex_triangle.isInside(v)) {
          //   //JSK_ROS_INFO("vertices is inside of the polygon");
          //   return false;
          // }
        }
      }
      return true;
    }
  }

  bool Polygon::isConvex()
  {
#ifdef DEBUG_GEO_UTIL
    for (size_t i = 0; i < getNumVertices(); i++) {
      Eigen::Vector3f n = directionAtPoint(i);
      JSK_ROS_INFO("n[%lu] [%f, %f, %f]", i, n[0], n[1], n[2]);
    }
#endif
    Eigen::Vector3f n0 = directionAtPoint(0);
    for (size_t i = 1; i < getNumVertices(); i++) {
      Eigen::Vector3f n = directionAtPoint(i);
      if (n0.dot(n) < 0) {
        return false;
      }
    }
    return true;
  }
  
  std::vector<Polygon::Ptr> Polygon::decomposeToTriangles()
  {
    if (cached_triangles_.size() != 0) {
      return cached_triangles_;
    }
    std::vector<Polygon::Ptr> ret;

    // if this polygon is triangle, return immediately
    if (isTriangle()) {
      ret.push_back(Polygon::Ptr( new Polygon(*this)));
      return ret;
    }

    pcl::EarClippingPatched clip;
    // convert
    pcl::PolygonMesh::Ptr input_mesh (new pcl::PolygonMesh);
    pcl::PCLPointCloud2 mesh_cloud;
    pcl::PointCloud<pcl::PointXYZ> mesh_pcl_cloud;
    boundariesToPointCloud<pcl::PointXYZ>(mesh_pcl_cloud);
    std::vector<pcl::Vertices> mesh_vertices(1);
    for (size_t i = 0; i < vertices_.size(); i++) {
      mesh_vertices[0].vertices.push_back(i);
    }
    //mesh_vertices[0].vertices.push_back(0); // close
    mesh_pcl_cloud.height = 1;
    mesh_pcl_cloud.width = mesh_pcl_cloud.points.size();
    pcl::toPCLPointCloud2<pcl::PointXYZ>(mesh_pcl_cloud, mesh_cloud);

    input_mesh->polygons = mesh_vertices;
    input_mesh->cloud = mesh_cloud;
    clip.setInputMesh(input_mesh);
    pcl::PolygonMesh output;
    clip.process(output);
    assert(output.polygons.size() != 0);
    // convert to Polygon instances
    for (size_t i = 0; i < output.polygons.size(); i++) {
      pcl::Vertices output_polygon_vertices = output.polygons[i];
      Vertices vs(output_polygon_vertices.vertices.size());
      for (size_t j = 0; j < output_polygon_vertices.vertices.size(); j++) {
        pcl::PointXYZ p
          = mesh_pcl_cloud.points[output_polygon_vertices.vertices[j]];
        Eigen::Vector3f v;
        pointFromXYZToVector<pcl::PointXYZ, Eigen::Vector3f>(p, v);
        vs[j] = v;
      }
      ret.push_back(Polygon::Ptr(new Polygon(vs, toCoefficients())));
    }
    cached_triangles_ = ret;
    return ret;
  }

  Eigen::Vector3f Polygon::getNormalFromVertices()
  {
    if (vertices_.size() >= 3) {
      return (vertices_[1] - vertices_[0]).cross(vertices_[2] - vertices_[0]).normalized();
    }
    else {
      JSK_ROS_ERROR("the number of vertices is not enough");
      return Eigen::Vector3f(0, 0, 0);
    }
  }

  size_t Polygon::previousIndex(size_t i)
  {
    if (i == 0) {
      return vertices_.size() - 1;
    }
    else {
      return i - 1;
    }
  }
  
  size_t Polygon::nextIndex(size_t i)
  {
    if (i == vertices_.size() - 1) {
      return 0;
    }
    else {
      return i + 1;
    }
  }

  Polygon Polygon::fromROSMsg(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f v;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], v);
      vertices.push_back(v);
    }
    return Polygon(vertices);
  }

  Polygon::Ptr Polygon::fromROSMsgPtr(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f v;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], v);
      vertices.push_back(v);
    }
    return Polygon::Ptr(new Polygon(vertices));
  }
  
  bool Polygon::isInside(const Eigen::Vector3f& p)
  {
    if (isTriangle()) {
      Eigen::Vector3f A = vertices_[0];
      Eigen::Vector3f B = vertices_[1];
      Eigen::Vector3f C = vertices_[2];
      // Eigen::Vector3f cross0 = (A - C).cross(p - A);
      // Eigen::Vector3f cross1 = (B - A).cross(p - B);
      // Eigen::Vector3f cross2 = (C - B).cross(p - C);
      
      Eigen::Vector3f cross0 = (B - A).cross(p - A);
      Eigen::Vector3f cross1 = (C - B).cross(p - B);
      Eigen::Vector3f cross2 = (A - C).cross(p - C);
      if (cross0.dot(cross1) >= 0 &&
          cross1.dot(cross2) >= 0) {
        return true;
      }
      else {
        return false;
      }
    }
    else {
      std::vector<Polygon::Ptr> triangles = decomposeToTriangles();
      for (size_t i = 0; i < triangles.size(); i++) {
        if (triangles[i]->isInside(p)) {
          return true;
        }
      }
      return false;
    }
  }

  ConvexPolygon::ConvexPolygon(const Vertices& vertices):
    Polygon(vertices)
  {

  }

  ConvexPolygon::ConvexPolygon(const Vertices& vertices,
                               const std::vector<float>& coefficients):
    Polygon(vertices, coefficients)
  {

  }
  
  void ConvexPolygon::projectOnPlane(
    const Eigen::Vector3f& p, Eigen::Vector3f& output)
  {
    Plane::project(p, output);
  }

  void ConvexPolygon::projectOnPlane(
    const Eigen::Affine3f& pose, Eigen::Affine3f& output)
  {
    Eigen::Vector3f p(pose.translation());
    Eigen::Vector3f output_p;
    projectOnPlane(p, output_p);
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] p: [%f, %f, %f]",
    //          p[0], p[1], p[2]);
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] output_p: [%f, %f, %f]",
    //          output_p[0], output_p[1], output_p[2]);
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(pose.rotation() * Eigen::Vector3f::UnitZ(),
                          coordinates().rotation() * Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf coords_rot(coordinates().rotation());
    Eigen::Quaternionf pose_rot(pose.rotation());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] rot: [%f, %f, %f, %f]",
    //          rot.x(), rot.y(), rot.z(), rot.w());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] coords_rot: [%f, %f, %f, %f]",
    //          coords_rot.x(), coords_rot.y(), coords_rot.z(), coords_rot.w());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] pose_rot: [%f, %f, %f, %f]",
    //          pose_rot.x(), pose_rot.y(), pose_rot.z(), pose_rot.w());
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] normal: [%f, %f, %f]", normal_[0], normal_[1], normal_[2]);
    // Eigen::Affine3f::Identity() *
    // output.translation() = Eigen::Translation3f(output_p);
    // output.rotation() = rot * pose.rotation();
    //output = Eigen::Translation3f(output_p) * rot * pose.rotation();
    output = Eigen::Affine3f(rot * pose.rotation());
    output.pretranslate(output_p);
    // Eigen::Vector3f projected_point = output * Eigen::Vector3f(0, 0, 0);
    // JSK_ROS_INFO("[ConvexPolygon::projectOnPlane] output: [%f, %f, %f]",
    //          projected_point[0], projected_point[1], projected_point[2]);
  }

  ConvexPolygon ConvexPolygon::flipConvex()
  {
    Vertices new_vertices;
    std::reverse_copy(vertices_.begin(), vertices_.end(),
                      std::back_inserter(new_vertices));
    std::vector<float> reversed_coefficients(4);
    reversed_coefficients[0] = - normal_[0];
    reversed_coefficients[1] = - normal_[1];
    reversed_coefficients[2] = - normal_[2];
    reversed_coefficients[3] = - d_;
    
    return ConvexPolygon(new_vertices, reversed_coefficients);
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
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
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
    pointFromVectorToVector<Eigen::Vector3f, Eigen::Vector3d>(output_f, output);
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
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f p;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], p);
      vertices.push_back(p);
    }
    return ConvexPolygon(vertices);
  }
  
  ConvexPolygon::Ptr ConvexPolygon::fromROSMsgPtr(const geometry_msgs::Polygon& polygon)
  {
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f p;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], p);
      vertices.push_back(p);
    }
    return ConvexPolygon::Ptr(new ConvexPolygon(vertices));
  }

  bool ConvexPolygon::distanceSmallerThan(const Eigen::Vector3f& p,
                                          double distance_threshold)
  {
    double dummy_distance;
    return distanceSmallerThan(p, distance_threshold, dummy_distance);
  }
  
  bool ConvexPolygon::distanceSmallerThan(const Eigen::Vector3f& p,
                                          double distance_threshold,
                                          double& output_distance)
  {
    // first check distance as Plane rather than Convex
    double plane_distance = distanceToPoint(p);
    if (plane_distance > distance_threshold) {
      output_distance = plane_distance;
      return false;
    }

    Eigen::Vector3f foot_point;
    project(p, foot_point);
    double convex_distance = (p - foot_point).norm();
    output_distance = convex_distance;
    return convex_distance < distance_threshold;
  }

  bool ConvexPolygon::allEdgesLongerThan(double thr)
  {
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f p_k = vertices_[i];
      Eigen::Vector3f p_k_1;
      if (i == vertices_.size() - 1) {
        p_k_1 = vertices_[0];
      }
      else {
        p_k_1 = vertices_[i + 1];
      }
      if ((p_k - p_k_1).norm() < thr) {
        return false;
      }
    }
    return true;
  }

  double ConvexPolygon::distanceFromVertices(const Eigen::Vector3f& p)
  {
    double min_distance = DBL_MAX;
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f v = vertices_[i];
      double d = (p - v).norm();
      if (d < min_distance) {
        min_distance = d;
      }
    }
    return min_distance;
  }
  
  ConvexPolygon::Ptr ConvexPolygon::magnifyByDistance(const double distance)
  {
    // compute centroid
    Eigen::Vector3f c = centroid();
    Vertices new_vertices(vertices_.size());
    for (size_t i = 0; i < vertices_.size(); i++) {
      new_vertices[i] = (vertices_[i] - c).normalized() * distance + vertices_[i];
      // JSK_ROS_INFO("old v: [%f, %f, %f]", vertices_[i][0], vertices_[i][1], vertices_[i][2]);
      // JSK_ROS_INFO("new v: [%f, %f, %f]", new_vertices[i][0], new_vertices[i][1], new_vertices[i][2]);
      // JSK_ROS_INFO("");
    }
    
    ConvexPolygon::Ptr ret (new ConvexPolygon(new_vertices));
    return ret;
  }
  
  ConvexPolygon::Ptr ConvexPolygon::magnify(const double scale_factor)
  {
    // compute centroid
    Eigen::Vector3f c = centroid();
    Vertices new_vertices;
    for (size_t i = 0; i < vertices_.size(); i++) {
      new_vertices.push_back((vertices_[i] - c) * scale_factor + c);
    }
    ConvexPolygon::Ptr ret (new ConvexPolygon(new_vertices));
    return ret;
  }

  geometry_msgs::Polygon ConvexPolygon::toROSMsg()
  {
    geometry_msgs::Polygon polygon;
    for (size_t i = 0; i < vertices_.size(); i++) {
      geometry_msgs::Point32 ros_point;
      ros_point.x = vertices_[i][0];
      ros_point.y = vertices_[i][1];
      ros_point.z = vertices_[i][2];
      polygon.points.push_back(ros_point);
    }
    return polygon;
  }


  bool ConvexPolygon::isProjectableInside(const Eigen::Vector3f& p)
  {
    Eigen::Vector3f foot_point;
    Plane::project(p, foot_point);
    return isInside(foot_point);
  }

  GridPlane::GridPlane(ConvexPolygon::Ptr plane, const double resolution):
    convex_(plane), resolution_(resolution)
  {

  }

  GridPlane::~GridPlane()
  {

  }

  GridPlane::IndexPair GridPlane::projectLocalPointAsIndexPair(
    const Eigen::Vector3f& p)
  {
    double offset_x = p[0] + 0.5 * resolution_;
    double offset_y = p[1] + 0.5 * resolution_;
    // return boost::make_tuple<int, int>(std::floor(offset_x / resolution_),
    //                                    std::floor(offset_y / resolution_));
    return boost::make_tuple<int, int>(boost::math::round(p[0] / resolution_),
                                       boost::math::round(p[1] / resolution_));
  }

  void GridPlane::addIndexPair(IndexPair pair)
  {
    cells_.insert(pair);
  }

  GridPlane::Ptr GridPlane::dilate(int num)
  {
    GridPlane::Ptr ret (new GridPlane(convex_, resolution_));
    for (std::set<IndexPair>::iterator it = cells_.begin();
         it != cells_.end();
         ++it) {
      IndexPair the_index = *it;
      for (int xi = - num; xi <= num; xi++) {
        for (int yi = - num; yi <= num; yi++) {
          if (abs(xi) + abs(yi) <= num) {
            IndexPair new_pair = boost::make_tuple<int, int>(
              the_index.get<0>() + xi,
              the_index.get<1>() + yi);
            ret->cells_.insert(new_pair);
          }
        }
      }
    }
    return ret;
  }

  GridPlane::Ptr GridPlane::erode(int num)
  {
    GridPlane::Ptr ret (new GridPlane(convex_, resolution_));
    for (std::set<IndexPair>::iterator it = cells_.begin();
         it != cells_.end();
         ++it) {
      IndexPair the_index = *it;
      bool should_removed = false;
      for (int xi = - num; xi <= num; xi++) {
        for (int yi = - num; yi <= num; yi++) {
          if (abs(xi) + abs(yi) <= num) {
            IndexPair check_pair = boost::make_tuple<int, int>(
              the_index.get<0>() + xi,
              the_index.get<1>() + yi);
            if (!isOccupied(check_pair)) {
              should_removed = true;
            }
          }
        }
      }
      if (!should_removed) {
        ret->cells_.insert(the_index);
      }
    }
    return ret;
  }

  bool GridPlane::isOccupied(const IndexPair& pair)
  {
    bool result = cells_.find(pair) != cells_.end();
    // Verbosing for debug
    // JSK_ROS_INFO("Checking index pair (%d, %d)", pair.get<0>(), pair.get<1>());
    // JSK_ROS_INFO("Result: %d", result);
    // JSK_ROS_INFO("cells are:");
    // for (IndexPairSet::iterator it = cells_.begin();
    //      it != cells_.end();
    //      ++it) {
    //   JSK_ROS_INFO("  (%d, %d)", it->get<0>(), it->get<1>());
    // }
    return result;
  }

  bool GridPlane::isOccupied(const Eigen::Vector3f& p)
  {
    IndexPair pair = projectLocalPointAsIndexPair(p);
    return isOccupied(pair);
  }

  bool GridPlane::isOccupiedGlobal(const Eigen::Vector3f& p)
  {
    return isOccupied(convex_->coordinates().inverse() * p);
  }

  GridPlane::Ptr GridPlane::clone()
  {
    GridPlane::Ptr ret (new GridPlane(convex_, resolution_));
    ret->cells_ = cells_;
    return ret;
  }
  
  Eigen::Vector3f GridPlane::unprojectIndexPairAsLocalPoint(
    const IndexPair& pair)
  {
    return Eigen::Vector3f(pair.get<0>() * resolution_,
                           pair.get<1>() * resolution_,
                           0);
  }

  Eigen::Vector3f GridPlane::unprojectIndexPairAsGlobalPoint(
    const IndexPair& pair)
  {
    Eigen::Vector3f local_point = unprojectIndexPairAsLocalPoint(pair);
    return convex_->coordinates() * local_point;
  }

  void GridPlane::fillCellsFromCube(Cube& cube)
  {
    ConvexPolygon::Ptr intersect_polygon = cube.intersectConvexPolygon(*convex_);
    // 1. transform vertices into local coordinates
    // 2. compute min-max
    // 3. compute candidates
    // 4. filter candidates

    // 1. transform vertices into local coordinates
    Vertices local_vertices;
    Vertices global_vertices = intersect_polygon->getVertices();
    Eigen::Affine3f inv_coords = convex_->coordinates().inverse();
    for (size_t i = 0; i < global_vertices.size(); i++) {
      local_vertices.push_back(inv_coords * global_vertices[i]);
    }
    
    // 2. compute min-max
    double min_x = DBL_MAX;
    double min_y = DBL_MAX;
    double max_x = - DBL_MAX;
    double max_y = - DBL_MAX;
    for (size_t i = 0; i < local_vertices.size(); i++) {
      min_x = ::fmin(min_x, local_vertices[i][0]);
      min_y = ::fmin(min_y, local_vertices[i][1]);
      max_x = ::fmax(max_x, local_vertices[i][0]);
      max_y = ::fmax(max_y, local_vertices[i][1]);
    }
    // JSK_ROS_INFO("x: [%f~%f]", min_x, max_x);
    // JSK_ROS_INFO("y: [%f~%f]", min_y, max_y);
    // 3. compute candidates
    std::vector<Polygon::Ptr> triangles
      = intersect_polygon->decomposeToTriangles();
    for (double x = min_x; x <= max_x; x += resolution_) {
      for (double y = min_y; y <= max_y; y += resolution_) {
        Eigen::Vector3f local_p(x, y, 0);
        Eigen::Vector3f p = convex_->coordinates() * local_p;
        // 4. filter candidates
        bool insidep = false;
        for (size_t i = 0; i < triangles.size(); i++) {
          if (triangles[i]->isInside(p)) {
            insidep = true;
            break;
          }
        }
        if (insidep) {
          IndexPair pair = projectLocalPointAsIndexPair(local_p);
          addIndexPair(pair);
        }
      }
    }
  }

  size_t GridPlane::fillCellsFromPointCloud(
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    double distance_threshold,
    std::set<int>& non_plane_indices)
  {
    return fillCellsFromPointCloud(
      cloud, distance_threshold, M_PI / 2.0, non_plane_indices);
  }
  
  size_t GridPlane::fillCellsFromPointCloud(
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    double distance_threshold,
    double normal_threshold,
    std::set<int>& non_plane_indices)
  {
    Eigen::Affine3f local_coordinates = convex_->coordinates();
    Eigen::Affine3f inv_local_coordinates = local_coordinates.inverse();
    //std::vector<Polygon::Ptr> triangles = convex_->decomposeToTriangles();
    
    pcl::ExtractPolygonalPrismData<pcl::PointNormal> prism_extract;
    pcl::PointCloud<pcl::PointNormal>::Ptr
      hull_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      hull_output (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      rehull_cloud (new pcl::PointCloud<pcl::PointNormal>);
    convex_->boundariesToPointCloud<pcl::PointNormal>(*hull_cloud);
    // pcl::ConvexHull<pcl::PointNormal> chull;
    // chull.setDimension(2);
    // chull.setInputCloud (hull_cloud);
    // chull.reconstruct(*hull_output);
    
    // it's important to make it sure to close the loop of
    // convex hull
    hull_cloud->points.push_back(hull_cloud->points[0]);
    
    prism_extract.setInputCloud(cloud);
    prism_extract.setHeightLimits(-distance_threshold, distance_threshold);
    prism_extract.setInputPlanarHull(hull_cloud);
    //prism_extract.setInputPlanarHull(hull_output);
    // output_indices is set of indices which are on plane
    pcl::PointIndices output_indices;
    prism_extract.segment(output_indices);
    std::set<int> output_set(output_indices.indices.begin(),
                             output_indices.indices.end());
    Eigen::Vector3f n = convex_->getNormal();
    for (size_t i = 0; i < cloud->points.size(); i++) {
      if (output_set.find(i) != output_set.end()) {
        // check normal
        pcl::PointNormal p = cloud->points[i];
        Eigen::Vector3f n_p = p.getNormalVector3fMap();
        if (std::abs(n.dot(n_p)) > cos(normal_threshold)) {
          non_plane_indices.insert(i);
        }
      }
    }

    
    for (size_t i = 0; i < output_indices.indices.size(); i++) {
      //for (size_t i = 0; i < cloud->points.size(); i++) {
      int point_index = output_indices.indices[i];
      pcl::PointNormal p = cloud->points[point_index];
      Eigen::Vector3f ep = p.getVector3fMap();
      Eigen::Vector3f local_ep = inv_local_coordinates * ep;
      IndexPair pair = projectLocalPointAsIndexPair(local_ep);
      addIndexPair(pair);
    }
    return output_indices.indices.size();
  }
  
  size_t GridPlane::fillCellsFromPointCloud(
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    double distance_threshold)
  {
    std::set<int> dummy;
    return fillCellsFromPointCloud(cloud, distance_threshold, dummy);
  }
  
  jsk_recognition_msgs::SimpleOccupancyGrid GridPlane::toROSMsg()
  {
    jsk_recognition_msgs::SimpleOccupancyGrid ros_msg;
    std::vector<float> coeff;
    convex_->toCoefficients(coeff);
    //JSK_ROS_INFO("coef: [%f, %f, %f, %f]", coeff[0], coeff[1], coeff[2], coeff[3]);
    ros_msg.coefficients[0] = coeff[0];
    ros_msg.coefficients[1] = coeff[1];
    ros_msg.coefficients[2] = coeff[2];
    ros_msg.coefficients[3] = coeff[3];
    ros_msg.resolution = resolution_;
    for (std::set<IndexPair>::iterator it = cells_.begin();
         it != cells_.end();
         ++it) {
      IndexPair pair = *it;
      Eigen::Vector3f c = unprojectIndexPairAsLocalPoint(pair);
      geometry_msgs::Point p;
      pointFromVectorToXYZ<Eigen::Vector3f, geometry_msgs::Point>(
        c, p);
      ros_msg.cells.push_back(p);
    }
    return ros_msg;
  }

  GridPlane GridPlane::fromROSMsg(
    const jsk_recognition_msgs::SimpleOccupancyGrid& rosmsg,
    const Eigen::Affine3f& offset = Eigen::Affine3f::Identity())
  {
    boost::mutex::scoped_lock lock(global_chull_mutex);
    Plane plane = Plane(rosmsg.coefficients).transform(offset);
    // JSK_ROS_INFO("[GridPlane::fromROSMsg] c: [%f, %f, %f, %f]",
    //          rosmsg.coefficients[0],
    //          rosmsg.coefficients[1],
    //          rosmsg.coefficients[2],
    //          rosmsg.coefficients[3]);
    // JSK_ROS_INFO("[GridPlane::fromROSMsg] transformed c: [%f, %f, %f, %f]",
    //          plane.toCoefficients()[0],
    //          plane.toCoefficients()[1],
    //          plane.toCoefficients()[2],
    //          plane.toCoefficients()[3]);
    Eigen::Affine3f plane_coords = plane.coordinates();
    Eigen::Vector3f plane_origin(plane_coords.translation());
    // JSK_ROS_INFO_EIGEN_VECTOR3("[GridPlane::fromROSMsg] plane_origin",
    //                        plane_origin);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      vertices (new pcl::PointCloud<pcl::PointNormal>);
    for (size_t i = 0; i < rosmsg.cells.size(); i++) {
      Eigen::Vector3f local_p(rosmsg.cells[i].x, rosmsg.cells[i].y, 0);
      Eigen::Vector3f global_p = plane.coordinates() * local_p;
      pcl::PointNormal p;
      p.x = global_p[0];
      p.y = global_p[1];
      p.z = global_p[2];
      // JSK_ROS_INFO("[%f, %f, %f] => [%f, %f, %f]",
      //          local_p[0], local_p[1], local_p[2],
      //          global_p[0], global_p[1], global_p[2]);
      vertices->points.push_back(p);
    }
    pcl::ConvexHull<pcl::PointNormal> chull;
    //chull.setDimension(2);
    chull.setInputCloud (vertices);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      convex_vertices_cloud (new pcl::PointCloud<pcl::PointNormal>);
    chull.reconstruct (*convex_vertices_cloud);

    Vertices convex_vertices
      = pointCloudToVertices<pcl::PointNormal>(*convex_vertices_cloud);
    ConvexPolygon::Ptr convex(new ConvexPolygon(convex_vertices));
    // Check orientation
    if (!convex->isSameDirection(plane)) {
      // JSK_ROS_INFO("[GridPlane::fromROSMsg] flip convex");
      //convex = boost::make_shared<ConvexPolygon>(convex->flipConvex());
      Vertices reversed_convex_vertices;
      std::reverse_copy(convex_vertices.begin(), convex_vertices.end(),
                        std::back_inserter(reversed_convex_vertices));
      convex.reset(new ConvexPolygon(reversed_convex_vertices));
    }
    Eigen::Vector3f convex_origin(convex->coordinates().translation());
    Eigen::Vector3f convex_normal = convex->getNormal();
    // JSK_ROS_INFO_EIGEN_VECTOR3("[GridPlane::fromROSMsg] convex_origin",
    //                        convex_origin);
    // JSK_ROS_INFO_EIGEN_VECTOR3("[GridPlane::fromROSMsg] convex_normal",
    //                        convex_normal);
    GridPlane ret(convex, rosmsg.resolution);
    //JSK_ROS_INFO("resolution: %f", ret.resolution_);
    ret.fillCellsFromPointCloud(vertices, 1000.0);
    // JSK_ROS_INFO("cell size: %lu", ret.cells_.size());
    // JSK_ROS_INFO("original cell size: %lu", rosmsg.cells.size());
    return ret;
  }

  
  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot):
    pos_(pos), rot_(rot)
  {
    dimensions_.resize(3);
  }

  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
             const std::vector<double>& dimensions):
    pos_(pos), rot_(rot), dimensions_(dimensions)
  {
    
  }
  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
             const Eigen::Vector3f& dimensions):
    pos_(pos), rot_(rot)
  {
    dimensions_.resize(3);
    dimensions_[0] = dimensions[0];
    dimensions_[1] = dimensions[1];
    dimensions_[2] = dimensions[2];
  }

  Cube::Cube(const Eigen::Vector3f& pos,
             const Line& line_a, const Line& line_b, const Line& line_c)
  {
    double distance_a_b = line_a.distance(line_b);
    double distance_a_c = line_a.distance(line_c);
    double distance_b_c = line_b.distance(line_c);
    Line::Ptr axis;
    dimensions_.resize(3);
    Eigen::Vector3f ex, ey, ez;
    if (distance_a_b >= distance_a_c &&
        distance_a_b >= distance_b_c) {
      axis = line_a.midLine(line_b);
      line_a.parallelLineNormal(line_c, ex);
      line_c.parallelLineNormal(line_b, ey);
      
    }
    else if (distance_a_c >= distance_a_b &&
             distance_a_c >= distance_b_c) {
      axis = line_a.midLine(line_c);
      line_a.parallelLineNormal(line_b, ex);
      line_b.parallelLineNormal(line_c, ey);
    }
    else {
      // else if (distance_b_c >= distance_a_b &&
      //          distance_b_c >= distance_a_c) {
      axis = line_b.midLine(line_c);
      line_b.parallelLineNormal(line_a, ex);
      line_a.parallelLineNormal(line_c, ey);
    }
    dimensions_[0] = ex.norm();
    dimensions_[1] = ey.norm();
    axis->getDirection(ez);
    ez.normalize();
    ex.normalize();
    ey.normalize();
    if (ex.cross(ey).dot(ez) < 0) {
      ez = - ez;
    }
    rot_ = rotFrom3Axis(ex, ey, ez);
    axis->foot(pos, pos_);       // project
  }
  
  Cube::~Cube()
  {

  }

  std::vector<Segment::Ptr> Cube::edges()
  {
    std::vector<Segment::Ptr> ret;
    Eigen::Vector3f A = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f B = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f C = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f D = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (+ dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f E = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f F = pos_
      + rot_ * ((+ dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f G = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (+ dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    Eigen::Vector3f H = pos_
      + rot_ * ((- dimensions_[0] * Eigen::Vector3f::UnitX()) +
                (- dimensions_[1] * Eigen::Vector3f::UnitY()) +
                (- dimensions_[2] * Eigen::Vector3f::UnitZ())) / 2.0;
    
    ret.push_back(Segment::Ptr(new Segment(A, B)));
    ret.push_back(Segment::Ptr(new Segment(B, C)));
    ret.push_back(Segment::Ptr(new Segment(C, D)));
    ret.push_back(Segment::Ptr(new Segment(D, A)));
    ret.push_back(Segment::Ptr(new Segment(E, F)));
    ret.push_back(Segment::Ptr(new Segment(F, G)));
    ret.push_back(Segment::Ptr(new Segment(G, H)));
    ret.push_back(Segment::Ptr(new Segment(H, E)));
    ret.push_back(Segment::Ptr(new Segment(A, E)));
    ret.push_back(Segment::Ptr(new Segment(B, F)));
    ret.push_back(Segment::Ptr(new Segment(C, G)));
    ret.push_back(Segment::Ptr(new Segment(D, H)));
    return ret;
  } 
  
  ConvexPolygon::Ptr Cube::intersectConvexPolygon(Plane& plane)
  {
    std::vector<Segment::Ptr> candidate_edges = edges();
    Vertices intersects;
    for (size_t i = 0; i < candidate_edges.size(); i++) {
      Segment::Ptr edge = candidate_edges[i];
      Eigen::Vector3f p;
      if (edge->intersect(plane, p)) {
        intersects.push_back(p);
      }
    }
    //JSK_ROS_INFO("%lu intersects", intersects.size());
    // Compute convex hull
    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr chull_input
      = verticesToPointCloud<pcl::PointXYZ>(intersects);
    pcl::PointCloud<pcl::PointXYZ>::Ptr chull_cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    chull.setDimension(2);
    chull.setInputCloud(chull_input);
    {
      boost::mutex::scoped_lock lock(global_chull_mutex);
      chull.reconstruct(*chull_cloud);
    }
    
    return ConvexPolygon::Ptr(
      new ConvexPolygon(pointCloudToVertices(*chull_cloud)));
  }
  
  jsk_recognition_msgs::BoundingBox Cube::toROSMsg()
  {
    jsk_recognition_msgs::BoundingBox ret;
    ret.pose.position.x = pos_[0];
    ret.pose.position.y = pos_[1];
    ret.pose.position.z = pos_[2];
    ret.pose.orientation.x = rot_.x();
    ret.pose.orientation.y = rot_.y();
    ret.pose.orientation.z = rot_.z();
    ret.pose.orientation.w = rot_.w();
    ret.dimensions.x = dimensions_[0];
    ret.dimensions.y = dimensions_[1];
    ret.dimensions.z = dimensions_[2];
    return ret;
  }

  Vertices Cube::vertices()
  {
    Vertices vs;
    vs.push_back(buildVertex(0.5, 0.5, 0.5));
    vs.push_back(buildVertex(-0.5, 0.5, 0.5));
    vs.push_back(buildVertex(-0.5, -0.5, 0.5));
    vs.push_back(buildVertex(0.5, -0.5, 0.5));
    vs.push_back(buildVertex(0.5, 0.5, -0.5));
    vs.push_back(buildVertex(-0.5, 0.5, -0.5));
    vs.push_back(buildVertex(-0.5, -0.5, -0.5));
    vs.push_back(buildVertex(0.5, -0.5, -0.5));
    return vs;
  }
    
  
  Polygon::Ptr Cube::buildFace(const Eigen::Vector3f v0,
                               const Eigen::Vector3f v1,
                               const Eigen::Vector3f v2,
                               const Eigen::Vector3f v3)
  {
    Vertices vs;
    vs.push_back(v0);
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);
    Polygon::Ptr(new Polygon(vs));
  }
  
  std::vector<Polygon::Ptr> Cube::faces()
  {
    std::vector<Polygon::Ptr> fs(6);
    Vertices vs = vertices();
    Eigen::Vector3f A = vs[0];
    Eigen::Vector3f B = vs[1];
    Eigen::Vector3f C = vs[2];
    Eigen::Vector3f D = vs[3];
    Eigen::Vector3f E = vs[4];
    Eigen::Vector3f F = vs[5];
    Eigen::Vector3f G = vs[6];
    Eigen::Vector3f H = vs[7];
    Vertices vs0, vs1, vs2, vs3, vs4, vs5, vs6;
    vs0.push_back(A); vs0.push_back(E); vs0.push_back(F); vs0.push_back(B);
    vs1.push_back(B); vs1.push_back(F); vs1.push_back(G); vs1.push_back(C);
    vs2.push_back(C); vs2.push_back(G); vs2.push_back(H); vs2.push_back(D);
    vs3.push_back(D); vs3.push_back(H); vs3.push_back(E); vs3.push_back(A);
    vs4.push_back(A); vs4.push_back(B); vs4.push_back(C); vs4.push_back(D);
    vs5.push_back(E); vs5.push_back(H); vs5.push_back(G); vs5.push_back(F);
    fs[0].reset(new Polygon(vs0));
    fs[1].reset(new Polygon(vs1));
    fs[2].reset(new Polygon(vs2));
    fs[3].reset(new Polygon(vs3));
    fs[4].reset(new Polygon(vs4));
    fs[5].reset(new Polygon(vs5));
    return fs;
  }

  Eigen::Vector3f Cube::buildVertex(double i, double j, double k)
  {
    Eigen::Vector3f local = (Eigen::Vector3f::UnitX() * i * dimensions_[0] +
                             Eigen::Vector3f::UnitY() * j * dimensions_[1] +
                             Eigen::Vector3f::UnitZ() * k * dimensions_[2]);
    return Eigen::Translation3f(pos_) * rot_ * local;
  }
  
  Eigen::Vector3f Cube::nearestPoint(const Eigen::Vector3f& p,
                                     double& distance)
  {
    std::vector<Polygon::Ptr> current_faces = faces();
    double min_distance = DBL_MAX;
    Eigen::Vector3f min_point;
    for (size_t i = 0; i < current_faces.size(); i++) {
      Polygon::Ptr f = current_faces[i];
      double d;
      Eigen::Vector3f q = f->nearestPoint(p, d);
      if (min_distance > d) {
        min_distance = d;
        min_point = q;
      }
    }
    distance = min_distance;
    return min_point;
  }

  Cylinder::Cylinder(Eigen::Vector3f point, Eigen::Vector3f direction, double radius):
    point_(point), direction_(direction), radius_(radius)
  {

  }

  void Cylinder::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const double threshold,
                                  pcl::PointIndices& output)
  {
    Line line(direction_, point_);
    output.indices.clear();
    for (size_t i = 0; i < cloud.points.size(); i++) {
      Eigen::Vector3f p = cloud.points[i].getVector3fMap();
      double d = line.distanceToPoint(p);
      if (d < radius_ + threshold && d > radius_ - threshold) {
        output.indices.push_back(i);
      }
    }
  }

  void Cylinder::estimateCenterAndHeight(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                         const pcl::PointIndices& indices,
                                         Eigen::Vector3f& center,
                                         double& height)
  {
    Line line(direction_, point_);
    Vertices points;
    for (size_t i = 0; i < indices.indices.size(); i++) {
      int point_index = indices.indices[i];
      points.push_back(cloud.points[point_index].getVector3fMap());
    }
    PointPair min_max = line.findEndPoints(points);
    Eigen::Vector3f min_point = min_max.get<0>();
    Eigen::Vector3f max_point = min_max.get<1>();
    Eigen::Vector3f min_point_projected, max_point_projected;
    line.foot(min_point, min_point_projected);
    line.foot(max_point, max_point_projected);
    height = (min_point_projected - max_point_projected).norm();
    center = (min_point_projected + max_point_projected) / 2.0;
  }

  void Cylinder::toMarker(visualization_msgs::Marker& marker,
                          const Eigen::Vector3f& center,
                          const Eigen::Vector3f& uz,
                          const double height)
  {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = center[0];
    marker.pose.position.y = center[1];
    marker.pose.position.z = center[2];
    Eigen::Vector3f orig_z(0, 0, 1);
    Eigen::Quaternionf q;
    q.setFromTwoVectors(orig_z, uz);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = radius_ * 2;
    marker.scale.y = radius_ * 2;
    marker.scale.z = height;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }

  Eigen::Vector3f Cylinder::getDirection()
  {
    return direction_;
  }
}
