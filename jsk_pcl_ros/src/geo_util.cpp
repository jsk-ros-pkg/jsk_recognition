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
    // ROS_INFO("min: %f", min_alpha);
    // ROS_INFO("max: %f", max_alpha);
    return boost::make_tuple<Point, Point>(min_alpha_point, max_alpha_point);
  }

  void Line::print()
  {
    ROS_INFO("d: [%f, %f, %f], p: [%f, %f, %f]", direction_[0], direction_[1], direction_[2],
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
    foot(point, foot_point);
    return (foot_point - point).norm();
  }

  // double Segment::distance(const Segment& other)
  // {
    
  // }
  
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
  
  Polygon::Polygon(const Vertices& vertices):
    Plane((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalized(), vertices[0]),
    vertices_(vertices)
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
  
  Eigen::Vector3f Polygon::directionAtPoint(size_t i)
  {
    Eigen::Vector3f O = vertices_[i];
    Eigen::Vector3f A = vertices_[previousIndex(i)];
    Eigen::Vector3f B = vertices_[nextIndex(i)];
    Eigen::Vector3f OA = A - O;
    Eigen::Vector3f OB = B - O;
    Eigen::Vector3f n = (OA.normalized()).cross(OB.normalized());
    if (n.norm() == 0) {
      // ROS_ERROR("normal is 0");
      // ROS_ERROR("O: [%f, %f, %f]", O[0], O[1], O[2]);
      // ROS_ERROR("A: [%f, %f, %f]", A[0], A[1], A[2]);
      // ROS_ERROR("B: [%f, %f, %f]", B[0], B[1], B[2]);
      // ROS_ERROR("OA: [%f, %f, %f]", OA[0], OA[1], OA[2]);
      // ROS_ERROR("OB: [%f, %f, %f]", OB[0], OB[1], OB[2]);
      //exit(1);
    }
    return n.normalized();
  }
  
  bool Polygon::isTriangle() {
    return vertices_.size() == 3;
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
          ROS_ERROR("removed: %lu", i);
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
    //ROS_INFO("direction: [%f, %f, %f]", the_direction[0], the_direction[1], the_direction[2]);
    if (the_direction.norm() == 0.0) {
      ROS_ERROR("malformed polygon");
      exit(1);
    }
    if (direction.dot(the_direction) < 0) {
#ifdef DEBUG_GEO_UTIL
      ROS_INFO("triangle is not same direction");
      ROS_INFO("direction: [%f, %f, %f]", direction[0], direction[1], direction[2]);
      ROS_INFO("the_direction: [%f, %f, %f]",
               the_direction[0],
               the_direction[1],
               the_direction[2]);
      for (size_t i = 0; i < vertices_.size(); i++) {
        Eigen::Vector3f v = directionAtPoint(i);
        ROS_INFO("the_direction[%lu]: [%f, %f, %f]",
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
          ROS_INFO("P: [%f, %f, %f]", P[0], P[1], P[2]);
          ROS_INFO("A: [%f, %f, %f]", A[0], A[1], A[2]);
          ROS_INFO("B: [%f, %f, %f]", B[0], B[1], B[2]);
          ROS_INFO("C: [%f, %f, %f]", C[0], C[1], C[2]);
          ROS_INFO("Across: [%f, %f, %f]", Across[0], Across[1], Across[2]);
          ROS_INFO("Bcross: [%f, %f, %f]", Bcross[0], Bcross[1], Bcross[2]);
          ROS_INFO("Ccross: [%f, %f, %f]", Ccross[0], Ccross[1], Ccross[2]);
          ROS_INFO("Across-Bcross: %f", Across.dot(Bcross));
          ROS_INFO("Bcross-Ccross: %f", Bcross.dot(Ccross));
          ROS_INFO("Ccross-Across: %f", Ccross.dot(Across));
#endif
          if (((Across.dot(Bcross) > 0 &&
                Bcross.dot(Ccross) > 0 &&
                Ccross.dot(Across) > 0) ||
               (Across.dot(Bcross) < 0 &&
                Bcross.dot(Ccross) < 0 &&
                Ccross.dot(Across) < 0))) {
            // ROS_ERROR("%lu -- %lu is inside", index, i);
            return false;
          }
          // ConvexPolygon convex_triangle(triangle_candidate->vertices_);
          // if (convex_triangle.isInside(v)) {
          //   //ROS_INFO("vertices is inside of the polygon");
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
      ROS_INFO("n[%lu] [%f, %f, %f]", i, n[0], n[1], n[2]);
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
    std::vector<Polygon::Ptr> ret;
    
    // if this polygon is triangle, return immediately
    if (isTriangle()) {
      ret.push_back(Polygon::Ptr(this));
      return ret;
    }
    // the first triangle is special. the first triangle is at
    // farest point from origin.
    Eigen::Vector3f O(0, 0, 0);
    size_t farest_point_index = getFarestPointIndex(O);
    // we can remove the first triangle.
    Polygon::PtrPair polygon_pair = separatePolygon(farest_point_index);
    Polygon::Ptr first_triangle = polygon_pair.get<0>();
    Eigen::Vector3f way_of_normal = directionAtPoint(farest_point_index);
    if (way_of_normal.norm() == 0.0) {
      ROS_ERROR("way_of_norml is 0");
      exit(3);
    }
    ret.push_back(first_triangle);
    Polygon::Ptr the_polygon = polygon_pair.get<1>();
    while (the_polygon->getNumVertices() >= 3) {
      //ROS_INFO("found triangle:: %lu", the_polygon->getNumVertices());
      // the end of removal
      if (the_polygon->isTriangle()) {
        ret.push_back(the_polygon);
        break;
      }
      else {
#ifdef DEBUG_GEO_UTIL
        for (size_t i = 0; i < the_polygon->getNumVertices(); i++) {
          Eigen::Vector3f n = the_polygon->getVertex(i);
          ROS_WARN("v[%lu] [%f, %f, %f]", i, n[0], n[1], n[2]);
        }
        for (size_t i = 0; i < the_polygon->getNumVertices(); i++) {
          Eigen::Vector3f n = the_polygon->directionAtPoint(i);
          ROS_WARN("d[%lu] [%f, %f, %f]", i, n[0], n[1], n[2]);
        }
#endif
        // need to remove triangle patch
        size_t next_index = 0;
        while (true) {          // loop until success to remove next polygon
          if (the_polygon->isPossibleToRemoveTriangleAtIndex(
                next_index, way_of_normal)) {
            Polygon::PtrPair next_pair
              = the_polygon->separatePolygon(next_index);
            ret.push_back(next_pair.get<0>());
            the_polygon = next_pair.get<1>();
            break;
          }
          else {
            next_index = the_polygon->nextIndex(next_index);
            if (next_index == 0) {
              ROS_ERROR("looped! %lu", the_polygon->getNumVertices());
#ifdef DEBUG_GEO_UTIL
              for (size_t i = 0; i < the_polygon->getNumVertices(); i++) {
                Eigen::Vector3f v = the_polygon->getVertex(i);
                ROS_INFO("v[%lu] [%f, %f, %f]", i, v[0], v[1], v[2]);
                // ROS_INFO("Eigen::Vector3f v%lu(%f, %f, %f);", i, v[0], v[1], v[2]);
                // ROS_INFO("vs.push_back(v%lu);", i);
              }
              for (size_t i = 0; i < the_polygon->getNumVertices(); i++) {
                Eigen::Vector3f v0 = the_polygon->getVertex(i);
                Eigen::Vector3f v1 = the_polygon->getVertex(the_polygon->nextIndex(i));
                Eigen::Vector3f v = (v1 - v0);
                ROS_INFO("d[%lu] [%f, %f, %f]", i, v[0], v[1], v[2]);
                ROS_INFO("d[%lu] %f", i, v.norm());
              }
#endif
              return ret;
            }
          }
        }
      }
    }
    return ret;
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
  
  ConvexPolygon::ConvexPolygon(const Vertices& vertices,
                               const std::vector<float>& coefficients):
    Plane(coefficients), vertices_(vertices)
  {

  }
    
  
  ConvexPolygon::ConvexPolygon(const Vertices& vertices):
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
        if (direction.cross(direction2).dot(normal_) > 0) {
          continue;
        }
        else {
          return false;
        }
      }
      else {
        if (direction.cross(direction2).dot(normal_) < 0) {
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
    Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3f p;
      pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(
        polygon.points[i], p);
      vertices.push_back(p);
    }
    return ConvexPolygon(vertices);
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
    return convex_distance > distance_threshold;
  }

  double ConvexPolygon::area()
  {
    double sum = 0.0;
    for (size_t i = 0; i < vertices_.size(); i++) {
      Eigen::Vector3f p_k = vertices_[i];
      Eigen::Vector3f p_k_1;
      if (i == vertices_.size() - 1) {
        p_k_1 = vertices_[0];
      }
      else {
        p_k_1 = vertices_[i + 1];
      }
      sum += p_k.cross(p_k_1).norm();
    }
    return sum / 2.0;
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

  ConvexPolygon::Ptr ConvexPolygon::magnify(const double scale_factor)
  {
    // compute centroid
    Eigen::Vector3f centroid(0, 0, 0);
    for (size_t i = 0; i < vertices_.size(); i++) {
      centroid = centroid + vertices_[i];
    }
    centroid = centroid / vertices_.size();

    Vertices new_vertices;
    for (size_t i = 0; i < vertices_.size(); i++) {
      new_vertices.push_back((vertices_[i] - centroid) * scale_factor
                             + centroid);
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
  
  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot):
    pos_(pos), rot_(rot)
  {
    
  }

  Cube::Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
             const std::vector<double>& dimensions):
    pos_(pos), rot_(rot), dimensions_(dimensions)
  {
    
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

  BoundingBox Cube::toROSMsg()
  {
    BoundingBox ret;
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
  
}
