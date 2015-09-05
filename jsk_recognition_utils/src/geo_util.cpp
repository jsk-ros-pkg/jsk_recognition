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
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <algorithm>
#include <iterator>
#include <cfloat>
#include <pcl/conversions.h>
#include <boost/tuple/tuple_comparison.hpp>

#include <boost/foreach.hpp>
#include <boost/range/irange.hpp>
#include <boost/math/special_functions/round.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <pcl/point_types.h>
#include <pcl/surface/processing.h>
#include "jsk_recognition_utils/sensor_model_utils.h"

// #define DEBUG_GEO_UTIL
namespace jsk_recognition_utils
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
