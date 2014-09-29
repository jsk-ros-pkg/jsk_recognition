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

#include "jsk_pcl_ros/edgebased_cube_finder.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <jsk_pcl_ros/BoundingBoxArray.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>
#include <jsk_pcl_ros/PolygonArray.h>
#include <jsk_pcl_ros/ClusterPointIndices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

namespace jsk_pcl_ros
{
  CubeHypothesis::CubeHypothesis(const IndicesPair& pair,
                                 const CoefficientsPair& coefficients_pair,
                                 const double outlier_threshold):
    value_(0.0), indices_pair_(pair), coefficients_pair_(coefficients_pair),
    outlier_threshold_(outlier_threshold_)
  {

  }
  
  CubeHypothesis::~CubeHypothesis()
  {
  }

  void CubeHypothesis::computeCentroid(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::PointIndices::Ptr& indices,
    Eigen::Vector3f& output)
  {
    Eigen::Vector4f centroid;
    //pcl::compute3DCentroid(cloud, indices.indices, centroid);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.filter(*target_cloud);
    pcl::compute3DCentroid(*target_cloud, centroid);
    pointFromVectorToVector<Eigen::Vector4f, Eigen::Vector3f>(centroid, output);
  }

  void CubeHypothesis::getLinePoints(
    const Line& line,
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    const pcl::PointIndices::Ptr indices,
    Vertices& output)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(indices);
    extract.filter(*points);
    for (size_t i = 0; i < points->points.size(); i++) {
      pcl::PointXYZRGB p = points->points[i];
      Eigen::Vector3f p_eigen = p.getVector3fMap();
      Eigen::Vector3f foot_point;
      line.foot(p_eigen, foot_point);
      output.push_back(foot_point);
    }
  }

  ConvexPolygon::Ptr CubeHypothesis::buildConvexPolygon(
    const PointPair& a_edge_pair, const PointPair& b_edge_pair)
  {
    Vertices vertices;
    vertices.push_back(a_edge_pair.get<0>());
    vertices.push_back(a_edge_pair.get<1>());
    vertices.push_back(b_edge_pair.get<1>());
    vertices.push_back(b_edge_pair.get<0>());
    ConvexPolygon::Ptr convex (new ConvexPolygon(vertices));
    return convex;
  }
  
  double CubeHypothesis::evaluatePointOnPlanes(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    ConvexPolygon& polygon_a,
    ConvexPolygon& polygon_b)
  {
    std::vector<int> a_indices, b_indices;
    for (size_t i = 0; i < cloud.points.size(); i++) {
      pcl::PointXYZRGB pcl_point = cloud.points[i];
      if (pcl_isfinite(pcl_point.x) &&
          pcl_isfinite(pcl_point.y) &&
          pcl_isfinite(pcl_point.z)) { // we don't care nan points
        Eigen::Vector3f eigen_point = pcl_point.getVector3fMap();
        if (polygon_a.distanceSmallerThan(eigen_point, outlier_threshold_)) {
          a_indices.push_back(i);
        }
        if (polygon_b.distanceSmallerThan(eigen_point, outlier_threshold_)) {
          b_indices.push_back(i);
        }
      }
    }
    // ...the number of points...?
    return a_indices.size() + b_indices.size();
  }
  
  PointPair CubeHypothesis::computeAxisEndPoints(
    const Line& axis,
    const PointPair& a_candidates,
    const PointPair& b_candidates)
  {
    Vertices original_points;
    original_points.push_back(a_candidates.get<0>());
    original_points.push_back(a_candidates.get<1>());
    original_points.push_back(b_candidates.get<0>());
    original_points.push_back(b_candidates.get<1>());
    for (size_t i = 0; i < original_points.size(); i++) {
      Eigen::Vector3f p = original_points[i];
      ROS_INFO("[foot_point] [%f, %f, %f]", p[0], p[1], p[2]);
    }
    
    Vertices foot_points;
    for (size_t i = 0; i < original_points.size(); i++) {
      Eigen::Vector3f foot_point;
      axis.foot(original_points[i], foot_point);
      foot_points.push_back(foot_point);
    }
    double max_alpha = -DBL_MAX;
    double min_alpha = DBL_MAX;
    Eigen::Vector3f max_alpha_point, min_alpha_point;
    
    for (size_t i = 0; i < foot_points.size(); i++) {
      double alpha = axis.computeAlpha(foot_points[i]);
      if (alpha > max_alpha) {
        max_alpha = alpha;
        max_alpha_point = foot_points[i];
      }
      if (alpha < min_alpha) {
        min_alpha = alpha;
        min_alpha_point = foot_points[i];
      }
    }
    ROS_INFO("min_alpha_point: [%f, %f, %f]", min_alpha_point[0], min_alpha_point[1], min_alpha_point[2]);
    ROS_INFO("max_alpha_point: [%f, %f, %f]", max_alpha_point[0], max_alpha_point[1], max_alpha_point[2]);
    return boost::make_tuple(min_alpha_point, max_alpha_point);
  }

  
  PlanarCubeHypothesis::PlanarCubeHypothesis(
    const IndicesPair& pair, const CoefficientsPair& coefficients_pair, const double outlier_threshold):
    CubeHypothesis(pair, coefficients_pair, outlier_threshold)
  {

  }
  
  DiagnoalCubeHypothesis::DiagnoalCubeHypothesis(
    const IndicesPair& pair, const CoefficientsPair& coefficients_pair,
    const double outlier_threshold):
    CubeHypothesis(pair, coefficients_pair, outlier_threshold), resolution_(10)
  {

  }

  void DiagnoalCubeHypothesis::estimate(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
  {
    const double dt = (M_PI - 2.0 * min_angle_) / resolution_;
    Line::Ptr line_a
      = Line::fromCoefficients(coefficients_pair_.get<0>()->values);
    Line::Ptr line_b
      = Line::fromCoefficients(coefficients_pair_.get<1>()->values);
    if (!line_a->isSameDirection(*line_b)) {
      line_b = line_b->flip();
    }
    
    const double r2 = line_a->distance(*line_b);
    const double r = r2 / 2;
    Line::Ptr axis = line_a->midLine(*line_b);
    Eigen::Vector3f center;
    axis->getOrigin(center);
    ROS_INFO("line_a:");
    line_a->print();
    ROS_INFO("line_b:");
    line_b->print();
    ROS_INFO("axis:");
    axis->print();
    ROS_INFO("r: %f", r);
    
    // before evaluation, we fix line_a and line_b to be parallel
    // against line_c and axis.
    // in order to align line_a and line_b, we compute centroids of the points
    // on line_a and line_b and rotate the lines around the points.
    Eigen::Vector3f centroid_a, centroid_b;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = cloud.makeShared();
    computeCentroid(cloud_ptr, indices_pair_.get<0>(), centroid_a);
    computeCentroid(cloud_ptr, indices_pair_.get<1>(), centroid_b);
    ROS_INFO("centroid_a: [%f, %f, %f]", centroid_a[0], centroid_a[1], centroid_a[2]);
    ROS_INFO("centroid_b: [%f, %f, %f]", centroid_b[0], centroid_b[1], centroid_b[2]);
    Line::Ptr line_a_aligned = axis->parallelLineOnAPoint(centroid_a);
    Line::Ptr line_b_aligned = axis->parallelLineOnAPoint(centroid_b);
    ROS_INFO("line_a_aligned:");
    line_a_aligned->print();
    ROS_INFO("line_b_aligned:");
    line_b_aligned->print();
    
    Vertices line_a_points, line_b_points;
    getLinePoints(*line_a_aligned, cloud, indices_pair_.get<0>(),
                  line_a_points);
    getLinePoints(*line_b_aligned, cloud, indices_pair_.get<1>(),
                  line_b_points);
    PointPair line_a_end_points = line_a->findEndPoints(line_a_points);
    PointPair line_b_end_points = line_b->findEndPoints(line_b_points);
    double max_v = - DBL_MAX;
    double max_theta;
    Line::Ptr max_line_c;
    PointPair max_line_c_a_points, max_line_c_b_points;
    for (size_t i = 0; i < resolution_; i++) {
      ROS_INFO("estimate i: %lu", i);
      double theta = dt * i + min_angle_;
      Eigen::Vector3f point_on_x;
      line_a->foot(center, point_on_x);
      // 2D local cooridnate: r * cos(theta), r * sin(theta)
      // convert it according to axis
      // x // (point_on_x - center)
      // z // axis
      Eigen::Vector3f ex = (point_on_x - center).normalized();
      Eigen::Vector3f ez;
      axis->getDirection(ez);
      Eigen::Vector3f ey = ez.cross(ex).normalized();
      // ey should direct to origin. If not, ex should be flipped
      if (center.dot(ey) > 0) {
        ex = -ex;
        ey = ez.cross(ex).normalized();
      }
      Eigen::Vector3f point_on_y = center + r * ey;
      Line::Ptr line_c = axis->parallelLineOnAPoint(point_on_y);
      // line_a, b, c are almost parallel and these 3 lines make
      // 2 planes.
      //  ------------------------ line_a
      // /         A            /
      // ----------------------- line_c
      // \         B            \
      //  ----------------------- line_b
      
      // in order to evaluate plane A, first we exract the points
      // which have distance |line_a - line_c| + margin from line_c.
      // Second, For all those points, count all the points whose distance from
      // plane A is smaller than margin.
      // Thrid, we compute the area of the plane A and normalize the value.
      Eigen::Vector3f line_c_a_min_point, line_c_a_max_point;
      Eigen::Vector3f line_c_b_min_point, line_c_b_max_point;
      line_c->foot(line_a_end_points.get<0>(), line_c_a_min_point);
      line_c->foot(line_a_end_points.get<1>(), line_c_a_max_point);
      line_c->foot(line_b_end_points.get<0>(), line_c_b_min_point);
      line_c->foot(line_b_end_points.get<1>(), line_c_b_max_point);
      PointPair line_c_a_end_points = boost::make_tuple(line_c_a_min_point,
                                                        line_c_a_max_point);
      PointPair line_c_b_end_points = boost::make_tuple(line_c_b_min_point,
                                                        line_c_b_max_point);
      ConvexPolygon::Ptr plane_a = buildConvexPolygon(line_a_end_points,
                                                      line_c_a_end_points);
      ConvexPolygon::Ptr plane_b = buildConvexPolygon(line_b_end_points,
                                                      line_c_b_end_points);
      double v = evaluatePointOnPlanes(cloud, *plane_a, *plane_b);
      if (max_v < v) {
        max_v = v;
        max_theta = theta;
        max_line_c = line_c;
        max_line_c_a_points = line_c_a_end_points;
        max_line_c_b_points = line_c_b_end_points;
      } 
   }
    value_ = max_v;
    // estimate centroid
    PointPair axis_end_points = computeAxisEndPoints(
      *axis,
      max_line_c_a_points,
      max_line_c_b_points);
    ROS_INFO("end_point: [%f, %f, %f]", axis_end_points.get<0>()[0], axis_end_points.get<0>()[1], axis_end_points.get<0>()[2]);
    ROS_INFO("end_point: [%f, %f, %f]", axis_end_points.get<1>()[0], axis_end_points.get<1>()[1], axis_end_points.get<1>()[2]);
    Eigen::Vector3f midpoint
      = (axis_end_points.get<0>() + axis_end_points.get<1>()) / 2.0;
    double z_dimension = (axis_end_points.get<0>() - midpoint).norm() * 2;
    // compute cube
    ROS_INFO("midpoint: [%f, %f, %f]", midpoint[0], midpoint[1], midpoint[2]);
    cube_.reset(new Cube(midpoint, *line_a_aligned, *line_b_aligned, *max_line_c));
    std::vector<double> dimensions = cube_->getDimensions();
    dimensions[2] = z_dimension;
    cube_->setDimensions(dimensions);
  }

  int EdgebasedCubeFinder::countInliers(
    const pcl::PointCloud<PointT>::Ptr cloud,
    const ConvexPolygon::Ptr convex)
  {
    int num = 0;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
        Eigen::Vector3f ep = p.getVector3fMap();
        if (convex->distanceSmallerThan(ep, outlier_threshold_)) {
          num++;
        }
      }
    }
    return num;
  }
  
  void EdgebasedCubeFinder::filterBasedOnConvex(
    const pcl::PointCloud<PointT>::Ptr cloud,
    const std::vector<ConvexPolygon::Ptr>& convexes,
    std::vector<int>& output_indices)
  {
    
    for (size_t i = 0; i < convexes.size(); i++) {
      ConvexPolygon::Ptr convex = convexes[i];
      if (true) {
        // if (convex->area() > convex_area_threshold_ &&
        //     convex->allEdgesLongerThan(convex_edge_threshold_)) {
        //int inliers = countInliers(cloud, convex);
        //ROS_INFO("inliers: %d", inliers);
        //if (inliers > min_inliers_) {
        if (true) {
          output_indices.push_back(i);
        }
      }
    }
  }

  void EdgebasedCubeFinder::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    outlier_threshold_ = config.outlier_threshold;
    min_inliers_ = config.min_inliers;
    convex_area_threshold_ = config.convex_area_threshold;
    convex_edge_threshold_ = config.convex_edge_threshold;
    parallel_edge_distance_min_threshold_ = config.parallel_edge_distance_min_threshold;
    parallel_edge_distance_max_threshold_ = config.parallel_edge_distance_max_threshold;
  }
  
  void EdgebasedCubeFinder::onInit()
  {
    PCLNodelet::onInit();
    

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EdgebasedCubeFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);

    
    ////////////////////////////////////////////////////////
    // publishers
    ////////////////////////////////////////////////////////
    pub_ = advertise<jsk_pcl_ros::BoundingBoxArray>(*pnh_, "output", 1);
    pub_pose_array_
      = advertise<geometry_msgs::PoseArray>(*pnh_, "output_pose_array", 1);
    pub_debug_marker_
      = advertise<visualization_msgs::Marker>(*pnh_, "debug_marker", 1);
    pub_debug_filtered_cloud_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "debug_filtered_cloud", 1);
    pub_debug_polygons_
      = advertise<jsk_pcl_ros::PolygonArray>(*pnh_, "debug_polygons", 1);
    pub_debug_clusers_
      = advertise<ClusterPointIndices>(*pnh_, "debug_clusters", 1);
  }

  void EdgebasedCubeFinder::subscribe()
  {
    ////////////////////////////////////////////////////////
    // subscription
    ////////////////////////////////////////////////////////
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_edges_.subscribe(*pnh_, "input_edges", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_edges_);
    sync_->registerCallback(boost::bind(
                              &EdgebasedCubeFinder::estimate, this, _1, _2));
  }

  void EdgebasedCubeFinder::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_edges_.unsubscribe();
  }

  Line::Ptr EdgebasedCubeFinder::midLineFromCoefficientsPair(
    const CoefficientsPair& pair)
  {
    pcl::ModelCoefficients::Ptr coefficients_a = pair.get<0>();
    pcl::ModelCoefficients::Ptr coefficients_b = pair.get<1>();
    Line::Ptr line_a = Line::fromCoefficients(coefficients_a->values);
    Line::Ptr line_b = Line::fromCoefficients(coefficients_b->values);
    return line_a->midLine(*line_b);
  }

  pcl::PointCloud<EdgebasedCubeFinder::PointT>::Ptr EdgebasedCubeFinder::extractPointCloud(
    const pcl::PointCloud<PointT>::Ptr cloud,
    const pcl::PointIndices::Ptr indices)
  {
    pcl::PointCloud<PointT>::Ptr ret (new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.filter(*ret);
    return ret;
  }

  PointPair EdgebasedCubeFinder::minMaxPointOnLine(
    const Line& line,
    const pcl::PointCloud<PointT>::Ptr cloud)
  {
    Vertices points;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      Eigen::Vector3f eigen_p = p.getVector3fMap();
      Eigen::Vector3f foot;
      line.foot(eigen_p, foot);
      points.push_back(foot);
    }
    return line.findEndPoints(points);
  }
  
  ConvexPolygon::Ptr EdgebasedCubeFinder::convexFromPairs(
    const pcl::PointCloud<PointT>::Ptr cloud,
    const CoefficientsPair& coefficients_pair,
    const IndicesPair& indices_pair)
  {
    pcl::ModelCoefficients::Ptr coefficients_a = coefficients_pair.get<0>();
    pcl::ModelCoefficients::Ptr coefficients_b = coefficients_pair.get<1>();
    pcl::PointIndices::Ptr indices_a = indices_pair.get<0>();
    pcl::PointIndices::Ptr indices_b = indices_pair.get<1>();
    
    pcl::PointCloud<PointT>::Ptr cloud_a = extractPointCloud(cloud, indices_a);
    pcl::PointCloud<PointT>::Ptr cloud_b = extractPointCloud(cloud, indices_b);

    Line::Ptr line_a = Line::fromCoefficients(coefficients_a->values);
    Line::Ptr line_b = Line::fromCoefficients(coefficients_b->values);
    PointPair a_min_max = minMaxPointOnLine(*line_a, cloud_a);
    PointPair b_min_max = minMaxPointOnLine(*line_b, cloud_b);
    Vertices vertices;
    vertices.push_back(a_min_max.get<0>());
    vertices.push_back(a_min_max.get<1>());
    vertices.push_back(b_min_max.get<1>());
    vertices.push_back(b_min_max.get<0>());
    ConvexPolygon::Ptr ret (new ConvexPolygon(vertices));
    return ret;
  }
  
  void EdgebasedCubeFinder::filterPairsBasedOnParallelEdgeDistances(
    const std::vector<IndicesPair>& pairs,
    const std::vector<CoefficientsPair>& coefficients_pair,
    std::vector<IndicesPair>& filtered_indices_pairs,
    std::vector<CoefficientsPair>& filtered_coefficients_pairs)
  {
    for (size_t i = 0; i < coefficients_pair.size(); i++) {
      CoefficientsPair coefficients = coefficients_pair[i];
      pcl::ModelCoefficients::Ptr coefficients_a = coefficients_pair[i].get<0>();
      pcl::ModelCoefficients::Ptr coefficients_b = coefficients_pair[i].get<1>();
      Line::Ptr line_a = Line::fromCoefficients(coefficients_a->values);
      Line::Ptr line_b = Line::fromCoefficients(coefficients_b->values);

      // force to align two lines
      Line::Ptr axis = line_a->midLine(*line_b);
      Eigen::Vector3f origin_a, origin_b;
      line_a->getOrigin(origin_a);
      line_b->getOrigin(origin_b);
      Line::Ptr line_a_aligned = axis->parallelLineOnAPoint(origin_a);
      Line::Ptr line_b_aligned = axis->parallelLineOnAPoint(origin_b);
      Eigen::Vector3f distance_vector;
      line_a_aligned->parallelLineNormal(*line_b_aligned, distance_vector);
      double distance = distance_vector.norm();
      //double distance = line_a_aligned->distance(*line_b_aligned);
      ROS_INFO("d: %f", distance);
      if (distance < parallel_edge_distance_max_threshold_ &&
          distance > parallel_edge_distance_min_threshold_) {
        filtered_indices_pairs.push_back(pairs[i]);
        filtered_coefficients_pairs.push_back(coefficients);
      }
    }
  }

  //pcl::PointCloud<EdgebasedCubeFinder::PointT>::Ptr
  pcl::PointIndices::Ptr
  EdgebasedCubeFinder::preparePointCloudForRANSAC(
    const ConvexPolygon::Ptr convex,
    const CoefficientsPair& edge_coefficients_pair,
    const pcl::PointCloud<PointT>::Ptr cloud)
  {
    // extract the points which can be projected onto the convex.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    ConvexPolygon::Ptr magnified_convex = convex->magnify(1.1);
    pcl::PointCloud<PointT>::Ptr ret (new pcl::PointCloud<PointT>);
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
        Eigen::Vector3f ep = p.getVector3fMap();
        Eigen::Vector3f foot;
        magnified_convex->projectOnPlane(ep, foot);
        if (magnified_convex->isInside(foot) && convex->distanceSmallerThan(ep, outlier_threshold_)) {
        //if (magnified_convex->isInside(foot) && (ep - foot).norm()) {
          //NODELET_INFO("distance: %f", (ep - foot).norm());
          indices->indices.push_back(i);
        }
      }
    }
    return indices;
   }

   void EdgebasedCubeFinder::estimateParallelPlane(
     const ConvexPolygon::Ptr convex,
     const pcl::PointCloud<PointT>::Ptr filtered_cloud,
     pcl::PointIndices::Ptr output_inliers,
     pcl::ModelCoefficients::Ptr output_coefficients)
   {
     Eigen::Vector3f normal = convex->getNormal();
     pcl::SACSegmentation<PointT> seg;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setDistanceThreshold (outlier_threshold_);
     seg.setInputCloud(filtered_cloud);
     seg.setMaxIterations (10000);
     seg.setAxis(normal);
     seg.setEpsAngle(pcl::deg2rad(10.0));
     seg.segment (*output_inliers, *output_coefficients);
   }

   void EdgebasedCubeFinder::estimatePerpendicularPlane(
     const ConvexPolygon::Ptr convex,
     const CoefficientsPair& edge_coefficients,
     const pcl::PointCloud<PointT>::Ptr filtered_cloud,
     pcl::PointIndices::Ptr output_inliers,
     pcl::ModelCoefficients::Ptr output_coefficients)
   {
     Eigen::Vector3f normal_a = convex->getNormal();
     Line::Ptr mid_line = midLineFromCoefficientsPair(edge_coefficients);
     Eigen::Vector3f normal_b;
     mid_line->getDirection(normal_b);
     Eigen::Vector3f normal = normal_a.cross(normal_b);
     pcl::SACSegmentation<PointT> seg;
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setDistanceThreshold (outlier_threshold_);
     seg.setInputCloud(filtered_cloud);
     seg.setMaxIterations (10000);
     seg.setAxis(normal);
     seg.setEpsAngle(pcl::deg2rad(5.0));
     seg.segment (*output_inliers, *output_coefficients);
   }

  Cube::Ptr EdgebasedCubeFinder::cubeFromIndicesAndCoefficients(
    const pcl::PointCloud<EdgebasedCubeFinder::PointT>::Ptr cloud,
    const IndicesCoefficientsTriple& indices_coefficients_triple,
    pcl::PointCloud<EdgebasedCubeFinder::PointT>::Ptr points_on_edge)
  {
    Eigen::Vector3f ex, ey, ez;
    CoefficientsTriple coefficients_triple
      = indices_coefficients_triple.get<1>();
    IndicesTriple indices_triple
      = indices_coefficients_triple.get<0>();
    // do we need to align lines...??
    Line::Ptr mid_line
      = Line::fromCoefficients(coefficients_triple.get<0>()->values);
    Line::Ptr line_a
      = Line::fromCoefficients(coefficients_triple.get<1>()->values);
    Line::Ptr line_b
      = Line::fromCoefficients(coefficients_triple.get<2>()->values);
    // force to align
    if (!mid_line->isSameDirection(*line_a)) {
      line_a = line_a->flip();
    }
    if (!mid_line->isSameDirection(*line_b)) {
      line_b = line_b->flip();
    }
    Line::Ptr axis = line_a->midLine(*line_b);
    
    pcl::PointCloud<PointT>::Ptr point_on_a
      = extractPointCloud(cloud,
                          indices_triple.get<1>());
    pcl::PointCloud<PointT>::Ptr point_on_b
      = extractPointCloud(cloud,
                          indices_triple.get<2>());
    pcl::PointCloud<PointT>::Ptr point_on_c
      = extractPointCloud(cloud,
                          indices_triple.get<0>());
    Eigen::Vector4f a_centroid4, b_centroid4, c_centroid4;
    Eigen::Vector3f a_centroid, b_centroid, c_centroid;
    pcl::compute3DCentroid(*point_on_a, a_centroid4);
    pcl::compute3DCentroid(*point_on_b, b_centroid4);
    pcl::compute3DCentroid(*point_on_c, c_centroid4);
    pointFromVectorToVector<Eigen::Vector4f, Eigen::Vector3f>(
      a_centroid4, a_centroid);
    pointFromVectorToVector<Eigen::Vector4f, Eigen::Vector3f>(
      b_centroid4, b_centroid);
    pointFromVectorToVector<Eigen::Vector4f, Eigen::Vector3f>(
      c_centroid4, c_centroid);
    
    Line::Ptr line_a_aligned = axis->parallelLineOnAPoint(a_centroid);
    Line::Ptr line_b_aligned = axis->parallelLineOnAPoint(b_centroid);
    Line::Ptr mid_line_aligned = axis->parallelLineOnAPoint(c_centroid);
    //Line::Ptr axis_aligned = axis->parallelLineOnAPoint(c_centroid);
    pcl::PointCloud<PointT>::Ptr all_points(new pcl::PointCloud<PointT>);
    *all_points = *point_on_a + *point_on_b;
    *all_points = *all_points + *point_on_c;
    *points_on_edge = *all_points;
    
    // PointT a_centroid_p, b_centroid_p, c_centroid_p;
    // a_centroid_p.x = a_centroid[0];
    // a_centroid_p.y = a_centroid[1];
    // a_centroid_p.z = a_centroid[2];
    // b_centroid_p.x = b_centroid[0];
    // b_centroid_p.y = b_centroid[1];
    // b_centroid_p.z = b_centroid[2];
    // c_centroid_p.x = c_centroid[0];
    // c_centroid_p.y = c_centroid[1];
    // c_centroid_p.z = c_centroid[2];
    // points_on_edge->points.push_back(a_centroid_p);
    // points_on_edge->points.push_back(b_centroid_p);
    // points_on_edge->points.push_back(c_centroid_p);
      
    PointPair min_max_points = minMaxPointOnLine(*axis, all_points);
    PointT min_point, max_point;
    // min_point.x = min_max_points.get<0>()[0];
    // min_point.y = min_max_points.get<0>()[1];
    // min_point.z = min_max_points.get<0>()[2];
    // max_point.x = min_max_points.get<1>()[0];
    // max_point.y = min_max_points.get<1>()[1];
    // max_point.z = min_max_points.get<1>()[2];
    // points_on_edge->points.push_back(min_point);
    // points_on_edge->points.push_back(max_point);
    Eigen::Vector3f center_point
      = (min_max_points.get<0>() + min_max_points.get<1>()) / 2.0;
    double z_width = (min_max_points.get<0>() - min_max_points.get<1>()).norm();
    mid_line_aligned->getDirection(ez);
    mid_line_aligned->parallelLineNormal(*line_a_aligned, ex);
    mid_line_aligned->parallelLineNormal(*line_b_aligned, ey);
    
    double x_width = ex.norm();
    double y_width = ey.norm();
    
    ex.normalize();
    ey.normalize();
    ez.normalize();

    ROS_INFO("ex: [%f, %f, %f]", ex[0], ex[1], ex[2]);
    ROS_INFO("ey: [%f, %f, %f]", ey[0], ey[1], ey[2]);
    ROS_INFO("ez: [%f, %f, %f]", ez[0], ez[1], ez[2]);
    
    if (ex.cross(ey).dot(ez) < 0) {
      ez = -ez;
    }
    
    Eigen::Quaternionf rot = rotFrom3Axis(ex, ey, ez);
    std::vector<double> dimensions;
    dimensions.push_back(x_width);
    dimensions.push_back(y_width);
    dimensions.push_back(z_width);
    Cube::Ptr ret (new Cube(center_point, rot, dimensions));
    return ret;
  }
  
  void EdgebasedCubeFinder::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
    const ParallelEdgeArray::ConstPtr& input_edges)
  {
     boost::mutex::scoped_lock lock(mutex_);
     pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
     pcl::PointCloud<PointT>::Ptr all_filtered_cloud (new pcl::PointCloud<PointT>);
     pcl::fromROSMsg(*input_cloud, *cloud);
     visualization_msgs::Marker marker;
     jsk_pcl_ros::PolygonArray polygon_array;
     geometry_msgs::PoseArray pose_array;
     jsk_pcl_ros::BoundingBoxArray box_array;
     box_array.header = input_cloud->header;
     pose_array.header = input_cloud->header;
     std::vector<Cube::Ptr> cubes;
     std::vector<pcl::PointIndices::Ptr> candidate_cluster_indices;
     for (size_t i = 0; i < input_edges->edge_groups.size(); i++) {
       ParallelEdge parallel_edge = input_edges->edge_groups[i];
       std::vector<pcl::PointIndices::Ptr> edges
         = pcl_conversions::convertToPCLPointIndices(
           parallel_edge.cluster_indices);
       std::vector<pcl::ModelCoefficients::Ptr> coefficients
         = pcl_conversions::convertToPCLModelCoefficients(
           parallel_edge.coefficients);
       std::vector<IndicesCoefficientsTriple> triples
         = tripleIndicesAndCoefficients(edges, coefficients);
       std::vector<IndicesCoefficientsTriple> perpendicular_triples
         = filterPerpendicularEdgeTriples(triples);
       if (perpendicular_triples.size() > 0) {
         // buildup cube instance...
         pcl::PointCloud<PointT>::Ptr points_on_edges(new pcl::PointCloud<PointT>);
         pcl_conversions::toPCL(input_cloud->header, points_on_edges->header);
         for (size_t j = 0; j < perpendicular_triples.size(); j++) {
           pcl::PointCloud<PointT>::Ptr points_on_edge(new pcl::PointCloud<PointT>);
           Cube::Ptr cube = cubeFromIndicesAndCoefficients(
             cloud,
             perpendicular_triples[j],
             points_on_edge);
           *points_on_edges = *points_on_edges + *points_on_edge;
           cubes.push_back(cube);
         }
         pub_debug_filtered_cloud_.publish(points_on_edges);
       }
     }

     if (cubes.size() > 0) {
       for (size_t i = 0; i < cubes.size(); i++) {
         // publish cubes
         jsk_pcl_ros::BoundingBox ros_box = cubes[i]->toROSMsg();
         ros_box.header = input_cloud->header;
         box_array.boxes.push_back(ros_box);
         pose_array.poses.push_back(ros_box.pose);
       }
       pub_.publish(box_array);
       pub_pose_array_.publish(pose_array);
     }
  }

  bool EdgebasedCubeFinder::isPerpendicularVector(
    const Eigen::Vector3f& a,
    const Eigen::Vector3f& b)
  {
    double dot = a.normalized().dot(b.normalized());
    if (fabs(dot) >= 1.0) {
      return false;
    }
    else {
      double theta = fabs(acos(dot));
      NODELET_INFO("theta: %f", pcl::rad2deg(theta));
      if (fabs(theta - M_PI / 2.0) < pcl::deg2rad(20.0)) {
        return true;
      }
      else {
        return false;
      }
    }
  }
  
  EdgebasedCubeFinder::EdgeRelation EdgebasedCubeFinder::perpendicularEdgeTriple(
    const Line& edge_a,
    const Line& edge_b,
    const Line& edge_c)
  {
    Eigen::Vector3f a_b_normal, a_c_normal;
    edge_a.parallelLineNormal(edge_b, a_b_normal);
    edge_a.parallelLineNormal(edge_c, a_c_normal);
    if (isPerpendicularVector(a_b_normal, a_c_normal)) {
      return A_PERPENDICULAR;
    }
    else {
      Eigen::Vector3f b_a_normal, b_c_normal;
      edge_b.parallelLineNormal(edge_a, b_a_normal);
      edge_b.parallelLineNormal(edge_c, b_c_normal);
      if (isPerpendicularVector(b_a_normal, b_c_normal)) {
        return B_PERPENDICULAR;
      }
      else {
        Eigen::Vector3f c_a_normal, c_b_normal;
        edge_c.parallelLineNormal(edge_a, c_a_normal);
        edge_c.parallelLineNormal(edge_b, c_b_normal);
        if (isPerpendicularVector(c_a_normal, c_b_normal)) {
          return C_PERPENDICULAR;
        }
        else {
          return NOT_PERPENDICULAR;
        }
      }
    }
  }
  
  std::vector<IndicesCoefficientsTriple>
  EdgebasedCubeFinder::filterPerpendicularEdgeTriples(
    const std::vector<IndicesCoefficientsTriple>& triples)
  {
    std::vector<IndicesCoefficientsTriple> ret;
    for (size_t i = 0; i < triples.size(); i++) {
      pcl::ModelCoefficients::Ptr a_coefficients
        = triples[i].get<1>().get<0>();
      pcl::ModelCoefficients::Ptr b_coefficients
        = triples[i].get<1>().get<1>();
      pcl::ModelCoefficients::Ptr c_coefficients
        = triples[i].get<1>().get<2>();
      Line::Ptr edge_a
        = Line::fromCoefficients(a_coefficients->values);
      Line::Ptr edge_b
        = Line::fromCoefficients(b_coefficients->values);
      Line::Ptr edge_c
        = Line::fromCoefficients(c_coefficients->values);
      // check if these three are perpendicular or not
      EdgeRelation relation = perpendicularEdgeTriple(*edge_a,
                                                      *edge_b,
                                                      *edge_c);
      if (relation != NOT_PERPENDICULAR) {
        // rearrange
        if (relation == A_PERPENDICULAR) {
          ret.push_back(triples[i]);
        }
        else if (relation == B_PERPENDICULAR) {
          IndicesCoefficientsTriple new_triple
            = boost::make_tuple(
              boost::make_tuple(triples[i].get<0>().get<1>(),
                                triples[i].get<0>().get<0>(),
                                triples[i].get<0>().get<2>()),
              boost::make_tuple(
                          triples[i].get<1>().get<1>(),
                          triples[i].get<1>().get<0>(),
                          triples[i].get<1>().get<2>()));
          ret.push_back(new_triple);
        }
        else if (relation == C_PERPENDICULAR) {
          IndicesCoefficientsTriple new_triple
            = boost::make_tuple(
              boost::make_tuple(triples[i].get<0>().get<2>(),
                                triples[i].get<0>().get<0>(),
                                triples[i].get<0>().get<1>()),
              boost::make_tuple(
                          triples[i].get<1>().get<2>(),
                          triples[i].get<1>().get<0>(),
                          triples[i].get<1>().get<1>()));
          ret.push_back(new_triple);
        }
      }
    }
    return ret;
  }
  
  void EdgebasedCubeFinder::estimate2(
    const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
    const ParallelEdgeArray::ConstPtr& input_edges)
  {
     boost::mutex::scoped_lock lock(mutex_);
     pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
     pcl::PointCloud<PointT>::Ptr all_filtered_cloud (new pcl::PointCloud<PointT>);
     pcl::fromROSMsg(*input_cloud, *cloud);
     visualization_msgs::Marker marker;
     jsk_pcl_ros::PolygonArray polygon_array;
     std::vector<pcl::PointIndices::Ptr> candidate_cluster_indices;
     
     polygon_array.header = input_cloud->header;
     marker.pose.orientation.w = 1.0;
     marker.color.a = 1.0;
     marker.color.r = 1.0;
     marker.header = input_cloud->header;
     marker.scale.x = 0.01;
     marker.type = visualization_msgs::Marker::LINE_LIST;
     NODELET_INFO("%lu parallel edge groups", input_edges->edge_groups.size());
     geometry_msgs::PoseArray pose_array;
     pose_array.header = input_cloud->header;
     BoundingBoxArray ros_output;
     ros_output.header = input_cloud->header;

     for (size_t i = 0; i < input_edges->edge_groups.size(); i++) {
       ParallelEdge parallel_edge = input_edges->edge_groups[i];

       if (parallel_edge.cluster_indices.size() <= 1) {
         NODELET_ERROR("parallel edge group has only %lu edges",
                       parallel_edge.cluster_indices.size());
         continue;
       }
       NODELET_INFO("%lu parallel edge groups has %lu edges",
                    i, parallel_edge.cluster_indices.size());
       ////////////////////////////////////////////////////////
       // first convert all the pcl_msgs/PointIndices to pcl::PointIndices
       ////////////////////////////////////////////////////////
       std::vector<pcl::PointIndices::Ptr> edges
         = pcl_conversions::convertToPCLPointIndices(
           parallel_edge.cluster_indices);
       std::vector<pcl::ModelCoefficients::Ptr> coefficients
         = pcl_conversions::convertToPCLModelCoefficients(
           parallel_edge.coefficients);

       std::vector<IndicesPair> pairs = combinateIndices(edges);
       std::vector<CoefficientsPair> coefficients_pair
         = combinateCoefficients(coefficients);
       std::vector<IndicesPair> filtered_indices_pairs;
       std::vector<CoefficientsPair> filtered_coefficients_pairs;

       filtered_indices_pairs = pairs;
       filtered_coefficients_pairs = coefficients_pair;
       // filterPairsBasedOnParallelEdgeDistances(
       //   pairs, coefficients_pair,
       //   filtered_indices_pairs, filtered_coefficients_pairs);

       // convex based filtering...
       std::vector<ConvexPolygon::Ptr> convexes;
       for (size_t j = 0; j < filtered_coefficients_pairs.size(); j++) {
         ConvexPolygon::Ptr convex
           = convexFromPairs(cloud, filtered_coefficients_pairs[j],
                             pairs[j]);
         convexes.push_back(convex);
       }
       std::vector<int> filtered_indices;
       filterBasedOnConvex(cloud, convexes, filtered_indices);

       pcl::PointIndices::Ptr filtered_cube_candidate_indices(new pcl::PointIndices);
       for (size_t j = 0; j < filtered_indices.size(); j++) {
         int index = filtered_indices[j];
         ConvexPolygon::Ptr target_convex = convexes[index];
         IndicesPair target_edge_indices_pair
           = filtered_indices_pairs[index];
         CoefficientsPair target_edge_coefficients_pair
           = filtered_coefficients_pairs[index];
         // 1. roughly segment the points around the plane
         //pcl::PointCloud<PointT>::Ptr filtered_cloud
         pcl::PointIndices::Ptr filtered_indices
           = preparePointCloudForRANSAC(
             target_convex, target_edge_coefficients_pair, cloud);
         // *filtered_cube_candidate_indices
         //   = *filtered_indices + *filtered_cube_candidate_indices;
         filtered_cube_candidate_indices
           = addIndices(*filtered_cube_candidate_indices,
                        *filtered_indices);       
           
         // ROS_INFO("%lu -> %lu", cloud->points.size(), filtered_cloud->points.size());
         // *all_filtered_cloud = *all_filtered_cloud + *filtered_cloud;
         // // 2. estimate a plane parallel to the convex using RANSAC
         // pcl::ModelCoefficients::Ptr
         //   parallel_plane_coefficients (new pcl::ModelCoefficients);
         // pcl::PointIndices::Ptr
         //   parallel_plane_inliers (new pcl::PointIndices);

         // estimateParallelPlane(target_convex, filtered_cloud,
         //                       parallel_plane_inliers,
         //                       parallel_plane_coefficients);
         // if (parallel_plane_inliers->indices.size() > 0) {
         //   ConvexPolygon::Ptr parallel_convex
         //     = convexFromCoefficientsAndInliers<PointT>(
         //       filtered_cloud,
         //       parallel_plane_inliers,
         //       parallel_plane_coefficients);
         //   if (parallel_convex) {
         //     pcl::ModelCoefficients::Ptr
         //       perpendicular_plane_coefficients (new pcl::ModelCoefficients);
         //     pcl::PointIndices::Ptr
         //       perpendicular_plane_inliers (new pcl::PointIndices);
         //     estimatePerpendicularPlane(parallel_convex,
         //                                target_edge_coefficients_pair,
         //                                filtered_cloud,
         //                                perpendicular_plane_inliers,
         //                                perpendicular_plane_coefficients);
         //     if (perpendicular_plane_inliers->indices.size() > 0) {
         //      ConvexPolygon::Ptr perpendicular_convex
         //        = convexFromCoefficientsAndInliers<PointT>(
         //          filtered_cloud,
         //          perpendicular_plane_inliers,
         //          perpendicular_plane_coefficients);
         //      if (perpendicular_convex) {
         //        if (perpendicular_convex->angle(*parallel_convex) < pcl::deg2rad(10.0)) {
         //          geometry_msgs::PolygonStamped ros_polygon;
         //          ros_polygon.header = input_cloud->header;
         //          ros_polygon.polygon = parallel_convex->toROSMsg();
         //          polygon_array.polygons.push_back(ros_polygon);
         //          geometry_msgs::PolygonStamped ros_polygon2;
         //          ros_polygon2.header = input_cloud->header;
         //          ros_polygon2.polygon = perpendicular_convex->toROSMsg();
         //          polygon_array.polygons.push_back(ros_polygon2);
         //        }
         //      }
         //    }
         //   }
         // }
        // if (parallel_plane) {
        //   // 3. estimate a plane perpendicular to the convex using RANSAC
        //   pcl::ModelCoefficients::Ptr perpendicular_plane
        //     = estimatePerpendicularPlane(convex, filtered_cloud);
        //   if (perpendicular_plane) {
        //     // success to estimate, it's cube
        //     Cube::Ptr cube
        //       = makeupCubeResult();
        //     cubes.push_back(cube);
        //   }
        //   else {
        //     // failed to estimate perpendicular plane
        //     ROS_INFO("failed to estimate perpendicular plane");
        //   }
        // }
        // else {
        //   // failed to estimate parallel plane
        //   ROS_INFO("failed to estimate parallel plane");
        // }
//      }
      
      // for (size_t j = 0; j < filtered_indices.size(); j++) {
      //   int pair_index = filtered_indices[j];
      //   ConvexPolygon::Ptr convex = convexes[filtered_indices[j]];
        
      //   Vertices vs = convex->getVertices();
      //   for (size_t k = 0; k < vs.size(); k++) {
      //     Eigen::Vector3f A, B;
      //     Eigen::Vector3d Ad, Bd;
      //     A = vs[k];
      //     if (k + 1 != vs.size()) {
      //       B = vs[k + 1];
      //     }
      //     else {
      //       B = vs[0];
      //     }
      //     geometry_msgs::Point AP, BP;
      //     convertEigenVector(A, Ad);
      //     convertEigenVector(B, Bd);
      //     tf::pointEigenToMsg(Ad, AP);
      //     tf::pointEigenToMsg(Bd, BP);
      //     marker.points.push_back(AP);
      //     marker.points.push_back(BP);
      //     std_msgs::ColorRGBA green;
      //     green.a = 1.0; green.g = 1.0;
      //     marker.colors.push_back(green);
      //     marker.colors.push_back(green);
      //   }
       }
       candidate_cluster_indices.push_back(filtered_cube_candidate_indices);

       // estimate cube
       pcl::PointIndices::Ptr first_inliers(new pcl::PointIndices);
       pcl::ModelCoefficients::Ptr first_coefficients(new pcl::ModelCoefficients);
       ConvexPolygon::Ptr first_polygon
         = estimateConvexPolygon(
           cloud,
           filtered_cube_candidate_indices,
           first_coefficients,
           first_inliers);
       if (first_polygon) {
         geometry_msgs::PolygonStamped first_polygon_ros;
         first_polygon_ros.polygon = first_polygon->toROSMsg();
         first_polygon_ros.header = input_cloud->header;
         polygon_array.polygons.push_back(first_polygon_ros);
       }
      // std::vector<CubeHypothesis::Ptr> hypothesis_list;
      // NODELET_INFO("building hypothesis");
      // NODELET_INFO("pair size: %lu", pairs.size());
      // for (size_t j = 0; j < pairs.size(); j++) {
      //   NODELET_INFO("j -> %lu", j);
      //   IndicesPair pair = pairs[j];
      //   CoefficientsPair cpair = coefficients_pair[j];
      //   PlanarCubeHypothesis::Ptr
      //     planar_hypothesis (new PlanarCubeHypothesis(
      //                          pair, cpair, outlier_threshold_));
      //   DiagnoalCubeHypothesis::Ptr
      //     diagnoal_hypothesis (new DiagnoalCubeHypothesis(
      //                            pair, cpair, outlier_threshold_));
      //   //hypothesis_list.push_back(planar_hypothesis);
      //   hypothesis_list.push_back(diagnoal_hypothesis);
      // }
      // NODELET_INFO("estimating hypothesis");
      // for (size_t j = 0; j < hypothesis_list.size(); j++) {
      //   hypothesis_list[j]->estimate(*cloud);
      // }
      // // find max evaluated
      // NODELET_INFO("find max hypothesis");
      // CubeHypothesis::Ptr max_hypothesis;
      // double max_eval = -DBL_MAX;
      // for (size_t i = 0; i < hypothesis_list.size(); i++) {
      //   if (max_eval < hypothesis_list[i]->getValue()) {
      //     max_eval = hypothesis_list[i]->getValue();
      //     max_hypothesis = hypothesis_list[i];
      //   }
      // }
      // NODELET_INFO("convert to ros msg");
      // BoundingBox msg = max_hypothesis->getCube()->toROSMsg();
      // msg.header = input_cloud->header;
      // ros_output.boxes.push_back(msg);
      // pose_array.poses.push_back(msg.pose);
    }
    pub_.publish(ros_output);
    pub_pose_array_.publish(pose_array);
    pub_debug_marker_.publish(marker);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*all_filtered_cloud, ros_cloud);
    ros_cloud.header = input_cloud->header;
    pub_debug_filtered_cloud_.publish(ros_cloud);
    pub_debug_polygons_.publish(polygon_array);

    // convert std::vector<pcl::PointIndices::Ptr> to ClusterPointIndices
    ClusterPointIndices ros_cluster_indices;
    ros_cluster_indices.header = input_cloud->header;
    for (size_t i = 0; i < candidate_cluster_indices.size(); i++) {
      PCLIndicesMsg indices_msg;
      indices_msg.header = input_cloud->header;
      indices_msg.indices = candidate_cluster_indices[i]->indices;
      ros_cluster_indices.cluster_indices.push_back(indices_msg);
    }
    pub_debug_clusers_.publish(ros_cluster_indices);
  }

  std::vector<IndicesCoefficientsTriple>
  EdgebasedCubeFinder::tripleIndicesAndCoefficients(
    const std::vector<pcl::PointIndices::Ptr>& indices,
    const std::vector<pcl::ModelCoefficients::Ptr>& coefficients)
  {
    if (indices.size() != coefficients.size()) {
      NODELET_ERROR("size of indices and coefficients are not same");
      return std::vector<IndicesCoefficientsTriple>();
    }

    if (indices.size() <= 2 && coefficients.size() <= 2) {
      NODELET_WARN("[EdgebasedCubeFinder::tripleIndicesAndCoefficients] no enough canddiates");
      return std::vector<IndicesCoefficientsTriple>();
    }
    std::vector<IndicesCoefficientsTriple> ret;
    for (size_t i = 0; i < indices.size() - 2; i++) {
      for (size_t j = i + 1; j < indices.size() - 1; j++) {
        for (size_t k = j + 1; k < indices.size(); k++) {
          IndicesTriple indices_triple
            = boost::make_tuple(indices[i],
                                indices[j],
                                indices[k]);
          CoefficientsTriple coefficients_triple
            = boost::make_tuple(coefficients[i],
                                coefficients[j],
                                coefficients[k]);
          IndicesCoefficientsTriple indices_coefficients_triple
            = boost::make_tuple(indices_triple,
                                coefficients_triple);
          ret.push_back(indices_coefficients_triple);
        }
      }
    }
    return ret;
  }
  
  ConvexPolygon::Ptr EdgebasedCubeFinder::estimateConvexPolygon(
    const pcl::PointCloud<PointT>::Ptr cloud,
    const pcl::PointIndices::Ptr indices,
    pcl::ModelCoefficients::Ptr coefficients,
    pcl::PointIndices::Ptr inliers)
  {
    ////////////////////////////////////////////////////////
    // RANSAC
    ////////////////////////////////////////////////////////
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.setDistanceThreshold(0.003);
    seg.segment(*inliers, *coefficients);
    ////////////////////////////////////////////////////////
    // project points to the plane
    ////////////////////////////////////////////////////////
    if (inliers->indices.size() > 0) {
      return convexFromCoefficientsAndInliers<PointT>(
        cloud, inliers, coefficients);
    }
    else {
      return ConvexPolygon::Ptr();
    }
  }
  
  std::vector<IndicesPair> EdgebasedCubeFinder::combinateIndices(
    const std::vector<pcl::PointIndices::Ptr>& indices)
  {
    std::vector<IndicesPair> ret;
    for(size_t i = 0; i < indices.size() - 1; i++) {
      for (size_t j = i + 1; j < indices.size(); j++) {
        IndicesPair pair = boost::make_tuple(indices[i],
                                             indices[j]);
        ret.push_back(pair);
      }
    }
    return ret;
  }

  std::vector<CoefficientsPair> EdgebasedCubeFinder::combinateCoefficients(
    const std::vector<pcl::ModelCoefficients::Ptr>& coefficients)
  {
    std::vector<CoefficientsPair> ret;
    for(size_t i = 0; i < coefficients.size() - 1; i++) {
      for (size_t j = i + 1; j < coefficients.size(); j++) {
        CoefficientsPair pair = boost::make_tuple(coefficients[i],
                                                  coefficients[j]);
        ret.push_back(pair);
      }
    }
    return ret;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EdgebasedCubeFinder, nodelet::Nodelet);

