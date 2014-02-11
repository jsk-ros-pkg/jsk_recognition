/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "jsk_pcl_ros/snapit.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <Eigen/StdVector>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>

namespace jsk_pcl_ros
{
  SnapIt::SnapIt() {};
  SnapIt::~SnapIt() {};

  void SnapIt::onInit(void)
  {
    PCLNodelet::onInit();
    
    tf_listener_.reset(new tf::TransformListener);
    input_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    debug_candidate_points_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("debug_candidate_points", 10);
    debug_candidate_points_pub2_ = pnh_->advertise<sensor_msgs::PointCloud2>("debug_candidate_points2", 10);
    debug_candidate_points_pub3_ = pnh_->advertise<sensor_msgs::PointCloud2>("debug_model_points", 10);
    debug_centroid_pub_ = pnh_->advertise<geometry_msgs::PointStamped>("debug_centroid", 10);
    marker_pub_ = pnh_->advertise<visualization_msgs::Marker>("snapit_marker", 10);
    debug_centroid_after_trans_pub_ = pnh_->advertise<geometry_msgs::PointStamped>("debug_transformed_centroid", 10);
    sub_input_ = pnh_->subscribe("input", 1, &SnapIt::inputCallback, this);
    call_snapit_srv_ = pnh_->advertiseService("snapit", &SnapIt::snapitCallback,
                                              this);
  }

  void SnapIt::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::fromROSMsg(*input, *input_);
    input_frame_id_ = input->header.frame_id;
    input_header_ = input->header;
  }

  bool SnapIt::checkPointInsidePlane(EigenVector3fVector &plane_points,
                                     Eigen::Vector3f normal,
                                     Eigen::Vector3f point)
  {
    if (isnan(point[0]) || isnan(point[1]) || isnan(point[2])) {
      return false;
    }
    for (size_t i = 0; i < plane_points.size(); i++) {
      Eigen::Vector3f B;
      Eigen::Vector3f O = plane_points[i];

      if (i == (plane_points.size() - 1)) {
        B = plane_points[0];
      }
      else {
        B = plane_points[i + 1];
      }
      Eigen::Vector3f OB = B - O;
      Eigen::Vector3f OP = point - O;
      if ((OB.cross(OP)).dot(normal) < 0) {
        return false;
      }
    }

    return true;
  }
  
  bool SnapIt::extractPointsInsidePlanePole(geometry_msgs::PolygonStamped target_plane,
                                            pcl::PointIndices::Ptr inliers,
                                            EigenVector3fVector& points,
                                            Eigen::Vector3f &n,
                                            Eigen::Vector3f &p)
  {
    for (size_t i = 0; i < target_plane.polygon.points.size(); i++) {
      geometry_msgs::PointStamped point_stamped, transformed_point_stamped;
      point_stamped.header = target_plane.header;
      point_stamped.point.x = target_plane.polygon.points[i].x;
      point_stamped.point.y = target_plane.polygon.points[i].y;
      point_stamped.point.z = target_plane.polygon.points[i].z;
      tf_listener_->transformPoint(input_frame_id_, point_stamped, transformed_point_stamped);
      Eigen::Vector3f eigen_points;
      eigen_points[0] = transformed_point_stamped.point.x;
      eigen_points[1] = transformed_point_stamped.point.y;
      eigen_points[2] = transformed_point_stamped.point.z;
      points.push_back(eigen_points);
    }
    
    // create model
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    
    // extract independent 3 points
    Eigen::Vector3f O = points[0];
    Eigen::Vector3f A = points[1];
    for (size_t i = 2; i < points.size(); i++) {
      Eigen::Vector3f B = points[i];
      // check O, A, B on the sampe line or not...
      // OA x OB
      Eigen::Vector3f result = (A - O).cross(B - O);
      if (result.norm() < 1.0e-08) {
        // too small
        continue;
      }
      else {
        result.normalize();
        // big enough
        coefficients->values.resize (4);
        // A(x - ox) + B(y - oy) + C(z - oz) = 0
        // result = (A, B, C)
        coefficients->values[0] = result[0];
        coefficients->values[1] = result[1];
        coefficients->values[2] = result[2];
        coefficients->values[3] = O.dot(result);
        n = result;
        p = O;
      }
    }
    if (coefficients->values.size() != 4) {
      NODELET_ERROR("all the points of target_plane are on a line");
      return false;
    }

    // project input_ to the plane
    pcl::PointCloud<pcl::PointXYZ> cloud_projected;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (input_);
    proj.setModelCoefficients (coefficients);
    proj.filter (cloud_projected);

    
    // next, check the points inside of the plane or not...
    for (size_t i = 0; i < cloud_projected.points.size(); i++) {
      pcl::PointXYZ point = cloud_projected.points[i];
      Eigen::Vector3f point_vector = point.getVector3fMap();
      if (checkPointInsidePlane(points, n, point_vector)) {
        inliers->indices.push_back(i);
      }
    }
    //ROS_INFO("%lu points inside the plane", inliers->indices.size());
    return true;
  }

  void SnapIt::publishConvexHullMarker(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull) {
    // publish marker
    visualization_msgs::Marker marker;
    marker.header = input_header_;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    for (size_t i = 0; i < cloud_hull->points.size(); i++) {
      geometry_msgs::Point point;
      point.x = cloud_hull->points[i].x;
      point.y = cloud_hull->points[i].y;
      point.z = cloud_hull->points[i].z;
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points[0]);
    marker.pose.orientation.w = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.01;      // 1cm width
    marker_pub_.publish(marker);
  }
  
  void SnapIt::extractPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                  pcl::PointIndices::Ptr out_inliers,
                                  pcl::ModelCoefficients::Ptr out_coefficients,
                                  Eigen::Vector3f normal,
                                  double eps_angle) {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud(input);
    seg.setAxis(normal);
    if (eps_angle != 0.0) {
      seg.setEpsAngle(eps_angle);
    }
    else {
      seg.setEpsAngle(0.2);
    }
    seg.segment (*out_inliers, *out_coefficients);
  }
  
  bool SnapIt::processModelPlane(jsk_pcl_ros::CallSnapIt::Request& req,
                                 jsk_pcl_ros::CallSnapIt::Response& res) {
    // first build plane model
    geometry_msgs::PolygonStamped target_plane = req.request.target_plane;
    // check the size of the points
    if (target_plane.polygon.points.size() < 3) {
      NODELET_ERROR("not enough points included in target_plane");
      return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_inside_pole (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    EigenVector3fVector points;
    Eigen::Vector3f n, p;
    if (!extractPointsInsidePlanePole(target_plane, inliers, points, n, p)) {
      return false;
    }

    if (inliers->indices.size() < 3) {
      NODELET_ERROR("not enough points inside of the target_plane");
      return false;
    }
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_);
    extract.setIndices(inliers);
    extract.filter(*points_inside_pole);

    publishPointCloud(debug_candidate_points_pub_, points_inside_pole);
    
    // estimate plane
    
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
    extractPlanePoints(points_inside_pole, plane_inliers, plane_coefficients,
                       n, req.request.eps_angle);

    if (plane_inliers->indices.size () == 0)
    {
      NODELET_ERROR ("Could not estimate a planar model for the given dataset.");
      return false;
    }

    // extract plane points
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (plane_inliers);
    proj.setInputCloud (points_inside_pole);
    proj.setModelCoefficients (plane_coefficients);
    proj.filter (*plane_points);
    publishPointCloud(debug_candidate_points_pub3_, plane_points);
    
    // next, compute convexhull and centroid
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (plane_points);
    chull.setDimension(2);
    chull.setAlpha (0.1);
    
    chull.reconstruct (*cloud_hull);

    if (cloud_hull->points.size() < 3) {
      NODELET_ERROR("failed to estimate convex hull");
      return false;
    }
    publishConvexHullMarker(cloud_hull);
    
    Eigen::Vector4f C_new_4f;
    pcl::compute3DCentroid(*cloud_hull, C_new_4f);
    Eigen::Vector3f C_new;
    for (size_t i = 0; i < 3; i++) {
      C_new[i] = C_new_4f[i];
    }
    
    Eigen::Vector3f n_prime;
    n_prime[0] = plane_coefficients->values[0];
    n_prime[1] = plane_coefficients->values[1];
    n_prime[2] = plane_coefficients->values[2];
    plane_coefficients->values[3] = plane_coefficients->values[3] / n_prime.norm();
    n_prime.normalize();
    
    if (n_prime.dot(n) < 0) {
      n_prime = - n_prime;
      plane_coefficients->values[3] = - plane_coefficients->values[3];
    }
    
    Eigen::Vector3f n_cross = n.cross(n_prime);
    double theta = asin(n_cross.norm());

    Eigen::Quaternionf trans (Eigen::AngleAxisf(theta, n_cross.normalized()));
    
    // compute C
    Eigen::Vector3f C_orig(0, 0, 0);
    for (size_t i = 0; i < points.size(); i++) {
      C_orig = C_orig + points[i];
    }
    C_orig = C_orig / points.size();
    // compute C
    Eigen::Vector3f C = trans * C_orig;
    

    Eigen::Affine3f A = Eigen::Translation3f(C) * Eigen::Translation3f(C_new - C) * trans * Eigen::Translation3f(C_orig).inverse();
    tf::poseEigenToMsg((Eigen::Affine3d)A, res.transformation);

    geometry_msgs::PointStamped centroid;
    centroid.point.x = C_orig[0];
    centroid.point.y = C_orig[1];
    centroid.point.z = C_orig[2];
    centroid.header = target_plane.header;
    debug_centroid_pub_.publish(centroid);

    geometry_msgs::PointStamped centroid_transformed;
    centroid_transformed.point.x = C_new[0];
    centroid_transformed.point.y = C_new[1];
    centroid_transformed.point.z = C_new[2];
    centroid_transformed.header = target_plane.header;
    debug_centroid_after_trans_pub_.publish(centroid_transformed);
    
    return true;
  }

  double SnapIt::distanceAlongWithLine(const Eigen::Vector4f& point, const Eigen::Vector4f& center, const Eigen::Vector4f direction) {
    return fabs((point - center).dot(direction));
  }
  
  bool SnapIt::extractPointsInsideCylinder(const geometry_msgs::PointStamped& center, const geometry_msgs::Vector3Stamped direction,
                                           const double radius,
                                           const double height,
                                           pcl::PointIndices::Ptr inliers,
                                           Eigen::Vector3f &n,
                                           Eigen::Vector3f &C_orig,
                                           const double fat_factor) {
    // resolve tf
    geometry_msgs::PointStamped transformed_center;
    geometry_msgs::Vector3Stamped transformed_direction;
    tf_listener_->transformPoint(input_frame_id_, center, transformed_center);
    tf_listener_->transformVector(input_frame_id_, direction, transformed_direction);
    Eigen::Vector4f center_eigen, direction_eigen;
    center_eigen[0] = transformed_center.point.x;
    center_eigen[1] = transformed_center.point.y;
    center_eigen[2] = transformed_center.point.z;
    for (size_t i = 0; i < 3; i++) {
      C_orig[i] = center_eigen[i];
    }
    direction_eigen[0] = transformed_direction.vector.x;
    direction_eigen[1] = transformed_direction.vector.y;
    direction_eigen[2] = transformed_direction.vector.z;
    double fat_radius = radius * fat_factor;
    for (size_t i = 0; i < input_->points.size(); i++) {
      double distance = pcl::sqrPointToLineDistance (input_->points[i].getVector4fMap(), center_eigen, direction_eigen);
      if (sqrt(distance) < fat_radius * 3.0) {
        // compute the distance toward z-direction
        if (distanceAlongWithLine(input_->points[i].getVector4fMap(), center_eigen, direction_eigen) < height * fat_factor / 2.0) {
          inliers->indices.push_back(i);
        }
      }
    }
    for (size_t i = 0; i < 3; i++) {
      n[i] = direction_eigen[i];
    }
    
    return true;
  }

  void SnapIt::publishPointCloud(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud, *ros_cloud);
    pub.publish(ros_cloud);
  }
  
  bool SnapIt::processModelCylinder(jsk_pcl_ros::CallSnapIt::Request& req,
                                    jsk_pcl_ros::CallSnapIt::Response& res) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    Eigen::Vector3f n, C_orig;
    if (!extractPointsInsideCylinder(req.request.center,
                                     req.request.direction,
                                     req.request.radius,
                                     req.request.height,
                                     inliers, n, C_orig,
                                     1.3)) {
      return false;
    }
    if (inliers->indices.size() < 3) {
      NODELET_ERROR("not enough points inside of the target_plane");
      return false;
    }
    
    geometry_msgs::PointStamped centroid;
    centroid.point.x = C_orig[0];
    centroid.point.y = C_orig[1];
    centroid.point.z = C_orig[2];
    centroid.header = req.request.header;
    
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_);
    extract.setIndices(inliers);
    extract.filter(*candidate_points);
    
    publishPointCloud(debug_candidate_points_pub_, candidate_points);


    // first, to remove plane we estimate the plane
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
    extractPlanePoints(candidate_points, plane_inliers, plane_coefficients,
                       n, req.request.eps_angle);
    if (plane_inliers->indices.size() == 0) {
      NODELET_ERROR ("plane estimation failed");
      return false;
    }

    // remove the points blonging to the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_inside_pole_wo_plane (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud (candidate_points);
    extract.setIndices (plane_inliers);
    extract.setNegative (true);
    extract.filter (*points_inside_pole_wo_plane);

    publishPointCloud(debug_candidate_points_pub2_, points_inside_pole_wo_plane);
    
    // normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setInputCloud (points_inside_pole_wo_plane);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
    
    
    // segmentation
    pcl::ModelCoefficients::Ptr cylinder_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr cylinder_inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setRadiusLimits (0.01, req.request.radius * 1.2);
    seg.setDistanceThreshold (0.05);
    
    seg.setInputCloud(points_inside_pole_wo_plane);
    seg.setInputNormals (cloud_normals);
    seg.setMaxIterations (10000);
    seg.setNormalDistanceWeight (0.1);
    seg.setAxis(n);
    if (req.request.eps_angle != 0.0) {
      seg.setEpsAngle(req.request.eps_angle);
    }
    else {
      seg.setEpsAngle(0.35);
    }
    seg.segment (*cylinder_inliers, *cylinder_coefficients);
    if (cylinder_inliers->indices.size () == 0)
    {
      NODELET_ERROR ("Could not estimate a cylinder model for the given dataset.");
      return false;
    }

    debug_centroid_pub_.publish(centroid);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud (points_inside_pole_wo_plane);
    extract.setIndices (cylinder_inliers);
    extract.setNegative (false);
    extract.filter (*cylinder_points);

    publishPointCloud(debug_candidate_points_pub3_, cylinder_points);

    Eigen::Vector3f n_prime;
    Eigen::Vector3f C_new;
    for (size_t i = 0; i < 3; i++) {
      C_new[i] = cylinder_coefficients->values[i];
      n_prime[i] = cylinder_coefficients->values[i + 3];
    }

    double radius = fabs(cylinder_coefficients->values[6]);
    // inorder to compute centroid, we project all the points to the center line.
    // and after that, get the minimum and maximum points in the coordinate system of the center line
    double min_alpha = DBL_MAX;
    double max_alpha = -DBL_MAX;
    for (size_t i = 0; i < cylinder_points->points.size(); i++ ) {
      pcl::PointXYZ q = cylinder_points->points[i];
      double alpha = (q.getVector3fMap() - C_new).dot(n_prime);
      if (alpha < min_alpha) {
        min_alpha = alpha;
      }
      if (alpha > max_alpha) {
        max_alpha = alpha;
      }
    }
    // the center of cylinder
    Eigen::Vector3f C_new_prime = C_new + (max_alpha + min_alpha) / 2.0 * n_prime;
    
    Eigen::Vector3f n_cross = n.cross(n_prime);
    if (n.dot(n_prime)) {
      n_cross = - n_cross;
    }
    double theta = asin(n_cross.norm());
    Eigen::Quaternionf trans (Eigen::AngleAxisf(theta, n_cross.normalized()));
    Eigen::Vector3f C = trans * C_orig;
    Eigen::Affine3f A = Eigen::Translation3f(C) * Eigen::Translation3f(C_new_prime - C) * trans * Eigen::Translation3f(C_orig).inverse();
    tf::poseEigenToMsg((Eigen::Affine3d)A, res.transformation);

    geometry_msgs::PointStamped centroid_transformed;
    centroid_transformed.point.x = C_new_prime[0];
    centroid_transformed.point.y = C_new_prime[1];
    centroid_transformed.point.z = C_new_prime[2];
    centroid_transformed.header = req.request.header;
    debug_centroid_after_trans_pub_.publish(centroid_transformed);

    // publish marker
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = (max_alpha - min_alpha);
    marker.pose.position.x = C_new_prime[0];
    marker.pose.position.y = C_new_prime[1];
    marker.pose.position.z = C_new_prime[2];

    // n_prime -> z
    // n_cross.normalized() -> x
    Eigen::Vector3f z_axis = n_prime.normalized();
    Eigen::Vector3f y_axis = n_cross.normalized();
    Eigen::Vector3f x_axis = (y_axis.cross(z_axis)).normalized();
    Eigen::Matrix3f M;
    for (size_t i = 0; i < 3; i++) {
      M(i, 0) = x_axis[i];
      M(i, 1) = y_axis[i];
      M(i, 2) = z_axis[i];
    }
    
    Eigen::Quaternionf q (M);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.header = input_header_;
    marker_pub_.publish(marker);
    
    return true;
  }
  
  
  
  bool SnapIt::snapitCallback(jsk_pcl_ros::CallSnapIt::Request& req,
                              jsk_pcl_ros::CallSnapIt::Response& res)
  {
    switch (req.request.model_type) {
    case jsk_pcl_ros::SnapItRequest::MODEL_PLANE:
      return processModelPlane(req, res);
    case jsk_pcl_ros::SnapItRequest::MODEL_CYLINDER:
      return processModelCylinder(req, res);
    default:
      ROS_FATAL_STREAM("unknown model_type: " << req.request.model_type);
      return false;
    }
  }
  
  
}
typedef jsk_pcl_ros::SnapIt SnapIt;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, SnapIt, SnapIt, nodelet::Nodelet);
