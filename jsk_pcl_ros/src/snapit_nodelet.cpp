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
  
  bool SnapIt::snapitCallback(jsk_pcl_ros::CallSnapIt::Request& req,
                              jsk_pcl_ros::CallSnapIt::Response& res)
  {
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
    sensor_msgs::PointCloud2::Ptr debug_cloud(new sensor_msgs::PointCloud2);
    // publish for debug
    pcl::toROSMsg(*points_inside_pole, *debug_cloud);
    debug_cloud->header = input_header_;
    debug_candidate_points_pub_.publish(debug_cloud);

    // estimate plane
    
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud(points_inside_pole);
    seg.segment (*plane_inliers, *plane_coefficients);

    if (plane_inliers->indices.size () == 0)
    {
      NODELET_ERROR ("Could not estimate a planar model for the given dataset.");
      return false;
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
    float alpha = - plane_coefficients->values[3] - n_prime.dot(C);
    
    //Eigen::Vector3f C_new = C + alpha * n_prime;
    Eigen::Vector3f C_new = alpha * n_prime + C;

    Eigen::Affine3f A = Eigen::Translation3f(C_new - C) * trans;
    Eigen::Quaternionf final_rot(A.rotation());
    Eigen::Translation3f final_trans(A.translation());
    res.transformation.orientation.x = final_rot.x();
    res.transformation.orientation.y = final_rot.y();
    res.transformation.orientation.z = final_rot.z();
    res.transformation.orientation.w = final_rot.w();
    
    res.transformation.position.x = final_trans.x();
    res.transformation.position.y = final_trans.y();
    res.transformation.position.z = final_trans.z();

    
    if (isnan(alpha)) {
      NODELET_ERROR("alpha == nan");
      return false;
    }
    return true;
  }
  
  
}
typedef jsk_pcl_ros::SnapIt SnapIt;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, SnapIt, SnapIt, nodelet::Nodelet);
