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

#include "jsk_pcl_ros/hinted_plane_detector.h"
#include "pcl_ros/transforms.h"
#include <visualization_msgs/Marker.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros {
  HintedPlaneDetector::HintedPlaneDetector() {
  }

  HintedPlaneDetector::~HintedPlaneDetector() {
  }

  void HintedPlaneDetector::onInit() {
    PCLNodelet::onInit();

    input_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    tf_listener_.reset(new tf::TransformListener);
    marker_pub_ = pnh_->advertise<visualization_msgs::Marker>("marker", 10);
    debug_hint_centroid_pub_ = pnh_->advertise<geometry_msgs::PointStamped>("debug_hint_centroid", 10);
    debug_plane_points_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("debug_plane_points", 10);
    sub_input_ = pnh_->subscribe("input", 1, &HintedPlaneDetector::inputCallback, this);
    sub_hint_ = pnh_->subscribe("hint", 1, &HintedPlaneDetector::hintCallback, this);
    
  }

  void HintedPlaneDetector::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *input_);
    input_header_ = msg->header;
  }

  void HintedPlaneDetector::hintCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr hint_cloud_org (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hint_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fromROSMsg(*msg, *hint_cloud_org);
    // transform the pointcloud to input_header_
    
    tf::StampedTransform transform;
    tf_listener_->lookupTransform(input_header_.frame_id, msg->header.frame_id, msg->header.stamp, transform);
    pcl_ros::transformPointCloud(*hint_cloud_org, *hint_cloud, transform);
    
    // extract 3 points from hint_cloud
    if (hint_cloud->points.size() < 3) {
      NODELET_ERROR("too small the number of the points in hint_cloud");
      return;
    }

    pcl::PointXYZ A = hint_cloud->points[0];
    pcl::PointXYZ B = hint_cloud->points[hint_cloud->points.size() - 1];
    pcl::PointXYZ O = hint_cloud->points[hint_cloud->points.size() / 2];

    Eigen::Vector4f hint_centroid;
    pcl::compute3DCentroid(*hint_cloud, hint_centroid);

    geometry_msgs::PointStamped point;
    point.point.x = hint_centroid[0];
    point.point.y = hint_centroid[1];
    point.point.z = hint_centroid[2];
    point.header = input_header_;
    debug_hint_centroid_pub_.publish(point);
    
    // compute normal
    Eigen::Vector3f n = (B.getVector3fMap() - O.getVector3fMap()).cross(A.getVector3fMap() - O.getVector3fMap()).normalized();

    // Ax + By + Cz + D = 0
    // (x - O).dot(n) = 0;
    // D = - O.dot(n)
    // (A, B, C) = n
    // d = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)
    double D = - O.getVector3fMap().dot(n);
    const double thr = 0.05;
    pcl::PointIndices::Ptr candidate_inlier(new pcl::PointIndices);
    for (size_t i = 0; i < input_->points.size(); i++) {
      double d = fabs(n.dot(input_->points[i].getVector3fMap()) + D);
      if (d < thr) {
        candidate_inlier->indices.push_back(i);
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_);
    extract.setIndices(candidate_inlier);
    extract.filter(*candidate_points);
    
    // plane detection
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud(candidate_points);
    seg.setAxis(n);
    seg.setEpsAngle(0.1);
    seg.segment (*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (candidate_points);
    proj.setModelCoefficients (coefficients);
    proj.filter (*plane_points);

    sensor_msgs::PointCloud2::Ptr debug_ros_plane_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*plane_points, *debug_ros_plane_cloud);
    debug_ros_plane_cloud->header = input_header_;
    debug_plane_points_pub_.publish(debug_ros_plane_cloud);
    
    // conduct euclidean segmentation to remove planes far away
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (plane_points);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (640 * 480 * 1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (plane_points);
    ec.extract (cluster_indices);

    if (cluster_indices.size() == 0) {
      NODELET_ERROR("failed to segment points");
      return;
    }
    else {
      NODELET_INFO("%lu clusters", cluster_indices.size());
    }
    double min_distance = DBL_MAX;
    pcl::PointCloud<pcl::PointXYZ>::Ptr min_distance_cloud;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      // compute cog
      Eigen::Vector4f c;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices indices = cluster_indices[i];
      for (size_t j = 0; j < indices.indices.size(); j++) {
        cloud->points.push_back(plane_points->points[indices.indices[j]]);
      }
      pcl::compute3DCentroid(*cloud, c);
      double d = (c - hint_centroid).norm();
      if (d < min_distance) {
        NODELET_INFO_STREAM(i << " is min");
        min_distance = d;
        min_distance_cloud = cloud;
      }
      // sensor_msgs::PointCloud2::Ptr debug_ros_plane_cloud(new sensor_msgs::PointCloud2);
      // pcl::toROSMsg(*cloud, *debug_ros_plane_cloud);
      // debug_ros_plane_cloud->header = input_header_;
      // debug_plane_points_pub_.publish(debug_ros_plane_cloud);
      // ros::Duration(1.0).sleep();
    }

    NODELET_INFO("convexhull");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(min_distance_cloud);
    chull.setDimension(2);
    chull.setAlpha (0.1);
    
    chull.reconstruct (*cloud_hull);

    if (cloud_hull->points.size() < 3) {
      NODELET_ERROR("failed to estimate convex hull");
      return;
    }
    NODELET_INFO("convexhull done");
    
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
    
    NODELET_INFO("done");
  }
  
}

typedef jsk_pcl_ros::HintedPlaneDetector HintedPlaneDetector;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, HintedPlaneDetector, HintedPlaneDetector, nodelet::Nodelet);
