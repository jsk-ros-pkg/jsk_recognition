// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/organized_multi_plane_segmentation.h"
#include "jsk_pcl_ros/PolygonArray.h"
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <set>
#include<Eigen/StdVector>

#include <pluginlib/class_list_macros.h>

#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
#endif


namespace jsk_pcl_ros
{

  void OrganizedMultiPlaneSegmentation::onInit()
  {
    PCLNodelet::onInit();
    pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>("output", 1);
    polygon_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_polygon", 1);
    org_pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>("output_nonconnected", 1);
    org_polygon_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_nonconnected_polygon", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OrganizedMultiPlaneSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);

    sub_ = pnh_->subscribe("input", 1, &OrganizedMultiPlaneSegmentation::segment, this);
  }

  void OrganizedMultiPlaneSegmentation::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    if (min_size_ != config.min_size) {
      min_size_ = config.min_size;
    }
    if (angular_threshold_ != config.angular_threshold) {
      angular_threshold_ = config.angular_threshold;
    }
    if (distance_threshold_ != config.distance_threshold) {
      distance_threshold_ = config.distance_threshold;
    }
    if (max_curvature_ != config.max_curvature) {
      max_curvature_ = config.max_curvature;
    }
    if (connect_plane_angle_threshold_ != config.connect_plane_angle_threshold) {
      connect_plane_angle_threshold_ = config.connect_plane_angle_threshold;
    }
    if (connect_plane_distance_threshold_ != config.connect_plane_distance_threshold) {
      connect_plane_distance_threshold_ = config.connect_plane_distance_threshold;
    }
    if (connect_distance_threshold_ != config.connect_distance_threshold) {
      connect_distance_threshold_ = config.connect_distance_threshold;
    }
  }
  
  void OrganizedMultiPlaneSegmentation::segment
  (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    pcl::PointCloud<pcl::PointNormal> input;
    pcl::fromROSMsg(*msg, input);
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointNormal, pcl::Label> mps;
    mps.setMinInliers(min_size_);
    mps.setAngularThreshold(angular_threshold_);
    mps.setDistanceThreshold(distance_threshold_);
    mps.setMaximumCurvature(max_curvature_);
    mps.setInputCloud(input.makeShared());
    mps.setInputNormals(input.makeShared());

    std::vector<pcl::PlanarRegion<pcl::PointNormal>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointNormal> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    
    mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    {
      jsk_pcl_ros::ClusterPointIndices indices;
      jsk_pcl_ros::PolygonArray polygon_array;
      indices.header = msg->header;
      polygon_array.header = msg->header;
      for (size_t i = 0; i < inlier_indices.size(); i++) {
        pcl::PointIndices inlier = inlier_indices[i];
        PCLIndicesMsg one_indices;
        one_indices.indices = inlier.indices;
        one_indices.header = msg->header;
        indices.cluster_indices.push_back(one_indices);
        geometry_msgs::PolygonStamped polygon;
        polygon.header = msg->header;
        for (size_t j = 0; j < boundary_indices[i].indices.size(); j++) {
          geometry_msgs::Point32 point;
          point.x = input.points[boundary_indices[i].indices[j]].x;
          point.y = input.points[boundary_indices[i].indices[j]].y;
          point.z = input.points[boundary_indices[i].indices[j]].z;
          polygon.polygon.points.push_back(point);
        }
        polygon_array.polygons.push_back(polygon);
      }
      org_pub_.publish(indices);
      org_polygon_pub_.publish(polygon_array);
      if (regions.size() == 0) {
        pub_.publish(indices);
        return;
      }
    }

    // connection
    // this might be slow...
    ros::Time before_connect_time = ros::Time::now();
    NODELET_DEBUG("checking %lu connection", regions.size());
    pcl::ExtractIndices<pcl::PointNormal> extract;
    extract.setInputCloud(input.makeShared());
    std::vector<std::map<size_t, bool> > connection_map;
    connection_map.resize(regions.size());
    for (size_t i = 0; i < regions.size() - 1; i++) {
      for (size_t j = i + 1; j < regions.size(); j++) {
        // check if i and j can be connected
        pcl::ModelCoefficients a_coefficient = model_coefficients[i];
        pcl::ModelCoefficients b_coefficient = model_coefficients[j];
        Eigen::Vector3f a_normal(a_coefficient.values[0], a_coefficient.values[1], a_coefficient.values[2]);
        Eigen::Vector3f b_normal(b_coefficient.values[0], b_coefficient.values[1], b_coefficient.values[2]);
        double a_distance = a_coefficient.values[3];
        double b_distance = b_coefficient.values[3];
        // force to check the coefficients is normalized
        if (a_normal.norm() != 1.0) {
          a_distance = a_distance / a_normal.norm();
          a_normal = a_normal / a_normal.norm();
        }
        if (b_normal.norm() != 1.0) {
          b_distance = b_distance / b_normal.norm();
          b_normal = b_normal / b_normal.norm();
        }
        NODELET_DEBUG("%lu - %lu distance: %f", i, j, fabs(fabs(a_distance) - fabs(b_distance)));
        if (fabs(fabs(a_distance) - fabs(b_distance)) > connect_plane_distance_threshold_) {
          continue;
        }
        double theta = fabs(acos(a_normal.dot(b_normal)));
        NODELET_DEBUG("%lu - %lu angle: %f", i, j, theta);
        if (theta > M_PI / 2.0) {
          theta = M_PI  - theta;
        }
        if (theta > connect_plane_angle_threshold_) {
          continue;
        }
        // the planes are near enough as a plane formula.
        
        pcl::PointIndices::Ptr a_indices
          = boost::make_shared<pcl::PointIndices>(boundary_indices[i]);
        pcl::PointIndices::Ptr b_indices
          = boost::make_shared<pcl::PointIndices>(boundary_indices[j]);
        pcl::PointCloud<pcl::PointNormal> a_cloud, b_cloud;
        extract.setIndices(a_indices);
        extract.filter(a_cloud);
        extract.setIndices(b_indices);
        extract.filter(b_cloud);

        // compute the nearest neighbor of a_cloud and b_cloud,
        // and check the distance between them
        if (a_cloud.points.size() > 0) {
          pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
          kdtree.setInputCloud(a_cloud.makeShared());
          bool foundp = false;
          for (size_t pi = 0; pi < b_cloud.points.size(); pi++) {
            pcl::PointNormal p = b_cloud.points[pi];
            std::vector<int> k_indices;
            std::vector<float> k_sqr_distances;
            kdtree.radiusSearch(p, connect_distance_threshold_, k_indices, k_sqr_distances, 1);
            if (k_indices.size() > 0) {
              NODELET_DEBUG("%lu - %lu connected", i, j);
              foundp = true;
              break;
            }
          }
          if (foundp) {
            // i and j can be connected
            connection_map[i].insert(std::map<size_t, bool>::value_type(j, true));
          }
        }
      }
    }

    // connect the clouds
    {
      jsk_pcl_ros::ClusterPointIndices indices;
      jsk_pcl_ros::PolygonArray polygon_array;
      indices.header = msg->header;
      polygon_array.header = msg->header;
      std::vector<bool> skip_list;
      skip_list.resize(connection_map.size());
      std::vector<std::set<int> > cloud_sets;
      for (size_t i = 0; i < connection_map.size(); i++) {
        bool i_done = false;
        // check i is included in cloud_sets
        for (size_t j = 0; j < cloud_sets.size(); j++) {
          if (cloud_sets[j].find(i) != cloud_sets[j].end()) {
            // i is included in cloud_sets[j]
            NODELET_DEBUG("%lu is included in cloud_sets[%lu]", i, j);
            for (std::map<size_t, bool>::iterator it = connection_map[i].begin();
                 it != connection_map[i].end(); it++) {
              NODELET_DEBUG("%lu -> %lu", (*it).first, j);
              cloud_sets[j].insert((*it).first);
            }
            i_done = true;
            break;
          }
        }
        // i is not yet included in cloud_sets
        if (!i_done) {
          std::set<int> new_set;
          new_set.insert(i);
          for (std::map<size_t, bool>::iterator it = connection_map[i].begin();
               it != connection_map[i].end(); it++) {
            new_set.insert((*it).first);
          }
          cloud_sets.push_back(new_set);
        }
      }
      
      for (size_t i = 0; i < cloud_sets.size(); i++) {
        NODELET_DEBUG("%lu cloud", i);
        PCLIndicesMsg one_indices;
        one_indices.header = msg->header;
        for (std::set<int>::iterator it = cloud_sets[i].begin();
             it != cloud_sets[i].end();
             it++) {
          NODELET_DEBUG("%lu includes %d", i, *it);
          pcl::PointIndices inlier = inlier_indices[*it];
          // append indices...
          for (size_t j = 0; j < inlier.indices.size(); j++) {
            one_indices.indices.push_back(inlier.indices[j]);
          }
        }
        indices.cluster_indices.push_back(one_indices);
      }
      pub_.publish(indices);
    }
    
    ros::Time after_connect_time = ros::Time::now();
    NODELET_DEBUG("checking connection done, %f", (after_connect_time - before_connect_time).toSec());
    
  }
  
}

typedef jsk_pcl_ros::OrganizedMultiPlaneSegmentation OrganizedMultiPlaneSegmentation;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, OrganizedMultiPlaneSegmentation, OrganizedMultiPlaneSegmentation, nodelet::Nodelet);
