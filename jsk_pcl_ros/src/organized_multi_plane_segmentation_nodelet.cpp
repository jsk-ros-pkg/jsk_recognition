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
  }
  
  void OrganizedMultiPlaneSegmentation::segment
  (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    //pcl::PointCloud<pcl::PointNormal>::Ptr input(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal> input;
    pcl::fromROSMsg(*msg, input);
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointNormal, pcl::PointNormal, pcl::Label> mps;
    mps.setMinInliers(min_size_);
    mps.setAngularThreshold(angular_threshold_);
    mps.setDistanceThreshold(distance_threshold_);
    mps.setMaximumCurvature(max_curvature_);
    mps.setInputCloud(input.makeShared());
    mps.setInputNormals(input.makeShared());

    //std::vector<pcl::PlanarRegion<pcl::PointNormal> > regions;
    std::vector<pcl::PlanarRegion<pcl::PointNormal>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointNormal> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    
    mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    ROS_INFO("%lu regions", regions.size());
    jsk_pcl_ros::ClusterPointIndices indices;
    indices.header = msg->header;
    for (size_t i = 0; i < inlier_indices.size(); i++) {
      pcl::PointIndices inlier = inlier_indices[i];
      PCLIndicesMsg one_indices;
      one_indices.indices = inlier.indices;
      one_indices.header = msg->header;
      indices.cluster_indices.push_back(one_indices);
    } 
   pub_.publish(indices);
  }
  
}

typedef jsk_pcl_ros::OrganizedMultiPlaneSegmentation OrganizedMultiPlaneSegmentation;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, OrganizedMultiPlaneSegmentation, OrganizedMultiPlaneSegmentation, nodelet::Nodelet);
