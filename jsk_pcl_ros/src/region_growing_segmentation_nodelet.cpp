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

#include "jsk_pcl_ros/region_growing_segmentation.h"

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pluginlib/class_list_macros.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{

  void RegionGrowingSegmentation::onInit()
  {
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&RegionGrowingSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void RegionGrowingSegmentation::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &RegionGrowingSegmentation::segment, this);
  }

  void RegionGrowingSegmentation::unsubscribe()
  {
    sub_.shutdown();
  }

  void RegionGrowingSegmentation::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (number_of_neighbors_ != config.number_of_neighbors) {
      number_of_neighbors_ = config.number_of_neighbors;
    }
    if (min_size_ != config.min_size) {
      min_size_ = config.min_size;
    }
    if (max_size_ != config.max_size) {
      max_size_ = config.max_size;
    }
    if (smoothness_threshold_ != config.smoothness_threshold) {
      smoothness_threshold_ = config.smoothness_threshold;
    }
    if (curvature_threshold_ != config.curvature_threshold) {
      curvature_threshold_ = config.curvature_threshold;
    }
  }

  void RegionGrowingSegmentation::segment(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::search::Search<pcl::PointNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointNormal> > (new pcl::search::KdTree<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal> cloud;
    pcl::fromROSMsg(*msg, cloud);
    
    pcl::RegionGrowing<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setMinClusterSize (min_size_);
    reg.setMaxClusterSize (max_size_);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (number_of_neighbors_);
    reg.setInputCloud (cloud.makeShared());
    //reg.setIndices (indices);
    reg.setInputNormals (cloud.makeShared());
    reg.setSmoothnessThreshold (smoothness_threshold_);
    reg.setCurvatureThreshold (curvature_threshold_);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    jsk_recognition_msgs::ClusterPointIndices output;
    output.header = msg->header;

    for (size_t i = 0; i < clusters.size(); i++) {
      PCLIndicesMsg indices;
      indices.header = msg->header;
      indices.indices = clusters[i].indices;
      output.cluster_indices.push_back(indices);
    }
    pub_.publish(output);
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::RegionGrowingSegmentation,
                        nodelet::Nodelet);
