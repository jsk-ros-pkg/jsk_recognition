// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Satoshi Otsubo and JSK Lab
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

#include <pcl_conversions/pcl_conversions.h>

#include "jsk_pcl_ros/color_based_region_growing_segmentation.h"

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_msgs/ClusterPointIndices.h"


namespace jsk_pcl_ros
{

  void ColorBasedRegionGrowingSegmentation::onInit()
  {
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f=
      boost::bind (&ColorBasedRegionGrowingSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ColorBasedRegionGrowingSegmentation::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ColorBasedRegionGrowingSegmentation::segment, this);
  }

  void ColorBasedRegionGrowingSegmentation::unsubscribe()
  {
    sub_.shutdown();
  }

  void ColorBasedRegionGrowingSegmentation::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (distance_threshold_ != config.distance_threshold) {
      distance_threshold_ = config.distance_threshold;
    }
    if (point_color_threshold_ != config.point_color_threshold) {
      point_color_threshold_ = config.point_color_threshold;
    }
    if (region_color_threshold_ != config.region_color_threshold) {
      region_color_threshold_ = config.region_color_threshold;
    }
    if (min_cluster_size_ != config.min_cluster_size) {
      min_cluster_size_ = config.min_cluster_size;
    }
  }

  void ColorBasedRegionGrowingSegmentation::segment(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
      pcl::search::Search <pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (distance_threshold_);
    reg.setPointColorThreshold (point_color_threshold_);
    reg.setRegionColorThreshold (region_color_threshold_);
    reg.setMinClusterSize (min_cluster_size_);

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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ColorBasedRegionGrowingSegmentation, nodelet::Nodelet);
