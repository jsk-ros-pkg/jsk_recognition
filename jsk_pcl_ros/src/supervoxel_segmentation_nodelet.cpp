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
#include "jsk_pcl_ros/supervoxel_segmentation.h"
#include <pcl/segmentation/supervoxel_clustering.h>
namespace jsk_pcl_ros
{
  void SupervoxelSegmentation::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &SupervoxelSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output/indices", 1);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output/cloud", 1);
    onInitPostProcess();
  }

  void SupervoxelSegmentation::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &SupervoxelSegmentation::segment, this);
  }

  void SupervoxelSegmentation::unsubscribe()
  {
    sub_.shutdown();
  }

  void SupervoxelSegmentation::segment(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    if (cloud_msg->data.size() > 0) {
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(*cloud_msg, *cloud);
      pcl::SupervoxelClustering<PointT> super(voxel_resolution_,
                                              seed_resolution_,
                                              use_transform_);
      super.setInputCloud(cloud);
      super.setColorImportance(color_importance_);
      super.setSpatialImportance(spatial_importance_);
      super.setNormalImportance(normal_importance_);
      std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
      super.extract(supervoxel_clusters);
      pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
      std::vector<pcl::PointIndices> all_indices;
      for (std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr >::iterator
             it = supervoxel_clusters.begin();
           it != supervoxel_clusters.end();
           ++it) {
        pcl::Supervoxel<PointT>::Ptr super_voxel = it->second;
        pcl::PointCloud<PointT>::Ptr super_voxel_cloud = super_voxel->voxels_;
        pcl::PointIndices indices;
        // add indices...
        for (size_t i = 0; i < super_voxel_cloud->size(); i++) {
          indices.indices.push_back(i + output->points.size());
        }
        all_indices.push_back(indices);
        *output = *output + *super_voxel_cloud; // append
      }
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*output, ros_cloud);
      ros_cloud.header = cloud_msg->header;
      jsk_recognition_msgs::ClusterPointIndices ros_indices;
      ros_indices.cluster_indices = pcl_conversions::convertToROSPointIndices(
        all_indices,
        cloud_msg->header);
      ros_indices.header = cloud_msg->header;
      pub_cloud_.publish(ros_cloud);
      pub_indices_.publish(ros_indices);
    }
  }

  void SupervoxelSegmentation::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    color_importance_ = config.color_importance;
    spatial_importance_ = config.spatial_importance;
    normal_importance_ = config.normal_importance;
    voxel_resolution_ = config.voxel_resolution;
    seed_resolution_ = config.seed_resolution;
    use_transform_ = config.use_transform;
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::SupervoxelSegmentation, nodelet::Nodelet);
