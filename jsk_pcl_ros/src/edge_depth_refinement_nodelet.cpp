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

#include "jsk_pcl_ros/edge_depth_refinement.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>

namespace jsk_pcl_ros
{
  void EdgeDepthRefinement::onInit()
  {
    PCLNodelet::onInit();
    pub_indices_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>(
      "output", 1);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_indices_);
    sync_->registerCallback(boost::bind(&EdgeDepthRefinement::refine,
                                        this, _1, _2));
  }

  void EdgeDepthRefinement::refine(
    const sensor_msgs::PointCloud2ConstPtr &input,
    const jsk_pcl_ros::ClusterPointIndicesConstPtr &indices)
  {
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);
    jsk_pcl_ros::ClusterPointIndices output_ros_msg;
    output_ros_msg.header = input->header;
    for (size_t i_cluster = 0;
         i_cluster < indices->cluster_indices.size();
         i_cluster++) {
      PCLIndicesMsg indices_msg = indices->cluster_indices[i_cluster];
      std::vector<int> cluster_indices = indices_msg.indices;
      pcl::PointCloud<PointT>::Ptr original_line_cloud (new pcl::PointCloud<PointT>);
      
      
      //pcl::copyPointCloud(*cloud, cluster_indices, *original_line_cloud);
      
      // one line shoud have ONE line at most...?
      // in that case, we can estimate the true line by RANSAC
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_LINE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      seg.setInputCloud(cloud);
      pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices);
      indices_ptr->indices = cluster_indices;
      seg.setIndices(indices_ptr);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      seg.segment (*inliers, *coefficients);
      NODELET_INFO("%lu points", inliers->indices.size());
      PCLIndicesMsg output_indices_msg;
      output_indices_msg.header = input->header;
      output_indices_msg.indices = inliers->indices;
      output_ros_msg.cluster_indices.push_back(output_indices_msg);
    }
    pub_indices_.publish(output_ros_msg);
  }
  
}

#include <pluginlib/class_list_macros.h>
typedef jsk_pcl_ros::EdgeDepthRefinement EdgeDepthRefinement;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, EdgeDepthRefinement, EdgeDepthRefinement, nodelet::Nodelet);
