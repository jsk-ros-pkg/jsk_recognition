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

#include "jsk_pcl_ros/voxel_grid_downsample_decoder.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>


namespace jsk_pcl_ros
{

  int VoxelGridDownsampleDecoder::getPointcloudID(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input) {
    return input->slice_index;
  }
  
  int VoxelGridDownsampleDecoder::getPointcloudSequenceID(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input) {
    return input->sequence_id;
  }

  std::string VoxelGridDownsampleDecoder::getPointcloudFrameId(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input) {
    return input->point_cloud.header.frame_id;
  }
  
  void VoxelGridDownsampleDecoder::pointCB(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input)
  {
    NODELET_INFO_STREAM("new pointcloud!" << input->point_cloud.header.frame_id);
    
    int id = getPointcloudID(input);
    int new_sequence_id = getPointcloudSequenceID(input);
    std::string frame_id = getPointcloudFrameId(input);
    if (new_sequence_id != latest_sequence_id_) {
      NODELET_INFO_STREAM("clearing pointcloud");
      pc_buffer_.clear();
      latest_sequence_id_ = new_sequence_id;
    }
    
    // update the buffer
    if (id >= (int)pc_buffer_.size()) {
      // extend the buffer
      //pc_buffer_.resize(id + 1);
      int extend = id - pc_buffer_.size() + 1;
      NODELET_INFO_STREAM("extend " << extend << " pointclouds");
      for (int i = 0; i < extend; i++) {
        NODELET_INFO_STREAM("new pointcloud allocation!");
        pc_buffer_.push_back(jsk_recognition_msgs::SlicedPointCloudConstPtr());
        pc_buffer_[pc_buffer_.size() - 1].reset();
      }
    }
    //pc_buffer_.push_back(input);
    NODELET_INFO_STREAM("id: " << id << " size: " << pc_buffer_.size());
    pc_buffer_[id] = input;
    if (pc_buffer_.size() == 1) {
      // no need to do anything
    }
    else {
      if (previous_id_ + 1 != id) { // the point cloud is not continuous
        if (previous_id_ != (int)pc_buffer_.size() - 1) { // if not the last one
          // make pc_buffer_ shorten
          pc_buffer_.resize(previous_id_ + 1);
        }
      }
    }
    previous_id_ = id;          // update the previous one
    // publush the buffer
    publishBuffer();
    
  }

  void VoxelGridDownsampleDecoder::publishBuffer(void)
  {
    NODELET_INFO("publishBuffer");
    if (pc_buffer_.size() == 0 || !pc_buffer_[0]) {
      NODELET_WARN("no pointcloud is subscribed yet");
      return;
    }
    
    std::string result_frame_id = getPointcloudFrameId(pc_buffer_[0]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenated_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < pc_buffer_.size(); i++)
    {
      if (!pc_buffer_[i]) {
        NODELET_INFO_STREAM("buffer[" << i << "] is not yet available, skip it");
        continue;
      }
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      fromROSMsg(pc_buffer_[i]->point_cloud, *tmp_cloud);
      if (tmp_cloud->points.size() == 0) {
        NODELET_INFO_STREAM("buffer[" << i << "] is not yet available, skip it");
        continue;
      }

      
      tmp_cloud->header.frame_id = getPointcloudFrameId(pc_buffer_[i]);
      // transform the pointcloud
      pcl_ros::transformPointCloud(result_frame_id,
                                   *tmp_cloud,
                                   *transformed_tmp_cloud,
                                   tf_listener);
      // concatenate the tmp_cloud into concatenated_cloud
      for (size_t j = 0; j < transformed_tmp_cloud->points.size(); j++) {
        concatenated_cloud->points.push_back(transformed_tmp_cloud->points[j]);
      }
    }
    sensor_msgs::PointCloud2 out;
    toROSMsg(*concatenated_cloud, out);
    out.header = pc_buffer_[0]->point_cloud.header;
    out.header.frame_id = getPointcloudFrameId(pc_buffer_[0]);
    pub_.publish(out);
  }
  
  void VoxelGridDownsampleDecoder::onInit(void)
  {
    ConnectionBasedNodelet::onInit();
    previous_id_ = -1;
    // decoded output
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void VoxelGridDownsampleDecoder::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &VoxelGridDownsampleDecoder::pointCB,
                           this);
  }

  void VoxelGridDownsampleDecoder::unsubscribe()
  {
    sub_.shutdown();
  }
}


PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::VoxelGridDownsampleDecoder,
                        nodelet::Nodelet);
