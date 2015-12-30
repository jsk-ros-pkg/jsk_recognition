// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_pcl_ros/voxel_grid_large_scale.h"
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>

namespace jsk_pcl_ros
{
  void VoxelGridLargeScale::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&VoxelGridLargeScale::configCallback, this, _1, _2);
    srv_->setCallback(f);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void VoxelGridLargeScale::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &VoxelGridLargeScale::filter, this);
  }

  void VoxelGridLargeScale::unsubscribe()
  {
    sub_.shutdown();
  }
  
  void VoxelGridLargeScale::filter(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (leaf_size_ == 0.0) {
      pub_.publish(msg);
    }
    else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      // check size
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cloud, min_pt, max_pt);
      Eigen::Vector4f diff_pt = max_pt - min_pt;
      const int32_t int_max = std::numeric_limits<int32_t>::max();
      const int64_t dx = static_cast<int64_t>(diff_pt[0] / leaf_size_) + 1;
      const int64_t dy = static_cast<int64_t>(diff_pt[1] / leaf_size_) + 1;
      const int64_t dz = static_cast<int64_t>(diff_pt[2] / leaf_size_) + 1;
      const int min_dx = int_max / (dy * dz);
      const int num_x = dx / min_dx + 1;
      const double box_size = min_dx * leaf_size_;
      // ROS_INFO("num_x: %d", num_x);
      // ROS_INFO("%d, %d, %d", dx, dy, dz);
      // ROS_INFO("%d, %d, %d", min_dx, dx, dy*dz);
      for (int xi = 0; xi < num_x; xi++) {
        Eigen::Vector4f min_box = min_pt + Eigen::Vector4f(box_size * xi,
                                                           0,
                                                           0,
                                                           0);
        Eigen::Vector4f max_box = min_pt + Eigen::Vector4f(box_size * (xi + 1),
                                                           diff_pt[1],
                                                           diff_pt[2],
                                                           0);
        pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setMin(min_box);
        crop_box.setMax(max_box);
        crop_box.setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        crop_box.filter(*box_cloud);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        voxel.setInputCloud(box_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel.filter(*voxel_cloud);
        *output += *voxel_cloud;
      }
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*output, ros_cloud);
      ros_cloud.header = msg->header;
      pub_.publish(ros_cloud);
    }
  }

  void VoxelGridLargeScale::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    leaf_size_ = config.leaf_size;
  }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::VoxelGridLargeScale,
                        nodelet::Nodelet);
