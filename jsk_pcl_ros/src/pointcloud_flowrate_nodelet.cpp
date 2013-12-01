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

#include "jsk_pcl_ros/pointcloud_flowrate.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  void PointCloudFlowRate::onInit(void)
  {
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &PointCloudFlowRate::extract, this);
    publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    if (!pnh_->getParam("rate", rate_)) {
      rate_ = 1.0;              // 1Hz
    }
    int max_size;
    if (!pnh_->getParam("max_size", max_size)) {
      max_size_ = 500;
    }
    else {
      max_size_ = (size_t)max_size;
    }
  }
  
  void PointCloudFlowRate::extract(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg(*input, cloud_xyz);
    size_t cluster_num = cloud_xyz.points.size() / max_size_;
    NODELET_INFO("cluster num is: %lu", cluster_num);
    for (size_t i = 0; i < cluster_num; i++) {
      size_t start_index = i * max_size_;
      size_t finish_index = (i + 1) * max_size_ > cloud_xyz.points.size() ? cloud_xyz.points.size(): (i + 1) * max_size_;
      pcl::PointCloud<pcl::PointXYZ> subcloud;
      subcloud.points.resize(finish_index - start_index);
      for (size_t j = start_index; j < finish_index; j++) { // fill subcloud
        subcloud.points[j - start_index] = cloud_xyz.points[j];
      }
      sensor_msgs::PointCloud2::Ptr out_cloud(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(subcloud, *out_cloud);
      out_cloud->header = input->header;
      publisher_.publish(out_cloud);
      ros::Duration duration(1.0 / rate_);
      duration.sleep();
    }
  }
}

typedef jsk_pcl_ros::PointCloudFlowRate PointCloudFlowRate;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, PointCloudFlowRate, PointCloudFlowRate, nodelet::Nodelet);
