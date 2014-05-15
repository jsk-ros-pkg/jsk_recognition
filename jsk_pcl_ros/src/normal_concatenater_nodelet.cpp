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

#include "jsk_pcl_ros/normal_concatenater.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{

  void NormalConcatenater::concatenate(const sensor_msgs::PointCloud2::ConstPtr& xyz,
                                       const sensor_msgs::PointCloud2::ConstPtr& normal)
  {
    if (xyz->width != normal->width || xyz->height != normal->height) {
      NODELET_ERROR("~input and ~normal's width or height does not match");
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB> xyz_cloud;
    pcl::PointCloud<pcl::Normal> normal_cloud;
    pcl::PointCloud<pcl::PointXYZRGBNormal> concatenated_cloud;
    pcl::fromROSMsg(*xyz, xyz_cloud);
    pcl::fromROSMsg(*normal, normal_cloud);

    concatenated_cloud.points.resize(xyz_cloud.points.size());
    concatenated_cloud.width = xyz_cloud.width;
    concatenated_cloud.height = xyz_cloud.height;
    concatenated_cloud.is_dense = xyz_cloud.is_dense;

    for (size_t i = 0; i < concatenated_cloud.points.size(); i++) {
      pcl::PointXYZRGBNormal point;
      point.x = xyz_cloud.points[i].x;
      point.y = xyz_cloud.points[i].y;
      point.z = xyz_cloud.points[i].z;
      point.rgb = xyz_cloud.points[i].rgb;
      point.normal_x = normal_cloud.points[i].normal_x;
      point.normal_y = normal_cloud.points[i].normal_y;
      point.normal_z = normal_cloud.points[i].normal_z;
      concatenated_cloud.points[i] = point;
    }
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(concatenated_cloud, output_cloud);
    output_cloud.header = xyz->header;
    pub_.publish(output_cloud);
  }
  
  void NormalConcatenater::onInit()
  {
    PCLNodelet::onInit();
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    if (!pnh_->getParam("max_queue_size", maximum_queue_size_)) {
      maximum_queue_size_ = 100;
    }
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(maximum_queue_size_);
    sub_xyz_.subscribe(*pnh_, "input", 1);
    sub_normal_.subscribe(*pnh_, "normal", 1);
    sync_->connectInput(sub_xyz_, sub_normal_);
    sync_->registerCallback(boost::bind(&NormalConcatenater::concatenate, this, _1, _2));
  }

}

typedef jsk_pcl_ros::NormalConcatenater NormalConcatenater;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, NormalConcatenater, NormalConcatenater, nodelet::Nodelet);

