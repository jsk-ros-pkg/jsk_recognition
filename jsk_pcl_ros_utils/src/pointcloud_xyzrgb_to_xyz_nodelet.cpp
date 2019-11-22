/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include <pcl_conversions/pcl_conversions.h>

#include "jsk_pcl_ros_utils/pointcloud_xyzrgb_to_xyz.h"

namespace jsk_pcl_ros_utils
{

void PointCloudXYZRGBToXYZ::onInit()
{
  DiagnosticNodelet::onInit();
  pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
  onInitPostProcess();
}

void PointCloudXYZRGBToXYZ::subscribe()
{
  sub_ = pnh_->subscribe("input", 1, &PointCloudXYZRGBToXYZ::convert, this);
}

void PointCloudXYZRGBToXYZ::unsubscribe()
{
  sub_.shutdown();
}

void PointCloudXYZRGBToXYZ::convert(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  vital_checker_->poke();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *cloud_xyzrgb);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_xyz->points.resize(cloud_xyzrgb->points.size());
  cloud_xyz->is_dense = cloud_xyzrgb->is_dense;
  cloud_xyz->width = cloud_xyzrgb->width;
  cloud_xyz->height = cloud_xyzrgb->height;
  for (size_t i = 0; i < cloud_xyzrgb->points.size(); i++) {
    pcl::PointXYZ p;
    p.x = cloud_xyzrgb->points[i].x;
    p.y = cloud_xyzrgb->points[i].y;
    p.z = cloud_xyzrgb->points[i].z;
    cloud_xyz->points[i] = p;
  }
  sensor_msgs::PointCloud2 out_cloud_msg;
  pcl::toROSMsg(*cloud_xyz, out_cloud_msg);
  out_cloud_msg.header = cloud_msg->header;
  pub_.publish(out_cloud_msg);
}

}  // namespace jsk_pcl_ros_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::PointCloudXYZRGBToXYZ, nodelet::Nodelet);
