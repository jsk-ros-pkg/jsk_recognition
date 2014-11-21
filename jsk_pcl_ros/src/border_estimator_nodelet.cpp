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
#include <pcl/range_image/range_image_planar.h>
#include "jsk_pcl_ros/border_estimator.h"

namespace jsk_pcl_ros
{
  void BorderEstimator::onInit()
  {
    PCLNodelet::onInit();
    pub_border_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_border", 1);
    pub_veil_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_veil", 1);
    pub_shadow_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_shadow", 1);
  }

  void BorderEstimator::subscribe()
  {
    sub_point_.subscribe(*pnh_, "input", 1);
    sub_camera_info_.subscribe(*pnh_, "input_camera_info", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_point_, sub_camera_info_);
    sync_->registerCallback(boost::bind(&BorderEstimator::estimate, this, _1, _2));
  }

  void BorderEstimator::unsubscribe()
  {
    sub_point_.unsubscribe();
    sub_camera_info_.unsubscribe();
  }
  
  pcl::PointXYZ BorderEstimator::convertPoint(const pcl::PointWithRange& input)
  {
    pcl::PointXYZ output;
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
    return output;
  }
  
  void BorderEstimator::publishCloud(
    ros::Publisher& pub,
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std_msgs::Header& header)
  {
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    ros_msg.header = header;
    pub.publish(ros_msg);
  }
  
  void BorderEstimator::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::CameraInfo::ConstPtr& info)
  {
    if (msg->height == 1) {
      NODELET_ERROR("[BorderEstimator::estimate] pointcloud must be organized");
      return;
    }
    pcl::RangeImagePlanar range_image;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    Eigen::Affine3f dummytrans = Eigen::Affine3f::Identity();
    float fx = info->P[0];
    float cx = info->P[2];
    float tx = info->P[3];
    float fy = info->P[5];
    float cy = info->P[6];
    range_image.createFromPointCloudWithFixedSize (cloud,
                                                   msg->width,
                                                   msg->height,
                                                   cx, cy,
                                                   fx, fy,
                                                   dummytrans);
    range_image.setUnseenToMaxRange();
    pcl::RangeImageBorderExtractor border_extractor (&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute (border_descriptions);
    pcl::PointCloud<pcl::PointXYZ> border_points, veil_points, shadow_points;
    for (int y = 0; y < (int)range_image.height; ++y) {
      for (int x = 0; x < (int)range_image.width; ++x) {
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]) {
          border_points.points.push_back (
            convertPoint(range_image.points[y*range_image.width + x]));
        }
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT]) {
          veil_points.points.push_back (
            convertPoint(range_image.points[y*range_image.width + x]));
        }
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER]) {
          shadow_points.points.push_back (
            convertPoint(range_image.points[y*range_image.width + x]));
        }
      }
    }
    publishCloud(pub_border_, border_points, msg->header);
    publishCloud(pub_veil_, veil_points, msg->header);
    publishCloud(pub_shadow_, shadow_points, msg->header);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::BorderEstimator,
                        nodelet::Nodelet);
