// -*- mode: c++ -*-
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

#include "jsk_pcl_ros/add_color_from_image_to_organized.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>

namespace jsk_pcl_ros
{
  void AddColorFromImageToOrganized::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void AddColorFromImageToOrganized::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_image_.subscribe(*pnh_, "input/image", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_image_);
    sync_->registerCallback(boost::bind(&AddColorFromImageToOrganized::addColor, this, _1, _2));
  }

  void AddColorFromImageToOrganized::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_image_.unsubscribe();
  }

  void AddColorFromImageToOrganized::addColor(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    if (cloud_msg->header.frame_id != image_msg->header.frame_id)
    {
      NODELET_FATAL("frame_id does not match: [%s, %s]",
                    cloud_msg->header.frame_id.c_str(),
                    image_msg->header.frame_id.c_str());
      return;
    }
    if (cloud_msg->height != image_msg->height || cloud_msg->width != image_msg->width)
    {
      NODELET_FATAL("Size of input cloud and image does not match.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    cv::Mat image = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::BGR8)->image;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    rgb_cloud->points.resize(cloud->points.size());
    rgb_cloud->is_dense = cloud->is_dense;
    rgb_cloud->width = cloud->width;
    rgb_cloud->height = cloud->height;
    for (size_t j=0; j<cloud->height; j++)
    {
      for (size_t i=0; i<cloud->width; i++)
      {
        pcl::PointXYZ p_xyz = cloud->points[i + j * cloud->width];
        pcl::PointXYZRGB p_color;
        p_color.x = p_xyz.x;
        p_color.y = p_xyz.y;
        p_color.z = p_xyz.z;
        cv::Vec3b bgr = image.at<cv::Vec3b>(j, i);
        p_color.b = bgr[0];
        p_color.g = bgr[1];
        p_color.r = bgr[2];
        rgb_cloud->points[i + j * cloud->width] = p_color;
      }
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*rgb_cloud, ros_cloud);
    ros_cloud.header = cloud_msg->header;
    pub_.publish(ros_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::AddColorFromImageToOrganized, nodelet::Nodelet);
