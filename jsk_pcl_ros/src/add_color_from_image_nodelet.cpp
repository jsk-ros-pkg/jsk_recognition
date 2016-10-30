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

#include "jsk_pcl_ros/add_color_from_image.h"
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>

namespace jsk_pcl_ros
{
  void AddColorFromImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void AddColorFromImage::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_image_.subscribe(*pnh_, "input/image", 1);
    sub_info_.subscribe(*pnh_, "input/camera_info", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_image_, sub_info_);
    sync_->registerCallback(boost::bind(&AddColorFromImage::addColor,
                                        this, _1, _2, _3));
  }

  void AddColorFromImage::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_info_.unsubscribe();
    sub_image_.unsubscribe();
  }
  
  void AddColorFromImage::addColor(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    if ((cloud_msg->header.frame_id != image_msg->header.frame_id) ||
        (cloud_msg->header.frame_id != info_msg->header.frame_id)) {
      NODELET_FATAL("frame_id is not collect: [%s, %s, %s",
                    cloud_msg->header.frame_id.c_str(),
                    image_msg->header.frame_id.c_str(),
                    info_msg->header.frame_id.c_str());
      return;
    }
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    rgb_cloud->points.resize(cloud->points.size());
    rgb_cloud->is_dense = cloud->is_dense;
    rgb_cloud->width = cloud->width;
    rgb_cloud->height = cloud->height;
    cv::Mat image = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::BGR8)->image;
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info_msg);
    for (size_t i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZRGB p;
      p.x = cloud->points[i].x;
      p.y = cloud->points[i].y;
      p.z = cloud->points[i].z;
      //p.getVector3fMap() = cloud->points[i].getVector3fMap();
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
      // compute RGB
        cv::Point2d uv = model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
        if (uv.x > 0 && uv.x < image_msg->width &&
            uv.y > 0 && uv.y < image_msg->height) {
          cv::Vec3b rgb = image.at<cv::Vec3b>(uv.y, uv.x);
          p.r = rgb[2];
          p.g = rgb[1];
          p.b = rgb[0];
        }
      }
      rgb_cloud->points[i] = p;
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*rgb_cloud, ros_cloud);
    ros_cloud.header = cloud_msg->header;
    pub_.publish(ros_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::AddColorFromImage, nodelet::Nodelet);
