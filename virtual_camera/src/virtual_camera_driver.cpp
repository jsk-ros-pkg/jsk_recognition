/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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
 *   * Neither the name of the Kentaro Wada nor the names of its
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

#include "virtual_camera/virtual_camera_driver.h"

#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

namespace virtual_camera
{

void VirtualCameraDriver::onInit()
{
  nh_ = new ros::NodeHandle(getNodeHandle());
  pnh_ = new ros::NodeHandle(getPrivateNodeHandle());

  double max_rate = 0;

  // camera info
  if (pnh_->hasParam("camera_info_url"))
  {
    // load camera_info from file
    std::string camera_info_url;
    pnh_->getParam("camera_info_url", camera_info_url);
    camera_info_manager::CameraInfoManager* info_manager =
      new camera_info_manager::CameraInfoManager(*nh_, "virtual_camera", camera_info_url);
    info_msg_ = info_manager->getCameraInfo();
    pub_info_ = pnh_->advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    // publication rate
    pnh_->param("camera_info_rate", rate_info_, 30.0);
    if (rate_info_ > max_rate)
    {
      max_rate = rate_info_;
    }

    publish_info_ = true;
  }

  // image
  if (pnh_->hasParam("image_file"))
  {
    // load image from file
    std::string image_file;
    pnh_->getParam("image_file", image_file);
    cv::Mat img = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
    if (img.empty())
    {
      ROS_FATAL("Failed to load image from %s to publish.", image_file.c_str());
      exit(1);
    }
    if (publish_info_)
    {
      image_msg_ = cv_bridge::CvImage(info_msg_.header, "bgr8", img).toImageMsg();
    }
    else
    {
      image_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    }
    pub_image_ = pnh_->advertise<sensor_msgs::Image>("image_raw", 1);
    // publication rate
    pnh_->param("image_rate", rate_image_, 30.0);
    if (rate_image_ > max_rate)
    {
      max_rate = rate_image_;
    }

    publish_image_ = true;
  }

  // cloud
  if (pnh_->hasParam("pcd_file"))
  {
    // load point cloud from file
    std::string pcd_file;
    pnh_->getParam("pcd_file", pcd_file);
    if (pcl::io::loadPCDFile(pcd_file, cloud_msg_) == -1)
    {
      ROS_FATAL("Failed to load pcd file from %s.", pcd_file.c_str());
      exit(1);
    }
    if (publish_info_)
    {
      cloud_msg_.header = info_msg_.header;
    }
    pub_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("points", 1);
    // publication rate
    pnh_->param("point_cloud_rate", rate_cloud_, 30.0);
    if (rate_cloud_ > max_rate)
    {
      max_rate = rate_cloud_;
    }

    publish_cloud_ = true;
  }

  if (!(publish_info_ || publish_image_ || publish_cloud_))
  {
    ROS_FATAL("Nothing to publish. You should set ~camera_info_file, ~image_file or ~pcd_file.");
    exit(1);
  }

  double timer_rate = max_rate * 100;
  timer_pub_ = nh_->createTimer(
    ros::Duration(1 / timer_rate),
    &VirtualCameraDriver::publishCb, this);
}

void VirtualCameraDriver::publishCb(const ros::TimerEvent& event)
{
  boost::mutex::scoped_lock lock(mutex_);

  // publish camera info
  if (publish_info_ && (event.current_real - time_prev_pub_info_) >= ros::Duration(1 / rate_info_))
  {
    if (pub_info_.getNumSubscribers() > 0)
    {
      info_msg_.header.stamp = event.current_real;
      pub_info_.publish(info_msg_);
    }
    time_prev_pub_info_ = event.current_real;
  }

  // publish image
  if (publish_image_ && (event.current_real - time_prev_pub_image_) >= ros::Duration(1 / rate_image_))
  {
    if (pub_image_.getNumSubscribers() > 0)
    {
      image_msg_->header.stamp = event.current_real;
      pub_image_.publish(image_msg_);
    }
    time_prev_pub_image_ = event.current_real;
  }

  // publish point cloud
  if (publish_cloud_ && (event.current_real - time_prev_pub_cloud_) >= ros::Duration(1 / rate_cloud_))
  {
    if (pub_cloud_.getNumSubscribers() > 0)
    {
      cloud_msg_.header.stamp = event.current_real;
      pub_cloud_.publish(cloud_msg_);
    }
    time_prev_pub_cloud_ = event.current_real;
  }
}

}  // namespace virtual_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(virtual_camera::VirtualCameraDriver, nodelet::Nodelet);
