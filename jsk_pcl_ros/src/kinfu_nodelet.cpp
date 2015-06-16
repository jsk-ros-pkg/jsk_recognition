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

#include "jsk_pcl_ros/kinfu.h"
#include <geometry_msgs/PoseStamped.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <cv_bridge/cv_bridge.h>

namespace jsk_pcl_ros
{
  void Kinfu::subscribe()
  {

  }

  void Kinfu::unsubscribe()
  {

  }

  void Kinfu::onInit()
  {
    DiagnosticNodelet::onInit();
    initialized_ = false;
    pcl::gpu::setDevice(0);
    pcl::gpu::printShortCudaDeviceInfo(0);
    volume_size_ = pcl::device::kinfuLS::VOLUME_SIZE;
    shift_distance_ = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
    snapshot_rate_ = pcl::device::kinfuLS::SNAPSHOT_RATE; // defined in device.h
    pub_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output", 1);
    sub_depth_image_.subscribe(*pnh_, "input/depth", 1);
    sub_color_image_.subscribe(*pnh_, "input/color", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_depth_image_, sub_color_image_);
    sync_->registerCallback(boost::bind(&Kinfu::callback, this, _1, _2));
    sub_info_ = pnh_->subscribe("input/info", 1, &Kinfu::infoCallback, this);
  }

  void Kinfu::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    info_msg_ = info_msg;
  }

  void Kinfu::callback(const sensor_msgs::Image::ConstPtr& depth_image,
                       const sensor_msgs::Image::ConstPtr& rgb_image)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!info_msg_) {
      JSK_NODELET_WARN("camera info is not yet ready");
      return;
    }
    if (!initialized_) {
      Eigen::Vector3f volume_size = Eigen::Vector3f::Constant (volume_size_);
      
      // Setup kinfu
      kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shift_distance_,
                                                   info_msg_->height, info_msg_->width);
      // const Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf(pcl::deg2rad(-30.f), Vector3f::UnitX());
      // const Eigen::Vector3f t = volume_size * 0.5f - Eigen::Vector3f(0, 0, volume_size(2) / 2 * 1.2f);
      // //const Eigen::Vector3f t(-1.5, -1.5, 0.3);
      // const Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);
      // kinfu_->setInitialCameraPose(pose);
      kinfu_->setInitialCameraPose(Eigen::Affine3f::Identity());
      kinfu_->volume().setTsdfTruncDist (0.030f/*meters*/);
      kinfu_->setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
      //kinfu_->setIcpCorespFilteringParams (0.5f/*meters*/, sin ( pcl::deg2rad(40.f) ));
      kinfu_->setCameraMovementThreshold(0.001f);
      kinfu_->setDepthIntrinsics (info_msg_->K[0], info_msg_->K[4], info_msg_->K[2], info_msg_->K[5]);
      initialized_ = true;
    }
    if (kinfu_->icpIsLost()) {
      kinfu_->reset();
      JSK_NODELET_WARN("kinfu is reset");
    }
    cv::Mat depth_m_image = cv_bridge::toCvShare(depth_image, "32FC1")->image;
    cv::Mat depth_mm_image = depth_m_image * 1000.0;
    cv::Mat depth_mm_sc_image;
    depth_mm_image.convertTo(depth_mm_sc_image, CV_16UC1);
    depth_device_.upload(&(depth_mm_sc_image.data[0]), depth_image->width * 2, depth_image->height, depth_image->width);
    //colors_device_.upload(&(rgb_image->data[0]), rgb_image->step, rgb_image->height, rgb_image->width);

    //(*kinfu_)(depth_device_, colors_device_);
    (*kinfu_)(depth_device_);
    Eigen::Affine3f camera_pose = kinfu_->getCameraPose();
    geometry_msgs::PoseStamped ros_pose;
    tf::poseEigenToMsg(camera_pose, ros_pose.pose);
    ros_pose.header = depth_image->header;
    pub_pose_.publish(ros_pose);
  }
                       
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::Kinfu,
                        nodelet::Nodelet);
