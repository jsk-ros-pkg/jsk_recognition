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
#include "jsk_pcl_ros/capture_stereo_synchronizer.h"
#include <pcl/common/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Int32.h>

namespace jsk_pcl_ros
{
  void CaptureStereoSynchronizer::onInit()
  {
    DiagnosticNodelet::onInit();
    counter_ = 0;
    pnh_->param("rotational_bin_size", rotational_bin_size_, pcl::deg2rad(10.0));
    pnh_->param("positional_bin_size", positional_bin_size_, 0.1);
    pub_pose_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose", 1);
    pub_mask_ = advertise<sensor_msgs::Image>(*pnh_, "output/mask", 1);
    poses_.resize(0);
    pub_mask_indices_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/mask_indices", 1);
    pub_left_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output/left_image", 1);
    pub_left_cam_info_ = advertise<sensor_msgs::CameraInfo>(
      *pnh_, "output/left_camera_info", 1);
    pub_right_cam_info_ = advertise<sensor_msgs::CameraInfo>(
      *pnh_, "output/right_camera_info", 1);
    pub_disparity_ = advertise<stereo_msgs::DisparityImage>(
      *pnh_, "output/disparity", 1);
    pub_count_ = advertise<std_msgs::Int32>(
      *pnh_, "output/count", 1);
    onInitPostProcess();
 }

  bool CaptureStereoSynchronizer::checkNearPose(
    const geometry_msgs::Pose& new_pose)
  {
    Eigen::Affine3d new_affine;
    tf::poseMsgToEigen(new_pose, new_affine);
    for (size_t i = 0; i < poses_.size(); i++) {
      // convert pose into eigen
      Eigen::Affine3d affine;
      tf::poseMsgToEigen(poses_[i], affine);
      // compute difference
      Eigen::Affine3d diff = affine.inverse() * new_affine;
      double positional_difference = diff.translation().norm();
      if (positional_difference < positional_bin_size_) {
        Eigen::AngleAxisd rotational_difference(diff.rotation());
        if (rotational_difference.angle() < rotational_bin_size_) {
          return true;
        }
      }
    }
    return false;
  }
  
  void CaptureStereoSynchronizer::subscribe()
  {
    sub_pose_.subscribe(*pnh_, "input/pose", 1);
    sub_mask_.subscribe(*pnh_, "input/mask", 1);
    sub_mask_indices_.subscribe(*pnh_, "input/mask_indices", 1);
    sub_left_image_.subscribe(*pnh_, "input/left_image", 1);
    sub_left_cam_info_.subscribe(*pnh_, "input/left_camera_info", 1);
    sub_right_cam_info_.subscribe(*pnh_, "input/right_camera_info", 1);
    sub_disparity_.subscribe(*pnh_, "input/disparity", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_pose_,
                        sub_mask_,
                        sub_mask_indices_,
                        sub_left_image_,
                        sub_left_cam_info_,
                        sub_right_cam_info_,
                        sub_disparity_);
    sync_->registerCallback(boost::bind(&CaptureStereoSynchronizer::republish,
                                        this,
                                        _1,
                                        _2,
                                        _3,
                                        _4,
                                        _5,
                                        _6,
                                        _7));
  }

  void CaptureStereoSynchronizer::unsubscribe()
  {
    sub_pose_.unsubscribe();
    sub_mask_.unsubscribe();
    sub_mask_indices_.unsubscribe();
    sub_left_image_.unsubscribe();
    sub_left_cam_info_.unsubscribe();
    sub_right_cam_info_.unsubscribe();
    sub_disparity_.unsubscribe();
  }

  void CaptureStereoSynchronizer::republish(
    const geometry_msgs::PoseStamped::ConstPtr& pose,
    const sensor_msgs::Image::ConstPtr& mask,
    const PCLIndicesMsg::ConstPtr& mask_indices,
    const sensor_msgs::Image::ConstPtr& left_image,
    const sensor_msgs::CameraInfo::ConstPtr& left_cam_info,
    const sensor_msgs::CameraInfo::ConstPtr& right_cam_info,
    const stereo_msgs::DisparityImage::ConstPtr& disparity)
  {
    if (checkNearPose(pose->pose)) {
      ROS_DEBUG("too near");
    }
    else {
      ROS_INFO("%d sample", counter_++);
      poses_.push_back(pose->pose);
      pub_pose_.publish(pose);
      pub_mask_.publish(mask);
      pub_mask_indices_.publish(mask_indices);
      pub_left_image_.publish(left_image);
      pub_left_cam_info_.publish(left_cam_info);
      pub_right_cam_info_.publish(right_cam_info);
      pub_disparity_.publish(disparity);
    }
    std_msgs::Int32 count;
    count.data = counter_;
    pub_count_.publish(count);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::CaptureStereoSynchronizer, nodelet::Nodelet);
