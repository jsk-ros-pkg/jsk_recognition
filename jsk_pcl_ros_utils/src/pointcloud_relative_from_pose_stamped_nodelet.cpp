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
 *     disclaimer in the documentation and/or other materials provided
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

#include "jsk_pcl_ros_utils/pointcloud_relative_from_pose_stamped.h"
#include <jsk_recognition_utils/tf_listener_singleton.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_utils/pcl_ros_util.h>
#include <pcl/common/transforms.h>

namespace jsk_pcl_ros_utils
{
  void PointCloudRelativeFromPoseStamped::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pnh_->param<bool>("approximate_sync", approximate_sync_, false);
    onInitPostProcess();
  }

  PointCloudRelativeFromPoseStamped::~PointCloudRelativeFromPoseStamped() {
    // message_filters::Synchronizer needs to be called reset
    // before message_filters::Subscriber is freed.
    // Calling reset fixes the following error on shutdown of the nodelet:
    // terminate called after throwing an instance of
    // 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    //     what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    // Also see https://github.com/ros/ros_comm/issues/720 .
    sync_.reset();
    async_.reset();
  }

  void PointCloudRelativeFromPoseStamped::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_pose_.subscribe(*pnh_, "input/pose", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_cloud_, sub_pose_);
      async_->registerCallback(boost::bind(&PointCloudRelativeFromPoseStamped::transform, this, boost::placeholders::_1, boost::placeholders::_2));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_cloud_, sub_pose_);
      sync_->registerCallback(boost::bind(&PointCloudRelativeFromPoseStamped::transform, this, boost::placeholders::_1, boost::placeholders::_2));
    }
  }
  
  void PointCloudRelativeFromPoseStamped::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_pose_.unsubscribe();
  }

  void PointCloudRelativeFromPoseStamped::transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                                    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    vital_checker_->poke();
    if (!jsk_recognition_utils::isSameFrameId(cloud_msg->header.frame_id,
                                             pose_msg->header.frame_id)) {
      NODELET_ERROR("frame_id does not match. cloud: %s, pose: %s",
                        cloud_msg->header.frame_id.c_str(),
                        pose_msg->header.frame_id.c_str());
      return;
    }
    Eigen::Affine3f transform;
    tf::poseMsgToEigen(pose_msg->pose, transform);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    Eigen::Affine3f inverse_transform = transform.inverse();
    pcl::transformPointCloud(*cloud, *transformed_cloud, inverse_transform);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*transformed_cloud, ros_cloud);
    ros_cloud.header = cloud_msg->header;
    pub_.publish(ros_cloud);
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PointCloudRelativeFromPoseStamped, nodelet::Nodelet);
