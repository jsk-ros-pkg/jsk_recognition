// -*- mode: C++ -*-
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

#include "jsk_pcl_ros_utils/depth_image_error.h"
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros_utils
{
  void DepthImageError::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    depth_error_publisher_ = advertise<jsk_recognition_msgs::DepthErrorResult>(*pnh_, "output", 1);
  }

  void DepthImageError::subscribe()
  {
    sub_image_.subscribe(*pnh_, "image", 1);
    sub_point_.subscribe(*pnh_, "point", 1);
    sub_camera_info_.subscribe(*pnh_, "camera_info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ASyncPolicy> >(1000);
      async_->connectInput(sub_image_, sub_point_, sub_camera_info_);
      async_->registerCallback(boost::bind(&DepthImageError::calcError,
                                          this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
      sync_->connectInput(sub_image_, sub_point_, sub_camera_info_);
      sync_->registerCallback(boost::bind(&DepthImageError::calcError,
                                          this, _1, _2, _3));
    }
  }

  void DepthImageError::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_point_.unsubscribe();
  }


  void DepthImageError::calcError(const sensor_msgs::Image::ConstPtr& depth_image,
                                  const geometry_msgs::PointStamped::ConstPtr& uv_point,
                                  const sensor_msgs::CameraInfo::ConstPtr& camera_info)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat cv_depth_image = cv_ptr->image;
    double depth_from_depth_sensor = cv_depth_image.at<float>((int)uv_point->point.y, (int)uv_point->point.x);
    NODELET_INFO("timestamp diff is %f", (depth_image->header.stamp - uv_point->header.stamp).toSec());
    NODELET_INFO("(u, v) = (%d, %d)", (int)uv_point->point.x, (int)uv_point->point.y);
    NODELET_INFO("(z, d) = (%f, %f)", uv_point->point.z, depth_from_depth_sensor);
    if (! isnan(depth_from_depth_sensor)) {
      jsk_recognition_msgs::DepthErrorResult result;
      result.header.frame_id = depth_image->header.frame_id;
      result.header.stamp = depth_image->header.stamp;
      result.u = (int)uv_point->point.x;
      result.v = (int)uv_point->point.y;
      result.center_u = camera_info->P[2];
      result.center_v = camera_info->P[6];
      result.true_depth = uv_point->point.z;
      result.observed_depth = depth_from_depth_sensor;
      depth_error_publisher_.publish(result);
    }
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::DepthImageError, nodelet::Nodelet);
