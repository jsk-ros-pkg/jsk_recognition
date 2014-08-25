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

#include "jsk_pcl_ros/depth_image_error.h"
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  void DepthImageError::onInit()
  {
    PCLNodelet::onInit();
    depth_error_publisher_ = pnh_->advertise<DepthErrorResult>("output", 1);
    sub_image_.subscribe(*pnh_, "image", 1);
    sub_point_.subscribe(*pnh_, "point", 1);
    
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
    sync_->connectInput(sub_image_, sub_point_);
    sync_->registerCallback(boost::bind(&DepthImageError::calcError,
                                        this, _1, _2));
  }

  void DepthImageError::calcError(const sensor_msgs::Image::ConstPtr& depth_image,
                                  const geometry_msgs::PointStamped::ConstPtr& uv_point)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat cv_depth_image = cv_ptr->image;
    double depth_from_depth_sensor = cv_depth_image.at<float>((int)uv_point->point.y, (int)uv_point->point.x);
    NODELET_INFO("timestamp diff is %f", (depth_image->header.stamp - uv_point->header.stamp).toSec());
    NODELET_INFO("(u, v) = (%d, %d)", (int)uv_point->point.x, (int)uv_point->point.y);
    NODELET_INFO("(z, d) = (%f, %f)", uv_point->point.z, depth_from_depth_sensor);
    if (! isnan(depth_from_depth_sensor)) {
      jsk_pcl_ros::DepthErrorResult result;
      result.header.frame_id = depth_image->header.frame_id;
      result.header.stamp = depth_image->header.stamp;
      result.u = (int)uv_point->point.x;
      result.v = (int)uv_point->point.y;
      result.true_depth = uv_point->point.z;
      result.observed_depth = depth_from_depth_sensor;
      depth_error_publisher_.publish(result);
    }
  }
}

typedef jsk_pcl_ros::DepthImageError DepthImageError;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, DepthImageError, DepthImageError, nodelet::Nodelet);
