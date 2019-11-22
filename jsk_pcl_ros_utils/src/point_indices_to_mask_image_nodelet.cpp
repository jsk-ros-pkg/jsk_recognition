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
#include "jsk_pcl_ros_utils/point_indices_to_mask_image.h"
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace jsk_pcl_ros_utils
{
  void PointIndicesToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("static_image_size", static_image_size_, false);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PointIndicesToMaskImage::subscribe()
  {
    if (static_image_size_) {
      pnh_->getParam("width", width_);
      pnh_->getParam("height", height_);
      sub_input_static_ = pnh_->subscribe("input", 1, &PointIndicesToMaskImage::mask, this);
    }
    else {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_image_.subscribe(*pnh_, "input/image", 1);
      if (approximate_sync_) {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
        async_->connectInput(sub_input_, sub_image_);
        async_->registerCallback(boost::bind(&PointIndicesToMaskImage::mask, this, _1, _2));
      }
      else {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_input_, sub_image_);
        sync_->registerCallback(boost::bind(&PointIndicesToMaskImage::mask,
                                            this, _1, _2));
      }
    }
  }
  
  void PointIndicesToMaskImage::unsubscribe()
  {
    sub_input_.unsubscribe();
    if (!static_image_size_) {
      sub_image_.unsubscribe();
    }
  }

  void PointIndicesToMaskImage::convertAndPublish(
    const PCLIndicesMsg::ConstPtr& indices_msg,
    const int width,
    const int height)
  {
    cv::Mat mask_image = cv::Mat::zeros(height, width, CV_8UC1);
    for (size_t i = 0; i < indices_msg->indices.size(); i++) {
      int index = indices_msg->indices[i];
      if (index >= height * width || index < 0) {
        ROS_ERROR("Input index is out of expected mask size.");
        return;
      }
      int width_index = index % width;
      int height_index = index / width;
      mask_image.at<uchar>(height_index, width_index) = 255;
    }
    cv_bridge::CvImage mask_bridge(indices_msg->header,
                                   sensor_msgs::image_encodings::MONO8,
                                   mask_image);
    pub_.publish(mask_bridge.toImageMsg());
  }

  void PointIndicesToMaskImage::mask(
    const PCLIndicesMsg::ConstPtr& indices_msg)
  {
    vital_checker_->poke();
    convertAndPublish(indices_msg, width_, height_);
  }

  void PointIndicesToMaskImage::mask(
    const PCLIndicesMsg::ConstPtr& indices_msg,
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    convertAndPublish(indices_msg, image_msg->width, image_msg->height);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PointIndicesToMaskImage, nodelet::Nodelet);
