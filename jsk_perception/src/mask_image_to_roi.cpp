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

#include "jsk_perception/mask_image_to_roi.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void MaskImageToROI::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void MaskImageToROI::subscribe()
  {
    sub_mask_ = pnh_->subscribe("input", 1, &MaskImageToROI::convert, this);
    sub_info_ = pnh_->subscribe("input/camera_info", 1,
                                &MaskImageToROI::infoCallback, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/camera_info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void MaskImageToROI::unsubscribe()
  {
    sub_mask_.shutdown();
    sub_info_.shutdown();
  }

  void MaskImageToROI::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    latest_camera_info_ = info_msg;
  }

  void MaskImageToROI::convert(
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    if (latest_camera_info_) {
      sensor_msgs::CameraInfo camera_info(*latest_camera_info_);
      std::vector<cv::Point> indices;
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        mask_msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat mask = cv_ptr->image;
      for (size_t j = 0; j < mask.rows; j++) {
        for (size_t i = 0; i < mask.cols; i++) {
          if (mask.at<uchar>(j, i) == 255) {
            indices.push_back(cv::Point(i, j));
          }
        }
      }
      cv::Rect mask_rect = cv::boundingRect(indices);
      camera_info.roi.x_offset = mask_rect.x;
      camera_info.roi.y_offset = mask_rect.y;
      camera_info.roi.width = mask_rect.width;
      camera_info.roi.height = mask_rect.height;
      camera_info.header = mask_msg->header;
      pub_.publish(camera_info);
    }
    else {
      NODELET_ERROR("camera info is not yet available");
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::MaskImageToROI, nodelet::Nodelet);
