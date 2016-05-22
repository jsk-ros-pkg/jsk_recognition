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

#include "jsk_perception/unapply_mask_image.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/cv_utils.h>

namespace jsk_perception
{
  void UnapplyMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void UnapplyMaskImage::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_mask_.subscribe(*pnh_, "input/mask", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_image_, sub_mask_);
      async_->registerCallback(boost::bind(&UnapplyMaskImage::apply, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image_, sub_mask_);
      sync_->registerCallback(boost::bind(&UnapplyMaskImage::apply, this, _1, _2));
    }
    ros::V_string names = boost::assign::list_of("~input")("~input/mask");
    jsk_topic_tools::warnNoRemap(names);
  }

  void UnapplyMaskImage::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_mask_.unsubscribe();
  }

  void UnapplyMaskImage::apply(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    vital_checker_->poke();
    cv::Mat image = cv_bridge::toCvShare(image_msg,
                                         image_msg->encoding)->image;
    cv::Mat mask = cv_bridge::toCvShare(mask_msg,
                                        mask_msg->encoding)->image;
    cv::Mat output;
    bool single_channel = false;
    if (image_msg->encoding == sensor_msgs::image_encodings::BGR8 ||
        image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      single_channel = false;
    }
    else {
      single_channel = true;
    }
    if (single_channel) {
      output = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
    }
    else {
      output = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    }
    
    cv::Rect region = jsk_recognition_utils::boundingRectOfMaskImage(mask);
    for (int j = 0; j < image.rows; j++) {
      for (int i = 0; i < image.cols; i++) {
        if (single_channel) {
          output.at<uchar>(j + region.y, i + region.x)
            = image.at<uchar>(j, i);
        }
        else {
          output.at<cv::Vec3b>(j + region.y, i + region.x)
            = image.at<cv::Vec3b>(j, i);
        }
      }
    }
    pub_image_.publish(cv_bridge::CvImage(
                         image_msg->header,
                         image_msg->encoding,
                         output).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::UnapplyMaskImage, nodelet::Nodelet);
