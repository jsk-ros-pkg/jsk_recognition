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

#include "jsk_perception/apply_mask_image.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "jsk_perception/image_utils.h"

namespace jsk_perception
{
  void ApplyMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("mask_black_to_transparent", mask_black_to_transparent_, false);
    pub_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    pub_mask_ = advertise<sensor_msgs::Image>(
      *pnh_, "output/mask", 1);
  }

  void ApplyMaskImage::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_mask_.subscribe(*pnh_, "input/mask", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_image_, sub_mask_);
      async_->registerCallback(boost::bind(&ApplyMaskImage::apply, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image_, sub_mask_);
      sync_->registerCallback(boost::bind(&ApplyMaskImage::apply, this, _1, _2));
    }
  }

  void ApplyMaskImage::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_mask_.unsubscribe();
  }

  void ApplyMaskImage::apply(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    vital_checker_->poke();
    cv::Mat image;
    if (isBGRA(image_msg->encoding)) {
      cv::Mat tmp_image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
      cv::cvtColor(tmp_image, image, cv::COLOR_BGRA2BGR);
    }
    else if (isRGBA(image_msg->encoding)) {
      cv::Mat tmp_image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
      cv::cvtColor(tmp_image, image, cv::COLOR_RGBA2BGR);
    }
    else {  // BGR, RGB or GRAY
      image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
    }
    cv::Mat mask = cv_bridge::toCvShare(mask_msg, "mono8")->image;
    if (image.cols != mask.cols || image.rows != mask.rows) {
      JSK_NODELET_ERROR("size of image and mask is different");
      JSK_NODELET_ERROR("image: %dx%dx", image.cols, image.rows);
      JSK_NODELET_ERROR("mask: %dx%dx", mask.cols, mask.rows);
      return;
    }
    
    cv::Rect region = boundingRectOfMaskImage(mask);
    cv::Mat clipped_mask = mask(region);
    pub_mask_.publish(cv_bridge::CvImage(
                        mask_msg->header,
                        "mono8",
                        clipped_mask).toImageMsg());

    cv::Mat clipped_image = image(region);
    cv::Mat masked_image;
    clipped_image.copyTo(masked_image, clipped_mask);

    cv::Mat output_image;
    if (mask_black_to_transparent_) {
      if (sensor_msgs::image_encodings::isMono(image_msg->encoding)) {
        cv::cvtColor(masked_image, output_image, CV_GRAY2BGRA);
      }
      else if (isRGB(image_msg->encoding)) {
        cv::cvtColor(masked_image, output_image, CV_RGB2BGRA);
      }
      else {  // BGR, BGRA or RGBA
        cv::cvtColor(masked_image, output_image, CV_BGR2BGRA);
      }
      for (size_t j=0; j<clipped_mask.rows; j++) {
        for (int i=0; i<clipped_mask.cols; i++) {
          if (clipped_mask.at<uchar>(j, i) == 0) {
            cv::Vec4b color = output_image.at<cv::Vec4b>(j, i);
            color[3] = 0;  // mask black -> transparent
            output_image.at<cv::Vec4b>(j, i) = color;
          }
        }
      }
      // publish bgr8 image
      pub_image_.publish(cv_bridge::CvImage(
            image_msg->header,
            sensor_msgs::image_encodings::BGRA8,
            output_image).toImageMsg());
    }
    else {
      if (isBGRA(image_msg->encoding)) {
        cv::cvtColor(masked_image, output_image, cv::COLOR_BGR2BGRA);
      }
      else if (isRGBA(image_msg->encoding)) {
        cv::cvtColor(masked_image, output_image, cv::COLOR_BGR2RGBA);
      }
      else {  // BGR, RGB or GRAY
        masked_image.copyTo(output_image);
      }
      pub_image_.publish(cv_bridge::CvImage(
            image_msg->header,
            image_msg->encoding,
            output_image).toImageMsg());
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ApplyMaskImage, nodelet::Nodelet);
