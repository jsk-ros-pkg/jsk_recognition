// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
/*
 * subtract_mask_image.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_perception/subtract_mask_image.h>
#include <boost/assign.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception
{
  void SubtractMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void SubtractMaskImage::subscribe()
  {
    sub_src1_.subscribe(*pnh_, "input/src1", 1);
    sub_src2_.subscribe(*pnh_, "input/src2", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(queue_size_);
      async_->connectInput(sub_src1_, sub_src2_);
      async_->registerCallback(boost::bind(&SubtractMaskImage::subtract, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_src1_, sub_src2_);
      sync_->registerCallback(boost::bind(&SubtractMaskImage::subtract, this, _1, _2));
    }
  }

  void SubtractMaskImage::unsubscribe()
  {
    sub_src1_.unsubscribe();
    sub_src2_.unsubscribe();
  }

  void SubtractMaskImage::subtract(
    const sensor_msgs::Image::ConstPtr& src1_msg,
    const sensor_msgs::Image::ConstPtr& src2_msg)
  {
    vital_checker_->poke();

    if (src1_msg->width != src2_msg->width || src1_msg->height != src2_msg->height)
    {
      NODELET_ERROR("Size of masks are different!");
      NODELET_ERROR("input/src1 = %dx%d", src1_msg->width, src1_msg->height);
      NODELET_ERROR("input/src2 = %dx%d", src2_msg->width, src2_msg->height);
      return;
    }

    cv::Mat mask1 = cv_bridge::toCvShare(src1_msg, enc::MONO8)->image;
    cv::Mat mask2 = cv_bridge::toCvShare(src2_msg, enc::MONO8)->image;

    cv::Mat mask2_not;
    cv::bitwise_not(mask2, mask2_not);

    cv::Mat result = cv::Mat::zeros(mask1.size(), mask1.type());
    mask1.copyTo(result, mask2_not);
    pub_.publish(
      cv_bridge::CvImage(src1_msg->header, enc::MONO8, result).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::SubtractMaskImage, nodelet::Nodelet);
