// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include "jsk_perception/overlay_image_color_on_mono.h"
#include <boost/assign.hpp>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_utils/cv_utils.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void OverlayImageColorOnMono::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &OverlayImageColorOnMono::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void OverlayImageColorOnMono::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    color_alpha_ = config.color_alpha;
  }

  void OverlayImageColorOnMono::subscribe()
  {
    sub_color_.subscribe(*pnh_, "input/color", 1);
    sub_mono_.subscribe(*pnh_, "input/mono", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(sub_color_, sub_mono_);
      async_->registerCallback(boost::bind(&OverlayImageColorOnMono::overlay, this, _1, _2));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_color_, sub_mono_);
      sync_->registerCallback(boost::bind(&OverlayImageColorOnMono::overlay, this, _1, _2));
    }
    ros::V_string names = boost::assign::list_of("~input/color")("~input/mono");
    jsk_topic_tools::warnNoRemap(names);
  }

  void OverlayImageColorOnMono::unsubscribe()
  {
    sub_color_.unsubscribe();
    sub_mono_.unsubscribe();
  }

  void OverlayImageColorOnMono::overlay(const sensor_msgs::Image::ConstPtr& color_imgmsg,
                                        const sensor_msgs::Image::ConstPtr& mono_imgmsg)
  {
    // validate image channels
    if (sensor_msgs::image_encodings::numChannels(color_imgmsg->encoding) != 3) {
      NODELET_ERROR_THROTTLE(10, "Input ~image/color message must be 3 channels color image. (RGB/BGR).");
      return;
    }
    // validate image size
    if (! ((color_imgmsg->height == mono_imgmsg->height) && (color_imgmsg->width == mono_imgmsg->width))) {
      NODELET_ERROR_THROTTLE(
          10, "The size of input color and mono image is different: (color: h=%d w=%d), (mono: h=%d w=%d)",
          color_imgmsg->height, color_imgmsg->width, mono_imgmsg->height, mono_imgmsg->width);
      return;
    }

    boost::mutex::scoped_lock lock(mutex_);

    cv::Mat color_image = cv_bridge::toCvShare(color_imgmsg, color_imgmsg->encoding)->image;
    cv::Mat mono_image = cv_bridge::toCvShare(mono_imgmsg, sensor_msgs::image_encodings::MONO8)->image;

    cv::Mat overlayed_image = cv::Mat::zeros(color_image.rows, color_image.cols, CV_8UC3);
    for (size_t j = 0; j < overlayed_image.rows; j++) {
      for (size_t i = 0; i < overlayed_image.cols; i++) {
        cv::Vec3b color = color_image.at<cv::Vec3b>(j, i);
        uchar mono = mono_image.at<uchar>(j, i);
        overlayed_image.at<cv::Vec3b>(j, i) = cv::Vec3b(color[0] * color_alpha_ + mono * (1 - color_alpha_),
                                                        color[1] * color_alpha_ + mono * (1 - color_alpha_),
                                                        color[2] * color_alpha_ + mono * (1 - color_alpha_));
      }
    }
    pub_.publish(cv_bridge::CvImage(color_imgmsg->header,
                                    color_imgmsg->encoding,
                                    overlayed_image).toImageMsg());
  }

}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::OverlayImageColorOnMono, nodelet::Nodelet);
