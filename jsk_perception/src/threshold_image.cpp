// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) JSK, 2016 Lab
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

#include <jsk_perception/threshold_image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception {
  void ThresholdImage::onInit() {
    DiagnosticNodelet::onInit();

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ThresholdImage::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ThresholdImage::subscribe() {
    sub_image_ = pnh_->subscribe("input", 1, &ThresholdImage::binarize, this);
  }

  void ThresholdImage::unsubscribe() { sub_image_.shutdown(); }

  void ThresholdImage::configCallback(Config& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    threshold_value_ = config.threshold;
    threshold_type_ = config.threshold_type;
    max_binary_value_ = config.max_binary;
    apply_otsu_ = config.apply_otsu;
  }

  void ThresholdImage::binarize(const sensor_msgs::Image::ConstPtr& image_msg) {
    cv::Mat src_image =
      cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
    cv::Mat gray_image;
    cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Mat result_image;

    if (apply_otsu_) {
      threshold_type_ |= CV_THRESH_OTSU;
    }
    cv::threshold(gray_image, result_image, threshold_value_, max_binary_value_,
                  threshold_type_);
    pub_image_.publish(cv_bridge::CvImage(image_msg->header,
                                          sensor_msgs::image_encodings::MONO8,
                                          result_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::ThresholdImage, nodelet::Nodelet);
