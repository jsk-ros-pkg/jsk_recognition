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

#include "jsk_perception/color_filter.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_perception {

  /*** RGB ***/
  void RGBColorFilter::onInit() {
    r_max_ = 255;
    r_min_ = 0;
    g_max_ = 255;
    g_min_ = 0;
    b_max_ = 255;
    b_min_ = 0;

    ColorFilter::onInit();
  }

  void RGBColorFilter::updateCondition() {
    if (r_max_ < r_min_) std::swap(r_max_, r_min_);
    if (g_max_ < g_min_) std::swap(g_max_, g_min_);
    if (b_max_ < b_min_) std::swap(b_max_, b_min_);
    lower_color_range_ = cv::Scalar(b_min_, g_min_, r_min_);
    upper_color_range_ = cv::Scalar(b_max_, g_max_, r_max_);
  }

  void RGBColorFilter::configCallback(
    jsk_perception::RGBColorFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    r_max_ = config.r_limit_max;
    r_min_ = config.r_limit_min;
    g_max_ = config.g_limit_max;
    g_min_ = config.g_limit_min;
    b_max_ = config.b_limit_max;
    b_min_ = config.b_limit_min;
    updateCondition();
  }

  void RGBColorFilter::process(const cv::Mat& input_image,
                               const std_msgs::Header& header) {
    cv::Mat output_image;
    cv::inRange(input_image, lower_color_range_, upper_color_range_,
                output_image);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
                                    output_image).toImageMsg());
  }

  /*** HSI ***/
  void HSIColorFilter::onInit() {
    h_max_ = 255;
    h_min_ = 0;
    s_max_ = 255;
    s_min_ = 0;
    i_max_ = 255;
    i_min_ = 0;

    ColorFilter::onInit();
  }

  void HSIColorFilter::updateCondition() {
    if (h_max_ < h_min_) std::swap(h_max_, h_min_);
    if (s_max_ < s_min_) std::swap(s_max_, s_min_);
    if (i_max_ < i_min_) std::swap(i_max_, i_min_);
    lower_color_range_ = cv::Scalar(h_min_, s_min_, i_min_);
    upper_color_range_ = cv::Scalar(h_max_, s_max_, i_max_);
  }

  void HSIColorFilter::configCallback(
    jsk_perception::HSIColorFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    h_max_ = config.h_limit_max;
    h_min_ = config.h_limit_min;
    s_max_ = config.s_limit_max;
    s_min_ = config.s_limit_min;
    i_max_ = config.i_limit_max;
    i_min_ = config.i_limit_min;
    updateCondition();
  }

  void HSIColorFilter::process(const cv::Mat& input_image,
                               const std_msgs::Header& header) {
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat output_image;
    cv::inRange(hsv_image, lower_color_range_, upper_color_range_,
                output_image);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,
                                    output_image).toImageMsg());
  }

  /*** ColorFilter ***/
  template <typename Config>
  void ColorFilter<Config>::filter(
    const sensor_msgs::Image::ConstPtr& input_image_msg) {
    boost::mutex::scoped_lock lock(mutex_);
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        input_image_msg, sensor_msgs::image_encodings::BGR8);
      // cv::Mat bgr_image = cv_ptr->image;
      // process(bgr_image);
      process(cv_ptr->image, input_image_msg->header);
    } catch (cv_bridge::Exception& e) {
      JSK_NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  template <typename Config>
  void ColorFilter<Config>::onInit() {
    DiagnosticNodelet::onInit();
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ColorFilter::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  template <typename Config>
  void ColorFilter<Config>::subscribe() {
    sub_ = pnh_->subscribe("input", 1, &ColorFilter::filter, this);
  }

  template <typename Config>
  void ColorFilter<Config>::unsubscribe() {
    sub_.shutdown();
  }
}

typedef jsk_perception::RGBColorFilter RGBColorFilter;
typedef jsk_perception::HSIColorFilter HSIColorFilter;
PLUGINLIB_EXPORT_CLASS(jsk_perception::RGBColorFilter, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(jsk_perception::HSIColorFilter, nodelet::Nodelet);
