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

#include "jsk_perception/smoothing_filter.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_perception {

  /*** Bilateral Filter ***/
  void BilateralFilter::onInit() {
    filter_size_ = 7;
    sigma_color_ = 35;
    sigma_space_ = 5;
    SmoothingFilter::onInit();
  }

  void BilateralFilter::configCallback(
    jsk_perception::BilateralFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    filter_size_ = config.filter_size;
    sigma_color_ = config.sigma_color;
    sigma_space_ = config.sigma_space;
  }

  void BilateralFilter::process(const cv::Mat& input_image,
                                const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::bilateralFilter(input_image, filtered_image, filter_size_, sigma_color_,
                        sigma_space_);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Gaussian Filter ***/
  void GaussianFilter::onInit() {
    kernel_size_ = 1;
    sigma_x_ = 10;
    sigma_y_ = 10;
    SmoothingFilter::onInit();
  }

  void GaussianFilter::configCallback(
    jsk_perception::GaussianFilterConfig& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    kernel_size_ = config.kernel_size * 2 + 1;
    sigma_x_ = config.sigma_x;
    sigma_y_ = config.sigma_y;
  }

  void GaussianFilter::process(const cv::Mat& input_image,
                               const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::GaussianBlur(input_image, filtered_image,
                     cv::Size(kernel_size_, kernel_size_), sigma_x_, sigma_y_);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Median Filter ***/
  void MedianFilter::onInit() {
    kernel_size_ = 1;
    SmoothingFilter::onInit();
  }

  void MedianFilter::configCallback(jsk_perception::MedianFilterConfig& config,
                                    uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    kernel_size_ = config.kernel_size * 2 + 1;
  }

  void MedianFilter::process(const cv::Mat& input_image,
                             const std_msgs::Header& header) {
    cv::Mat filtered_image;
    cv::medianBlur(input_image, filtered_image, kernel_size_);
    pub_.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8,
                                    filtered_image).toImageMsg());
  }

  /*** Smoothing Filter ***/
  template <typename Config>
  void SmoothingFilter<Config>::filter(
    const sensor_msgs::Image::ConstPtr& input_image_msg) {
    boost::mutex::scoped_lock lock(mutex_);
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        input_image_msg, sensor_msgs::image_encodings::BGR8);
      process(cv_ptr->image, input_image_msg->header);
    } catch (cv_bridge::Exception& e) {
      JSK_NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  template <typename Config>
  void SmoothingFilter<Config>::onInit() {
    DiagnosticNodelet::onInit();
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&SmoothingFilter::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  template <typename Config>
  void SmoothingFilter<Config>::subscribe() {
    sub_ = pnh_->subscribe("input", 1, &SmoothingFilter::filter, this);
  }

  template <typename Config>
  void SmoothingFilter<Config>::unsubscribe() {
    sub_.shutdown();
  }
}

typedef jsk_perception::BilateralFilter BilateralFilter;
typedef jsk_perception::GaussianFilter GaussianFilter;
typedef jsk_perception::MedianFilter MedianFilter;
PLUGINLIB_EXPORT_CLASS(jsk_perception::BilateralFilter, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(jsk_perception::GaussianFilter, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(jsk_perception::MedianFilter, nodelet::Nodelet);
