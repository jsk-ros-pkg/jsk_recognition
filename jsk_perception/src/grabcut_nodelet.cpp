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

#include "jsk_perception/grabcut.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  void GrabCut::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &GrabCut::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_foreground_
      = advertise<sensor_msgs::Image>(*pnh_, "output/foreground", 1);
    pub_background_
      = advertise<sensor_msgs::Image>(*pnh_, "output/background", 1);
    pub_foreground_mask_
      = advertise<sensor_msgs::Image>(*pnh_, "output/foreground_mask", 1);
    pub_background_mask_
      = advertise<sensor_msgs::Image>(*pnh_, "output/background_mask", 1);
    onInitPostProcess();
  }

  void GrabCut::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_foreground_.subscribe(*pnh_, "input/foreground", 1);
    sub_background_.subscribe(*pnh_, "input/background", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_image_, sub_foreground_, sub_background_);
    sync_->registerCallback(boost::bind(&GrabCut::segment,
                                        this, _1, _2, _3));
  }

  void GrabCut::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_foreground_.unsubscribe();
    sub_background_.unsubscribe();
  }

  void GrabCut::updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    // foreground and background is not continuous,
    // so just say OK
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                 "GrabCut running");
  }

  void GrabCut::segment(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& foreground_msg,
    const sensor_msgs::Image::ConstPtr& background_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat input;
    cv::Mat in_image = cv_bridge::toCvShare(image_msg,
                                            image_msg->encoding)->image;
    if (in_image.channels() == 3) {
      input = in_image;
    }
    else if (in_image.channels() == 1) {
      input = cv::Mat::zeros(in_image.rows, in_image.cols, CV_8UC3);
      for (size_t j = 0; j < in_image.rows; ++j) {
        for (size_t i = 0; i < in_image.cols; ++i) {
          input.at<cv::Vec3b>(j, i) = cv::Vec3b(in_image.at<uchar>(j, i),
                                                in_image.at<uchar>(j, i),
                                                in_image.at<uchar>(j, i));
        }
      }
    }
    cv::Mat foreground = cv_bridge::toCvCopy(
      foreground_msg, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat background = cv_bridge::toCvCopy(
      background_msg, sensor_msgs::image_encodings::MONO8)->image;
    if (!(input.cols == foreground.cols &&
          input.rows == foreground.rows &&
          background.cols == foreground.cols &&
          background.rows == foreground.rows)) {
      NODELET_WARN("size of image is not corretct");
      return;
    }
    cv::Mat mask = cv::Mat::zeros(input.size(), CV_8UC1);
    mask.setTo(cv::Scalar::all(cv::GC_PR_BGD));
    for (size_t j = 0; j < input.rows; j++) {
      for (size_t i = 0; i < input.cols; i++) {
        if (foreground.at<uchar>(j, i) == 255) {
          if (use_probable_pixel_seed_) {
            mask.at<uchar>(j, i) = cv::GC_PR_FGD;
          }
          else {
            mask.at<uchar>(j, i) = cv::GC_FGD;
          }
        }
        if (background.at<uchar>(j, i) == 255) {
          if (use_probable_pixel_seed_) {
            mask.at<uchar>(j, i) = cv::GC_PR_BGD;
          }
          else {
            mask.at<uchar>(j, i) = cv::GC_BGD;
          }
        }
      }
    }
    cv::Rect roi;
    cv::Mat bgd_model;
    cv::Mat fgd_model;
    
    cv::grabCut(input, mask, roi, bgd_model, fgd_model, 5, cv::GC_INIT_WITH_MASK);
    cv::Mat bgd, fgd, bgd_mask, fgd_mask;
    // model -> mask
    //cv::compare(mask, cv::GC_BGD, bgd_mask, cv::CMP_EQ);
    bgd_mask = (mask == cv::GC_BGD) | (mask == cv::GC_PR_BGD);
    //cv::compare(mask, cv::GC_FGD, fgd_mask, cv::CMP_EQ);
    fgd_mask = (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD);
    // mask -> image
    input.copyTo(bgd, bgd_mask);
    input.copyTo(fgd, fgd_mask);
    cv_bridge::CvImage fg_bridge(
      image_msg->header, sensor_msgs::image_encodings::BGR8, fgd);
    cv_bridge::CvImage bg_bridge(
      image_msg->header, sensor_msgs::image_encodings::BGR8, bgd);
    cv_bridge::CvImage fg_mask_bridge(
      image_msg->header, sensor_msgs::image_encodings::MONO8, fgd_mask);
    cv_bridge::CvImage bg_mask_bridge(
      image_msg->header, sensor_msgs::image_encodings::MONO8, bgd_mask);
    pub_foreground_.publish(fg_bridge.toImageMsg());
    pub_background_.publish(bg_bridge.toImageMsg());
    pub_foreground_mask_.publish(fg_mask_bridge.toImageMsg());
    pub_background_mask_.publish(bg_mask_bridge.toImageMsg());
  }

  void GrabCut::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    use_probable_pixel_seed_ = (config.seed_pixel_policy == 1);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::GrabCut, nodelet::Nodelet);
