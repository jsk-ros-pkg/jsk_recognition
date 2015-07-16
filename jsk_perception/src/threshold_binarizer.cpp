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


#include "jsk_perception/threshold_binarizer.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  ThresholdBinarizer::ThresholdBinarizer(): DiagnosticNodelet("ThresholdBinarizer")
  {
    thre_lower_ = 0;
    thre_upper_ = 255;
    cv::namedWindow ("TrackbarWindow", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar ("TrackbarLower", "TrackbarWindow", &thre_lower_, 256, 0);
    cv::createTrackbar ("TrackbarUpper", "TrackbarWindow", &thre_upper_, 256, 0);
    cv::startWindowThread();
  }

  ThresholdBinarizer::~ThresholdBinarizer()
  {
    cv::destroyWindow("TrackbarWindow");
  }

  void ThresholdBinarizer::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void ThresholdBinarizer::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ThresholdBinarizer::binarize, this);
  }

  void ThresholdBinarizer::unsubscribe()
  {
    sub_.shutdown();
  }

  void ThresholdBinarizer::binarize(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    cv::Mat image = cv_ptr->image;
    cv::Mat binary_image_lower, binary_image_upper, binary_image;

    cv::waitKey(10);
    cv::threshold(image, binary_image_lower, thre_lower_-1, 255, cv::THRESH_BINARY);
    cv::threshold(image, binary_image_upper, thre_upper_-1, 255, cv::THRESH_BINARY_INV);
    cv:bitwise_and(binary_image_lower, binary_image_upper, binary_image);
    pub_.publish(cv_bridge::CvImage(image_msg->header,
                                    sensor_msgs::image_encodings::MONO8,
                                    binary_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ThresholdBinarizer, nodelet::Nodelet);
