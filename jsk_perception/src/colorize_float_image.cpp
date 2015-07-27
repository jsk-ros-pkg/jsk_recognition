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

#include "jsk_perception/colorize_float_image.h"
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void ColorizeFloatImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void ColorizeFloatImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ColorizeFloatImage::colorize, this);
  }

  void ColorizeFloatImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void ColorizeFloatImage::colorize(const sensor_msgs::Image::ConstPtr& msg)
  {
    cv::Mat float_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    cv::Mat color_image = cv::Mat(float_image.rows, float_image.cols, CV_8UC3);
    float min_value = FLT_MAX;
    float max_value = - FLT_MAX;
    for (size_t j = 0; j < float_image.rows; j++) {
      for (size_t i = 0; i < float_image.cols; i++) {
        float v = float_image.at<float>(j, i);
        if (v != - FLT_MAX) {
          min_value = std::min(v, min_value);
          max_value = std::max(v, max_value);
        }
      }
    }
    for (size_t j = 0; j < float_image.rows; j++) {
      for (size_t i = 0; i < float_image.cols; i++) {
        float v = float_image.at<float>(j, i);
        if (v != - FLT_MAX) {
          std_msgs::ColorRGBA c = jsk_topic_tools::heatColor((v - min_value) / (max_value - min_value));
          color_image.at<cv::Vec3b>(j, i) = cv::Vec3b(c.r * 255, c.g * 255, c.b * 255);
        }
        else {
          color_image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
        }
      }
    }
    pub_.publish(cv_bridge::CvImage(msg->header,
                                    sensor_msgs::image_encodings::RGB8,
                                    color_image).toImageMsg());
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ColorizeFloatImage, nodelet::Nodelet);
