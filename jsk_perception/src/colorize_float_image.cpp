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
#include <boost/assign.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void ColorizeFloatImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pnh_->param("channel", channel_, 0);
    onInitPostProcess();
  }

  void ColorizeFloatImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ColorizeFloatImage::colorize, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ColorizeFloatImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void ColorizeFloatImage::colorize(const sensor_msgs::Image::ConstPtr& msg)
  {
    int numchannels = sensor_msgs::image_encodings::numChannels(msg->encoding);
    if (numchannels < channel_) {
      ROS_ERROR("Image Channel(%s) %d is less than parameter channel (%d)",
                msg->encoding.c_str(),
                sensor_msgs::image_encodings::numChannels(msg->encoding),
                channel_);
    }
    cv::Mat float_image = cv_bridge::toCvShare(msg)->image;
    cv::Mat color_image = cv::Mat(float_image.rows, float_image.cols, CV_8UC3);

    if (channel_ != 0 || numchannels > 1) {
      std::vector<cv::Mat> planes;
      cv::split(float_image, planes);
      float_image = planes[channel_];
    }

    double min_value;
    double max_value;
    cv::minMaxLoc(float_image, &min_value, &max_value);

    for (size_t j = 0; j < float_image.rows; j++) {
      for (size_t i = 0; i < float_image.cols; i++) {
        float v = float_image.at<float>(j, i);
        if (std::isnan(v)) {
          color_image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
        }
        else {
          std_msgs::ColorRGBA c = jsk_topic_tools::heatColor((v - min_value) / (max_value - min_value));
          color_image.at<cv::Vec3b>(j, i) = cv::Vec3b(c.r * 255, c.g * 255, c.b * 255);
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
