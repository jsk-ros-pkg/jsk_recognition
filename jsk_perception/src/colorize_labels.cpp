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

#include "jsk_perception/colorize_labels.h"
#include <jsk_recognition_utils/cv_utils.h>
#include <jsk_topic_tools/log_utils.h>
#include <boost/assign.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void ColorizeLabels::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void ColorizeLabels::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ColorizeLabels::colorize, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ColorizeLabels::unsubscribe()
  {
    sub_.shutdown();
  }

  void ColorizeLabels::colorize(
    const sensor_msgs::Image::ConstPtr& label_image_msg)
  {
    cv::Mat label_image = cv_bridge::toCvShare(
      label_image_msg, label_image_msg->encoding)->image;
    ROS_DEBUG("%dx%d", label_image_msg->width, label_image_msg->height);
    cv::Mat output_image;
    jsk_recognition_utils::labelToRGB(label_image, output_image);
    cv::cvtColor(output_image, output_image, CV_RGB2BGR);
    pub_.publish(
      cv_bridge::CvImage(
        label_image_msg->header,
        sensor_msgs::image_encodings::BGR8,
        output_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ColorizeLabels, nodelet::Nodelet);
