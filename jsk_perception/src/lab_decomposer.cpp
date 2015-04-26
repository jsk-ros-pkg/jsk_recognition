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


#include "jsk_perception/lab_decomposer.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  void LabDecomposer::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_l_ = advertise<sensor_msgs::Image>(*pnh_, "output/l", 1);
    pub_a_ = advertise<sensor_msgs::Image>(*pnh_, "output/a", 1);
    pub_b_ = advertise<sensor_msgs::Image>(*pnh_, "output/b", 1);
  }

  void LabDecomposer::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &LabDecomposer::decompose, this);
  }

  void LabDecomposer::unsubscribe()
  {
    sub_.shutdown();
  }

  void LabDecomposer::decompose(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, image_msg->encoding);
    cv::Mat image = cv_ptr->image;
    cv::Mat lab_image;
    std::vector<cv::Mat> lab_planes;
    if (image_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::cvtColor(image, lab_image, CV_BGR2Lab);
    }
    else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(image, lab_image, CV_RGB2Lab);
    }
    else {
      NODELET_ERROR("unsupported format to Lab: %s", image_msg->encoding.c_str());
      return;
    }
    cv::split(lab_image, lab_planes);
    cv::Mat l = lab_planes[0];
    cv::Mat a = lab_planes[1];
    cv::Mat b = lab_planes[2];
    pub_l_.publish(cv_bridge::CvImage(
                     image_msg->header,
                     sensor_msgs::image_encodings::MONO8,
                     l).toImageMsg());
    pub_a_.publish(cv_bridge::CvImage(
                     image_msg->header,
                     sensor_msgs::image_encodings::MONO8,
                     a).toImageMsg());
    pub_b_.publish(cv_bridge::CvImage(
                     image_msg->header,
                     sensor_msgs::image_encodings::MONO8,
                     b).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::LabDecomposer, nodelet::Nodelet);
