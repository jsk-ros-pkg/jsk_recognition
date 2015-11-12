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


#include "jsk_perception/morphological_operator.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{

  void MorphologicalImageOperatorNodelet::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &MorphologicalImageOperatorNodelet::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void MorphologicalImageOperatorNodelet::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", 1, &MorphologicalImageOperatorNodelet::imageCallback, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void MorphologicalImageOperatorNodelet::unsubscribe()
  {
    sub_.shutdown();
  }

  void MorphologicalImageOperatorNodelet::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    method_ = config.method;
    size_ = config.size;
    iterations_ = config.iterations;
  }

  void MorphologicalImageOperatorNodelet::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8)->image;
    int type;
    if (method_ == 0) {
      type = cv::MORPH_RECT;
    }
    else if (method_ == 1) {
      type = cv::MORPH_CROSS;
    }
    else if (method_ == 2) {
      type = cv::MORPH_ELLIPSE;
    }
    cv::Mat output_image;

    cv::Mat element = cv::getStructuringElement(
      type,
      cv::Size(2 * size_ + 1, 2 * size_+1),
      cv::Point(size_, size_));
    apply(image, output_image, element);
    pub_.publish(
      cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::MONO8,
                         output_image).toImageMsg());
  }

  void MorphologicalImageOperator::apply(
    const cv::Mat& input, cv::Mat& output, const cv::Mat& element)
  {
    cv::morphologyEx(input, output, operation_, element,
                     /*anchor=*/cv::Point(-1,-1), iterations_);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::Dilate, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_perception::Erode, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_perception::Opening, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_perception::Closing, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_perception::MorphologicalGradient, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_perception::TopHat, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_perception::BlackHat, nodelet::Nodelet);
