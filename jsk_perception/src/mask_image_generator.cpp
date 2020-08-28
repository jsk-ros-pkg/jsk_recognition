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

#include "jsk_perception/mask_image_generator.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif


namespace jsk_perception
{
  void MaskImageGenerator::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&MaskImageGenerator::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void MaskImageGenerator::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &MaskImageGenerator::generate, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void MaskImageGenerator::unsubscribe()
  {
    sub_.shutdown();
  }

  void MaskImageGenerator::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    offset_x_ = config.offset_x;
    offset_y_ = config.offset_y;
    width_ = config.width;
    height_ = config.height;
  }

  void MaskImageGenerator::generate(const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    int min_x = std::min(offset_x_, (int)msg->width);
    int min_y = std::min(offset_y_, (int)msg->height);
    int max_x = std::min(min_x + width_, (int)msg->width);
    int max_y = std::min(min_y + height_, (int)msg->height);
    cv::Mat mask_image = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    cv::rectangle(mask_image,
                  cv::Point(min_x, min_y),
                  cv::Point(max_x, max_y),
                  cv::Scalar(255), CV_FILLED);
    pub_.publish(cv_bridge::CvImage(msg->header,
                                    sensor_msgs::image_encodings::MONO8,
                                    mask_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::MaskImageGenerator, nodelet::Nodelet);
