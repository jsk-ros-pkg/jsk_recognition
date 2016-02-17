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

#include "jsk_perception/single_channel_histogram.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void SingleChannelHistogram::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("use_mask", use_mask_, false);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &SingleChannelHistogram::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void SingleChannelHistogram::subscribe()
  {
    ros::V_string names;
    if (use_mask_) {
      sub_image_.subscribe(*pnh_, "input", 1);
      sub_mask_.subscribe(*pnh_, "input/mask", 1);
      names.push_back("~input");
      names.push_back("~input/mask");
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image_, sub_mask_);
      sync_->registerCallback(boost::bind(&SingleChannelHistogram::compute,
                                          this, _1, _2));
    }
    else {
      sub_ = pnh_->subscribe("input", 1,
                             &SingleChannelHistogram::compute,
                             this);
      names.push_back("~input");
    }
    jsk_topic_tools::warnNoRemap(names);
  }

  void SingleChannelHistogram::unsubscribe()
  {
    if (use_mask_) {
      sub_image_.unsubscribe();
      sub_mask_.unsubscribe();
    }
    else {
      sub_.shutdown();
    }
  }

  void SingleChannelHistogram::compute(
    const sensor_msgs::Image::ConstPtr& msg,
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::Mat mask;
    if (mask_msg) {
      mask = cv_bridge::toCvCopy(mask_msg, mask_msg->encoding)->image;
    }
    float range[] = { min_value_, max_value_ } ;
    const float* histRange = { range };
    cv::MatND hist;
    bool uniform = true; bool accumulate = false;
    
    cv::calcHist(&image, 1, 0, mask, hist, 1, &hist_size_,
                 &histRange, uniform, accumulate);
    jsk_recognition_msgs::ColorHistogram histogram;
    histogram.header = msg->header;
    for (int i = 0; i < hist_size_; i++) {
      histogram.histogram.push_back(hist.at<float>(0, i));
    }
    pub_.publish(histogram);
  }
  
  void SingleChannelHistogram::compute(
    const sensor_msgs::Image::ConstPtr& msg)
  {
    compute(msg, sensor_msgs::Image::ConstPtr());
  }

  void SingleChannelHistogram::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    hist_size_ = config.hist_size;
    min_value_ = config.min_value;
    max_value_ = config.max_value;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::SingleChannelHistogram, nodelet::Nodelet);
