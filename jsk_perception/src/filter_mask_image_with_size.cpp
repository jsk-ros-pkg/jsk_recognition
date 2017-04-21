/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include "jsk_perception/filter_mask_image_with_size.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void FilterMaskImageWithSize::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("use_reference", use_reference_, false);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&FilterMaskImageWithSize::configCallback, this, _1, _2);
    srv_->setCallback(f);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void FilterMaskImageWithSize::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_size_ = config.min_size;
    max_size_ = config.max_size;
    if (use_reference_)
    {
      min_relative_size_ = config.min_relative_size;
      max_relative_size_ = config.max_relative_size;
    }
    else
    {
      if ((config.min_relative_size != 0) || (config.max_relative_size != 1))
      {
        ROS_WARN("Rosparam ~min_relative_size and ~max_relative_size is enabled only with ~use_reference is true,"
                 " and will be overwritten by 0 and 1.");
      }
      min_relative_size_ = config.min_relative_size = 0;
      max_relative_size_ = config.max_relative_size = 1;
    }
  }

  void FilterMaskImageWithSize::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    ros::V_string names = boost::assign::list_of("~input");
    if (use_reference_)
    {
      sub_reference_.subscribe(*pnh_, "input/reference", 1);
      if (approximate_sync_)
      {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(queue_size_);
        async_->connectInput(sub_input_, sub_reference_);
        async_->registerCallback(boost::bind(&FilterMaskImageWithSize::filter, this, _1, _2));
      }
      else
      {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_input_, sub_reference_);
        sync_->registerCallback(boost::bind(&FilterMaskImageWithSize::filter, this, _1, _2));
      }
      names.push_back("~input/reference");
    }
    else
    {
      sub_input_.registerCallback(&FilterMaskImageWithSize::filter, this);
    }
    jsk_topic_tools::warnNoRemap(names);
  }

  void FilterMaskImageWithSize::unsubscribe()
  {
    sub_input_.unsubscribe();
    if (use_reference_)
    {
      sub_reference_.unsubscribe();
    }
  }

  void FilterMaskImageWithSize::filter(
    const sensor_msgs::Image::ConstPtr& input_msg)
  {
    filter(input_msg, input_msg);
  }

  void FilterMaskImageWithSize::filter(
    const sensor_msgs::Image::ConstPtr& input_msg,
    const sensor_msgs::Image::ConstPtr& reference_msg)
  {
    if ((input_msg->height != reference_msg->height) ||
        (input_msg->width != reference_msg->width))
    {
      ROS_FATAL("Input mask size must match. input: (%d, %d), reference: (%d, %d)",
                input_msg->height, input_msg->width,
                reference_msg->height, reference_msg->width);
      return;
    }
    cv::Mat input = cv_bridge::toCvShare(input_msg, input_msg->encoding)->image;
    cv::Mat reference = cv_bridge::toCvShare(reference_msg, reference_msg->encoding)->image;
    double size_image = input_msg->height * input_msg->width;
    double size_input = cv::countNonZero(input > 127) / size_image;
    double size_reference = cv::countNonZero(reference > 127) / size_image;
    double size_relative = size_input / size_reference;
    ROS_INFO("image relative: %lf <= %lf <= %lf, mask relative: %lf <= %lf <= %lf",
             min_size_, size_input, max_size_,
             min_relative_size_, size_relative, max_relative_size_);
    if (!std::isnan(size_relative) &&
        (min_size_ <= size_input) && (size_input <= max_size_) &&
        (min_relative_size_ <= size_relative) && (size_relative <= max_relative_size_))
    {
      pub_.publish(input_msg);
    }
  }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::FilterMaskImageWithSize, nodelet::Nodelet);
