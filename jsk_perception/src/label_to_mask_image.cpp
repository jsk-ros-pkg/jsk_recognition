// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
#include "jsk_perception/label_to_mask_image.h"
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/assign.hpp>

namespace jsk_perception
{

  void LabelToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    // dynamic_reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&LabelToMaskImage::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void LabelToMaskImage::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    label_value_ = config.label_value;
  }

  void LabelToMaskImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1,
                           &LabelToMaskImage::convert,
                           this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void LabelToMaskImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void LabelToMaskImage::convert(
    const sensor_msgs::Image::ConstPtr& label_msg)
  {
    cv_bridge::CvImagePtr label_img_ptr = cv_bridge::toCvCopy(
      label_msg, sensor_msgs::image_encodings::TYPE_32SC1);

    cv::Mat mask_image = cv::Mat::zeros(label_msg->height,
                                        label_msg->width,
                                        CV_8UC1);
    for (size_t j = 0; j < label_img_ptr->image.rows; j++)
    {
      for (size_t i = 0; i < label_img_ptr->image.cols; i++)
      {
        int label = label_img_ptr->image.at<int>(j, i);
        if (label == label_value_) {
          mask_image.at<uchar>(j, i) = 255;
        }
      }
    }
    pub_.publish(cv_bridge::CvImage(
          label_msg->header,
          sensor_msgs::image_encodings::MONO8,
          mask_image).toImageMsg());
  }

}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::LabelToMaskImage, nodelet::Nodelet);
