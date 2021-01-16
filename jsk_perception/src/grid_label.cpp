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

#include "jsk_perception/grid_label.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif

namespace jsk_perception
{
  void GridLabel::onInit()
  {
    DiagnosticNodelet::onInit();
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&GridLabel::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    pnh_->param("use_camera_info", use_camera_info_, false);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void GridLabel::subscribe()
  {
    if (use_camera_info_) {
      sub_ = pnh_->subscribe(
        "input", 1, &GridLabel::infoCallback, this);
    }
    else {
      sub_ = pnh_->subscribe(
        "input", 1, &GridLabel::imageCallback, this);
    }
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void GridLabel::unsubscribe()
  {
    sub_.shutdown();
  }

  void GridLabel::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    label_size_ = config.label_size;
  }
  
  void GridLabel::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat label = cv::Mat::zeros(info_msg->height,
                                   info_msg->width,
                                   CV_32SC1); // int
    makeLabel(label, info_msg->header);
  }

  void GridLabel::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat label = cv::Mat::zeros(image_msg->height,
                                   image_msg->width,
                                   CV_32SC1); // int
    makeLabel(label, image_msg->header);
  }

  void GridLabel::makeLabel(cv::Mat& label, const std_msgs::Header& header) {
    int num_u = ceil(label.cols / (float)label_size_);
    int num_v = ceil(label.rows / (float)label_size_);
    int counter = 1;
    for (int v = 0; v < num_v; v++) {
      for (int u = 0; u < num_u; u++) {
        cv::Rect region(u * label_size_, v * label_size_,
                        label_size_, label_size_);
        cv::rectangle(label, region, cv::Scalar(counter), CV_FILLED);
        ++counter;
      }
    }
    pub_.publish(cv_bridge::CvImage(header,
                                    sensor_msgs::image_encodings::TYPE_32SC1,
                                    label).toImageMsg());
  }
    
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::GridLabel, nodelet::Nodelet);
