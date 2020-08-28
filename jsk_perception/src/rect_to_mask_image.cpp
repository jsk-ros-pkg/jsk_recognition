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

#include "jsk_perception/rect_to_mask_image.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif

namespace jsk_perception
{
  void RectToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void RectToMaskImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &RectToMaskImage::convert, this);
    sub_info_ = pnh_->subscribe("input/camera_info", 1,
                                &RectToMaskImage::infoCallback, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/camera_info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void RectToMaskImage::unsubscribe()
  {
    sub_.shutdown();
    sub_info_.shutdown();
  }

  void RectToMaskImage::convert(
    const geometry_msgs::PolygonStamped::ConstPtr& rect_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (camera_info_) {
      cv::Mat mask_image = cv::Mat::zeros(camera_info_->height,
                                          camera_info_->width,
                                          CV_8UC1);
      geometry_msgs::Point32 P0 = rect_msg->polygon.points[0];
      geometry_msgs::Point32 P1 = rect_msg->polygon.points[2];
      double min_x = std::max(std::min(P0.x, P1.x), 0.0f);
      double max_x = std::max(P0.x, P1.x);
      double min_y = std::max(std::min(P0.y, P1.y), 0.0f);
      double max_y = std::max(P0.y, P1.y);
      double width = std::min(max_x - min_x, camera_info_->width - min_x);
      double height = std::min(max_y - min_y, camera_info_->height - min_y);
      cv::Rect region(min_x, min_y, width, height);
      cv::rectangle(mask_image, region, cv::Scalar(255), CV_FILLED);
      pub_.publish(cv_bridge::CvImage(
                     rect_msg->header,
                     sensor_msgs::image_encodings::MONO8,
                     mask_image).toImageMsg());
    }
  }

  
  void RectToMaskImage::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_ = info_msg;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::RectToMaskImage, nodelet::Nodelet);
