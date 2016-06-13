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

#include "jsk_perception/rect_array_to_density_image.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void RectArrayToDensityImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void RectArrayToDensityImage::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input/image", 1);
    sub_rects_.subscribe(*pnh_, "input/rect_array", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(sub_image_, sub_rects_);
      async_->registerCallback(boost::bind(&RectArrayToDensityImage::convert, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_image_, sub_rects_);
      sync_->registerCallback(boost::bind(&RectArrayToDensityImage::convert, this, _1, _2));
    }
    ros::V_string names = boost::assign::list_of("~input/image")("~input/rect_array");
    jsk_topic_tools::warnNoRemap(names);
  }

  void RectArrayToDensityImage::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_rects_.unsubscribe();
  }

  void RectArrayToDensityImage::convert(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const jsk_recognition_msgs::RectArray::ConstPtr& rects_msg)
  {
    vital_checker_->poke();
    cv::Mat density_image = cv::Mat::zeros(image_msg->height, image_msg->width, CV_32FC1);

    // Compute density
    for (size_t k=0; k<rects_msg->rects.size(); k++) {
      jsk_recognition_msgs::Rect rect = rects_msg->rects[k];
      for (int j=rect.y; j<(rect.y + rect.height); j++) {
        for (int i=rect.x; i<(rect.x + rect.width); i++) {
          density_image.at<float>(j, i)++;
        }
      }
    }

    // Scale the density image value to [0, 1]
    double min_image_value;
    double max_image_value;
    cv::minMaxLoc(density_image, &min_image_value, &max_image_value);
    cv::Mat(density_image - min_image_value).convertTo(
        density_image, CV_32FC1, 1. / (max_image_value - min_image_value));

    pub_.publish(cv_bridge::CvImage(
                    image_msg->header,
                    "32FC1",
                    density_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::RectArrayToDensityImage, nodelet::Nodelet);
