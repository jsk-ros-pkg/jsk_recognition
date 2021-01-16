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

#include "jsk_perception/convex_hull_mask_image.h"
#include <boost/assign.hpp>
#include <boost/tuple/tuple.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_utils/cv_utils.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif

namespace jsk_perception
{
  void ConvexHullMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ConvexHullMaskImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ConvexHullMaskImage::rectify, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ConvexHullMaskImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void ConvexHullMaskImage::rectify(
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    vital_checker_->poke();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      mask_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat mask = cv_ptr->image;

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    boost::tuple<int, double> max_area;
    std::vector<std::vector<cv::Point> >hull(contours.size());
    // Find the convex hull object for each contour
    for (size_t i = 0; i < contours.size(); i++) {
      // Find max area to create mask later
      double area = cv::contourArea(contours[i]);
      if (area > max_area.get<1>()) {
        max_area = boost::make_tuple<int, double>(i, area);
      }
      cv::convexHull(cv::Mat(contours[i]), hull[i], false);
    }

    cv::Mat convex_hull_mask = cv::Mat::zeros(mask_msg->height, mask_msg->width, CV_8UC1);
    cv::drawContours(convex_hull_mask, hull, max_area.get<0>(), cv::Scalar(255), CV_FILLED);

    pub_.publish(cv_bridge::CvImage(
                    mask_msg->header,
                    sensor_msgs::image_encodings::MONO8,
                    convex_hull_mask).toImageMsg());
  }

}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::ConvexHullMaskImage, nodelet::Nodelet);
