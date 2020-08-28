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

#include "jsk_perception/contour_finder.h"
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
  void ContourFinder::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_debug_image_ = advertise<sensor_msgs::Image>(*pnh_, "debug", 1);
    pub_convex_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/convex", 1);
    onInitPostProcess();
  }

  void ContourFinder::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ContourFinder::segment, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ContourFinder::unsubscribe()
  {
    sub_.shutdown();
  }

  void ContourFinder::segment(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<std::vector<cv::Point> > convex_contours(1);
    std::vector<cv::Vec4i> hierarchy;
    cv::RNG rng(12345);
    cv::Mat input = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::MONO8)->image;
    cv::findContours(input, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    cv::Mat drawing = cv::Mat::zeros(input.size(), CV_8UC3);
    for( int i = 0; i< contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      cv::drawContours(drawing, contours, i, color, CV_FILLED, 8, hierarchy, 0, cv::Point());
    }
    pub_debug_image_.publish(cv_bridge::CvImage(
                               image_msg->header,
                               sensor_msgs::image_encodings::BGR8,
                               drawing).toImageMsg());
    // combine all the contours into one contours
    std::vector<cv::Point> all_contours;
    for (size_t i = 0; i < contours.size(); i++) {
      for (size_t j = 0; j < contours[i].size(); j++) {
        all_contours.push_back(contours[i][j]);
      }
    }
    // build convex
    cv::convexHull(cv::Mat(all_contours), convex_contours[0]);
    cv::Mat convex_image = cv::Mat::zeros(input.size(), CV_8UC1);
    cv::drawContours(convex_image, convex_contours, 0, cv::Scalar(255), CV_FILLED);
    pub_convex_image_.publish(
      cv_bridge::CvImage(
        image_msg->header,
        sensor_msgs::image_encodings::MONO8,
        convex_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ContourFinder, nodelet::Nodelet);

