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

#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "jsk_perception/consensus_tracking.h"

namespace jsk_perception
{
  void ConsensusTracking::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);

    pub_mask_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/mask", 1);
    pub_debug_image_ = advertise<sensor_msgs::Image>(*pnh_, "debug/image", 1);

    // subscribers to set initial tracking window.
    sub_image_to_init_.subscribe(*pnh_, "input", 1);
    sub_polygon_to_init_.subscribe(*pnh_, "input/polygon", 1);
    if (approximate_sync_)
    {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(sub_image_to_init_, sub_polygon_to_init_);
      async_->registerCallback(boost::bind(&ConsensusTracking::setInitialWindow, this, _1, _2));
    }
    else
    {
      sync_ = boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy> >(queue_size_);
      sync_->connectInput(sub_image_to_init_, sub_polygon_to_init_);
      sync_->registerCallback(boost::bind(&ConsensusTracking::setInitialWindow, this, _1, _2));
    }

    onInitPostProcess();
  }

  void ConsensusTracking::subscribe()
  {
    // subscribers to process the tracking
    sub_image_ = pnh_->subscribe("input", 1, &ConsensusTracking::getTrackingResult, this);
  }

  void ConsensusTracking::unsubscribe()
  {
    sub_image_.shutdown();
  }

  void ConsensusTracking::setInitialWindow(const sensor_msgs::Image::ConstPtr& image_msg,
                                           const geometry_msgs::PolygonStamped::ConstPtr& poly_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    // Convert color image to gray to track.
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);

    // Set initial rectangle vertices
    cv::Point2f initial_top_left = cv::Point2f(poly_msg->polygon.points[0].x, poly_msg->polygon.points[0].y);
    cv::Point2f initial_bottom_right = cv::Point2f(poly_msg->polygon.points[1].x, poly_msg->polygon.points[1].y);

    cmt.initialise(gray, initial_top_left, initial_bottom_right);
    window_initialized_ = true;
    ROS_INFO("A window is initialized. top_left: (%lf, %lf), bottom_right: (%lf, %lf)",
             initial_top_left.x, initial_top_left.y, initial_bottom_right.x, initial_bottom_right.y);
  }

  void ConsensusTracking::getTrackingResult(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!window_initialized_)
    {
      return;
    }

    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    // Convert color image to gray and track it.
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cmt.processFrame(gray);

    // Draw rectangle for debug view.
    for (int i=0; i < cmt.trackedKeypoints.size(); i++)
    {
      cv::circle(image, cmt.trackedKeypoints[i].first.pt, 3, cv::Scalar(255, 0, 0));
    }
    cv::line(image, cmt.topLeft, cmt.topRight, cv::Scalar(0, 30, 255));
    cv::line(image, cmt.topRight, cmt.bottomRight, cv::Scalar(0, 30, 255));
    cv::line(image, cmt.bottomRight, cmt.bottomLeft, cv::Scalar(0, 30, 255));
    cv::line(image, cmt.bottomLeft, cmt.topLeft, cv::Scalar(0, 30, 255));

    // Generate object mask.
    cv::Point diff = cmt.topLeft - cmt.bottomLeft;
    cv::Point center2 = cmt.topLeft + cmt.bottomRight;
    cv::Point2f center = cv::Point2f(center2.x / 2, center2.y / 2);
    cv::Size2f size = cv::Size2f(cv::norm(cmt.topLeft - cmt.topRight), cv::norm(cmt.topLeft - cmt.bottomLeft));
    cv::RotatedRect rRect = cv::RotatedRect(center, size, asin(diff.x / cv::norm(diff)) * 180 / M_PI);
    cv::Point2f vertices2f[4];
    rRect.points(vertices2f);
    cv::Point vertices[4];
    for (int i = 0; i < 4; ++i)
    {
      vertices[i] = vertices2f[i];
    }
    cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    cv::fillConvexPoly(mask, vertices, 4, cv::Scalar(255));

    // Publish all.
    pub_mask_image_.publish(cv_bridge::CvImage(image_msg->header,
                                               sensor_msgs::image_encodings::MONO8,
                                               mask).toImageMsg());
    pub_debug_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                sensor_msgs::image_encodings::BGR8,
                                                image).toImageMsg());
  }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::ConsensusTracking, nodelet::Nodelet);
