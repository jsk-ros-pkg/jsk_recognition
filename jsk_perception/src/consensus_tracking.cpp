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

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "jsk_perception/consensus_tracking.h"

namespace jsk_perception
{
  void ConsensusTracking::onInit()
  {
    DiagnosticNodelet::onInit();
    int topleft_x;
    int topleft_y;
    int bottomright_x;
    int bottomright_y;
    pnh_->param("topleft_x", topleft_x, 100);
    pnh_->param("topleft_y", topleft_y, 100);
    pnh_->param("bottomright_x", bottomright_x, 200);
    pnh_->param("bottomright_y", bottomright_y, 200);
    pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pub_mask_image_ = advertise<sensor_msgs::Image>(*pnh_, "mask_output", 1);

    first_initialize_ = true;

    init_top_left_ = cv::Point2f(topleft_x, topleft_y);
    init_bottom_right_ = cv::Point2f(bottomright_x, bottomright_y);
    sub_rect_ = pnh_->subscribe("set_rect", 1, &ConsensusTracking::reset_rect, this);
    sub_rect_poly_ = pnh_->subscribe("set_poly", 1, &ConsensusTracking::reset_rect_with_poly, this);

    onInitPostProcess();
  }

  void ConsensusTracking::reset_rect(jsk_recognition_msgs::Rect rc)
  {
    init_top_left_ = cv::Point2f(rc.x - rc.width/2, rc.y - rc.height/2);
    init_bottom_right_ = cv::Point2f(rc.x + rc.width/2, rc.y + rc.height/2);

    first_initialize_ = true;
  }

  void ConsensusTracking::reset_rect_with_poly(geometry_msgs::PolygonStamped poly)
  {
    init_top_left_ = cv::Point2f(poly.polygon.points[0].x, poly.polygon.points[0].y);
    init_bottom_right_ = cv::Point2f(poly.polygon.points[1].x, poly.polygon.points[1].y);

    first_initialize_ = true;
  }

  void ConsensusTracking::subscribe()
  {
    sub_image_ = pnh_->subscribe("input", 1, &ConsensusTracking::tracking, this);
  }

  void ConsensusTracking::unsubscribe()
  {
    sub_image_.shutdown();
  }

  void ConsensusTracking::tracking(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    cv::Mat input_msg = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    cv::Mat im_gray;
    cv::cvtColor(input_msg, im_gray, CV_RGB2GRAY);
    if (first_initialize_)
    {
      cmt.initialise(im_gray, init_top_left_, init_bottom_right_);
      first_initialize_ = false;
    }
    cmt.processFrame(im_gray);

    for (int i=0; i < cmt.trackedKeypoints.size(); i++)
    {
      cv::circle(input_msg, cmt.trackedKeypoints[i].first.pt, 3, cv::Scalar(255, 0, 0));
    }
    cv::line(input_msg, cmt.topLeft, cmt.topRight, cv::Scalar(0, 30, 255));
    cv::line(input_msg, cmt.topRight, cmt.bottomRight, cv::Scalar(0, 30, 255));
    cv::line(input_msg, cmt.bottomRight, cmt.bottomLeft, cv::Scalar(0, 30, 255));
    cv::line(input_msg, cmt.bottomLeft, cmt.topLeft, cv::Scalar(0, 30, 255));

    cv::Point diff = cmt.topLeft - cmt.bottomLeft;
    cv::Point center2 = cmt.topLeft+cmt.bottomRight;
    cv::Point2f center = cv::Point2f(center2.x / 2, center2.y / 2);
    cv::Size2f size = cv::Size2f(cv::norm(cmt.topLeft-cmt.topRight), cv::norm(cmt.topLeft-cmt.bottomLeft));
    cv::RotatedRect rRect = cv::RotatedRect(center, size, asin(diff.x/cv::norm(diff))* 180 / M_PI);

    cv::Point2f vertices2f[4];
    cv::Point vertices[4];
    rRect.points(vertices2f);
    for (int i = 0; i < 4; ++i)
    {
      vertices[i] = vertices2f[i];
    }
    cv::Mat mask_image = cv::Mat::zeros(input_msg.size().height, input_msg.size().width, CV_8UC1);
    cv::fillConvexPoly(mask_image, vertices, 4, cv::Scalar(255));

    pub_image_.publish(cv_bridge::CvImage(image_msg->header,
                                          image_msg->encoding,
                                          input_msg).toImageMsg());
    pub_mask_image_.publish(cv_bridge::CvImage(image_msg->header,
                                               sensor_msgs::image_encodings::MONO8,
                                               mask_image).toImageMsg());
  }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::ConsensusTracking, nodelet::Nodelet);
