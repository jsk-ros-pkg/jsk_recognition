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


#include "jsk_perception/centroid_publisher.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  CentroidPublisher::CentroidPublisher(): DiagnosticNodelet("CentroidPublisher")
  {
  }

  CentroidPublisher::~CentroidPublisher()
  {
  }

  void CentroidPublisher::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<geometry_msgs::PointStamped>(*pnh_, "centroid", 1);
  }

  void CentroidPublisher::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &CentroidPublisher::input_cb, this);
  }

  void CentroidPublisher::unsubscribe()
  {
    sub_.shutdown();
  }

  void CentroidPublisher::input_cb(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    double cx, cy;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    cv::Mat image = cv_ptr->image;
    cv::Moments moments = cv::moments(image, 1);
    cx = moments.m10/moments.m00;
    cy = moments.m01/moments.m00;

    geometry_msgs::PointStamped point;
    point.header = image_msg->header;
    point.point.x = cx;
    point.point.y = cy;
    point.point.z = 0;
    pub_.publish(point);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::CentroidPublisher, nodelet::Nodelet);
