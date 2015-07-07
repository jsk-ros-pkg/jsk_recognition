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

#include "jsk_perception/consensus_tracking.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <math.h> 

#define PI 3.141592

namespace jsk_perception
{
  void ConsensusTracking::onInit()
  {
    DiagnosticNodelet::onInit();
    // pnh_->param("use_panorama", use_panorama_, false);
    // pnh_->param("simple_panorama", simple_panorama_, false);
    pub_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);

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
    if(first_initialize_){
      cv::Point2f initTopLeft(150,80);
      cv::Point2f initBottomDown(300,200);

      cmt.initialise(im_gray, initTopLeft, initBottomDown);
      first_initialize_ = false;
    }
    cmt.processFrame(im_gray);

    for(int i = 0; i<cmt.trackedKeypoints.size(); i++)
      cv::circle(input_msg, cmt.trackedKeypoints[i].first.pt, 3, cv::Scalar(255,255,255));
    cv::line(input_msg, cmt.topLeft, cmt.topRight, cv::Scalar(255,255,255));
    cv::line(input_msg, cmt.topRight, cmt.bottomRight, cv::Scalar(255,255,255));
    cv::line(input_msg, cmt.bottomRight, cmt.bottomLeft, cv::Scalar(255,255,255));
    cv::line(input_msg, cmt.bottomLeft, cmt.topLeft, cv::Scalar(255,255,255));
    
    imshow("frame", input_msg);
    cv::waitKey(1);
    pub_image_.publish(cv_bridge::CvImage(image_msg->header,
                                          image_msg->encoding,
                                          input_msg).toImageMsg());
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ConsensusTracking, nodelet::Nodelet);
