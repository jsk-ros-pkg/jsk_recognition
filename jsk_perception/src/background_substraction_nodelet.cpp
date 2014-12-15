// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "jsk_perception/background_substraction.h"

namespace jsk_perception
{
  void BackgroundSubstraction::onInit()
  {
    DiagnosticNodelet::onInit();
    image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    bg_.set("nmixtures", 3);
    bg_.set("detectShadows", 1);
  }

  void BackgroundSubstraction::subscribe()
  {
    it_.reset(new image_transport::ImageTransport(*pnh_));
    sub_ = it_->subscribe("image", 1, &BackgroundSubstraction::substract, this);
  }

  void BackgroundSubstraction::unsubscribe()
  {
    sub_.shutdown();
  }

  void BackgroundSubstraction::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "BackgroundSubstraction running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "BackgroundSubstraction", vital_checker_, stat);
    }

  }
  
  void BackgroundSubstraction::substract(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr
      = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;
    // bg.set("nmixtures", 3);
    // bg.set("detectShadows", 1);
    
    //cv::BackgroundSubtractorMOG2 bg(30, 16.0, false);
    
    std::vector <std::vector<cv::Point > > contours;
    //cv::Mat fg, back;
    if (prev_image_.empty()) {
      prev_image_.create(image.size(), image.type());
    }
    bg_(image, prev_image_);
    //bg.getBackgroundImage (back);
    // cv::Mat output;
    // cv::erode (fg, fg, cv::Mat ());
    // cv::dilate (fg, fg, cv::Mat ());
    // cv::findContours (fg, contours, CV_RETR_EXTERNAL,
    //                   CV_CHAIN_APPROX_NONE);
    // cv::drawContours (image, contours, -1, cv::Scalar (0, 0, 255), 2);
    
    // //cv::bitwise_and(image, image, output, prev_image_);
    sensor_msgs::Image::Ptr diff_image
      = cv_bridge::CvImage(image_msg->header,
                           sensor_msgs::image_encodings::MONO8,
                           prev_image_).toImageMsg();
    image_pub_.publish(diff_image);
    //cv::imshow("input", image);
    //cv::imshow("output", prev_image_);
    //cv::imshow("back", back);
    //cv::waitKey(1);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::BackgroundSubstraction, nodelet::Nodelet);
