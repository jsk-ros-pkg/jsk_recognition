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

#include "jsk_perception/snake_segmentation.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION < 3
#include <opencv2/legacy/legacy.hpp>
#endif
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void SnakeSegmentation::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &SnakeSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_debug_ = advertise<sensor_msgs::Image>(*pnh_, "debug", 1);
    onInitPostProcess();
  }

  void SnakeSegmentation::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    alpha_ = config.alpha;
    beta_ = config.beta;
    gamma_ = config.gamma;
    max_iterations_ = config.max_iterations;
    epsilon_ = config.epsilon;
    if (config.window_size % 2 == 1) {
      window_size_ = config.window_size;
    }
    else {
      config.window_size = window_size_;
    }
  }


  
  void SnakeSegmentation::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &SnakeSegmentation::segment, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void SnakeSegmentation::unsubscribe()
  {
    sub_.shutdown();
  }

  void SnakeSegmentation::segment(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
#if CV_MAJOR_VERSION >= 3
    ROS_ERROR("cvSnakeImage is not supported in OpenCV3");
#else
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat input = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::MONO8)->image;
    // snake is only supported in legacy format
    IplImage ipl_input = input;
    float alpha = alpha_;
    float beta = beta_;
    float gamma = gamma_;
    CvTermCriteria criteria;
    criteria.type = CV_TERMCRIT_ITER;
    criteria.max_iter = max_iterations_;
    criteria.epsilon = epsilon_;
    int coeffUsage = CV_VALUE;
    CvSize win = cv::Size(window_size_, window_size_);
    int calcGradient = 1;
    std::vector<CvPoint> points;
    // all the points on  edge of  border of image
    const int resolution = 20;
    for (size_t i = 0; i < resolution; i++) {
      double r = (double)i/resolution;
      points.push_back(cvPoint(0, r * image_msg->height));
    }
    for (size_t i = 0; i < resolution; i++) {
      double r = (double)i/resolution;
      points.push_back(cvPoint(image_msg->width * r, image_msg->height));
    }
    for (size_t i = 0; i < resolution; i++) {
      double r = (double)i/resolution;
      points.push_back(cvPoint(image_msg->width, image_msg->height * (1 - r)));
    }
    for (size_t i = 0; i < resolution; i++) {
      double r = (double)i/resolution;
      points.push_back(cvPoint(image_msg->width * (1 - r), 0));
    }
    cvSnakeImage(&ipl_input, &(points[0]), points.size(), &alpha, &beta, &gamma,
                 coeffUsage, win, criteria, calcGradient);
    cv::Mat debug_image;
    cv::cvtColor(input, debug_image, CV_GRAY2BGR);
    for (int i = 1; i < points.size(); i++) {
      cv::line(debug_image, points[i - 1], points[i], cv::Scalar(0, 100, 0), 2);
      cv::circle(debug_image, points[i], 2, cv::Scalar(100, 0, 0), 1);
    }

    pub_debug_.publish(cv_bridge::CvImage(
                         image_msg->header,
                         sensor_msgs::image_encodings::BGR8,
                         debug_image).toImageMsg());
#endif
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::SnakeSegmentation, nodelet::Nodelet);
