// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include "jsk_perception/concave_hull_mask_image.h"
#include <boost/assign.hpp>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif


namespace jsk_perception
{
  void ConcaveHullMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&ConcaveHullMaskImage::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ConcaveHullMaskImage::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_area_ = config.min_area;
    max_area_ = config.max_area;
  }

  void ConcaveHullMaskImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ConcaveHullMaskImage::concave, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ConcaveHullMaskImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void ConcaveHullMaskImage::concave(const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    vital_checker_->poke();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat mask = cv_ptr->image;

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    int max_area = 0;
    if (max_area_ < 0) {
      max_area = mask.cols * mask.rows;
    } else {
      max_area = max_area_;
    }

    // Prune contours
    std::vector<std::vector<cv::Point> > pruned_contours;
    for (size_t i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i]);
      if (min_area_ <= area && area <= max_area) {
        pruned_contours.push_back(contours[i]);
      }
    }

    // Smooth the contours
    std::vector<std::vector<cv::Point> > smoothed_contours;
    smoothed_contours.resize(pruned_contours.size());
    for (size_t i = 0; i < pruned_contours.size(); i++) {
      std::vector<float> x;
      std::vector<float> y;
      const size_t n = pruned_contours[i].size();

      for (size_t j = 0; j < n; j++) {
        x.push_back(pruned_contours[i][j].x);
        y.push_back(pruned_contours[i][j].y);
      }

      cv::Mat G;
      cv::transpose(cv::getGaussianKernel(11, 4.0, CV_32FC1), G);

      std::vector<float> x_smooth;
      std::vector<float> y_smooth;

      cv::filter2D(x, x_smooth, CV_32FC1, G);
      cv::filter2D(y, y_smooth, CV_32FC1, G);

      for (size_t j = 0; j < n; j++) {
        smoothed_contours[i].push_back(cv::Point2f(x_smooth[j], y_smooth[j]));
      }
    }

    cv::Mat concave_hull_mask = cv::Mat::zeros(mask.size(), mask.type());
    for (size_t i = 0; i < smoothed_contours.size(); i++) {
      cv::drawContours(concave_hull_mask, smoothed_contours, i, cv::Scalar(255), CV_FILLED);
    }

    pub_.publish(cv_bridge::CvImage(mask_msg->header,
                                    sensor_msgs::image_encodings::MONO8,
                                    concave_hull_mask).toImageMsg());

  }

}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::ConcaveHullMaskImage, nodelet::Nodelet);
