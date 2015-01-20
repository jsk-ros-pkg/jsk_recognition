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

#include "jsk_perception/color_histogram_label_match.h"
#include <cv_bridge/cv_bridge.h>
#include <jsk_topic_tools/color_utils.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void ColorHistogramLabelMatch::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &ColorHistogramLabelMatch::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_debug_ = advertise<sensor_msgs::Image>(
      *pnh_, "debug", 1);
  }

  void ColorHistogramLabelMatch::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_label_.subscribe(*pnh_, "input/label", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_image_, sub_label_);
    sync_->registerCallback(
      boost::bind(
        &ColorHistogramLabelMatch::match, this, _1, _2));
    sub_histogram_ = pnh_->subscribe(
      "input/histogram", 1, &ColorHistogramLabelMatch::histogramCallback, this);
  }

  void ColorHistogramLabelMatch::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_label_.unsubscribe();
    sub_histogram_.shutdown();
  }

  void ColorHistogramLabelMatch::getLabels(const cv::Mat& label,
                                           std::vector<int>& keys)
  {
    std::map<int, bool> map;
    for (int j = 0; j < label.rows; j++) {
      for (int i = 0; i < label.cols; i++) {
        int label_value = label.at<int>(j, i);
        if (map.find(label_value) == map.end()) {
          map[label_value] = true;
        }
      }
    }

    for (std::map<int, bool>::iterator it = map.begin();
         it != map.end();
         ++it) {
      keys.push_back(it->first);
    }
  }

  void ColorHistogramLabelMatch::getMaskImage(const cv::Mat& label_image,
                                              const int label,
                                              cv::Mat& mask)
  {
    for (int j = 0; j < label_image.rows; j++) {
      for (int i = 0; i < label_image.cols; i++) {
        if (label_image.at<int>(j, i) == label) {
          mask.at<uchar>(j, i) = 255;
        }
      }
    }
  }
  
  void ColorHistogramLabelMatch::match(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::Image::ConstPtr& label_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (histogram_.empty()) {
      NODELET_DEBUG("no reference histogram is available");
      return;
    }
    
    cv_bridge::CvImageConstPtr image_bridge
      = cv_bridge::toCvShare(image_msg, image_msg->encoding);
    cv_bridge::CvImageConstPtr label_bridge
      = cv_bridge::toCvShare(label_msg, label_msg->encoding);
    cv::Mat image = image_bridge->image.clone();
    cv::Mat label = label_bridge->image.clone();
    
    std::vector<int> labels;
    getLabels(label, labels);
    
    cv::Mat coefficients_image = cv::Mat::zeros(image_msg->height,
                                                image_msg->width,
                                                CV_8UC3); // BGR8
    int hist_size = histogram_.cols;
    float range[] = { min_value_, max_value_ };
    const float* hist_range = { range };
    double min_coef = DBL_MAX;
    double max_coef = - DBL_MAX;
    for (size_t i = 0; i < labels.size(); i++) {
      int label_index = labels[i];
      cv::Mat mask = cv::Mat::zeros(label.rows, label.cols, CV_8UC1);
      getMaskImage(label, label_index, mask);
      cv::MatND hist;
      bool uniform = true; bool accumulate = false;
      cv::calcHist(&image, 1, 0, mask, hist, 1,
                   &hist_size, &hist_range, uniform, accumulate);
      cv::normalize(hist, hist, 1, hist.rows, cv::NORM_L2, -1,
                    cv::Mat());
      cv::Mat hist_mat = cv::Mat::zeros(1, hist_size, CV_32FC1);
      for (size_t j = 0; j < hist_size; j++) {
        hist_mat.at<float>(0, j) = hist.at<float>(0, j);
      }
      //cv::Mat hist_mat = hist;
      
      double coef = coefficients(hist_mat, histogram_);
      if (min_coef > coef) {
        min_coef = coef;
      }
      if (max_coef < coef) {
        max_coef = coef;
      }
      std_msgs::ColorRGBA coef_color = jsk_topic_tools::heatColor(coef);
      for (size_t j = 0; j < coefficients_image.rows; j++) {
        for (size_t i = 0; i < coefficients_image.cols; i++) {
          if (mask.at<uchar>(j, i) == 255) {
            coefficients_image.at<cv::Vec3b>(j, i)
              = cv::Vec3b(int(coef_color.b * 255),
                          int(coef_color.g * 255),
                          int(coef_color.r * 255));
          }
        }
      }
    }
    NODELET_INFO("coef: %f - %f", min_coef, max_coef);
    pub_debug_.publish(
      cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::BGR8,
                         coefficients_image).toImageMsg());
  }

  double ColorHistogramLabelMatch::coefficients(
    const cv::Mat& ref_hist,
    const cv::Mat& target_hist)
  {
    if (coefficient_method_ == 0) {
      return (1.0 + cv::compareHist(ref_hist, target_hist, CV_COMP_CORREL)) / 2.0;
    }
    else if (coefficient_method_ == 1) {
      double x = cv::compareHist(ref_hist, target_hist, CV_COMP_CHISQR);
      return 1/ (x * x + 1);
    }
    else if (coefficient_method_ == 2) {
      return cv::compareHist(ref_hist, target_hist, CV_COMP_INTERSECT);
    }
    else if (coefficient_method_ == 3) {
      return 1.0 - cv::compareHist(ref_hist, target_hist, CV_COMP_BHATTACHARYYA);
    }
    else if (coefficient_method_ == 4 || coefficient_method_ == 5) {
      cv::Mat ref_sig = cv::Mat::zeros(ref_hist.cols, 2, CV_32FC1);
      cv::Mat target_sig = cv::Mat::zeros(ref_hist.cols, 2, CV_32FC1);
      //NODELET_INFO("ref_hist.cols = %d", ref_hist.cols);
      for (size_t i = 0; i < ref_hist.cols; i++) {
        ref_sig.at<float>(i, 0) = ref_hist.at<float>(0, i);
        target_sig.at<float>(i, 0) = target_hist.at<float>(0, i);
        ref_sig.at<float>(i, 1) = i;
        target_sig.at<float>(i, 1) = i;
      }
      if (coefficient_method_ == 4) {
        double x = cv::EMD(ref_sig, target_sig, CV_DIST_L1);
        return 1.0  / (1.0 + x * x);
      }
      else {
        double x = cv::EMD(ref_sig, target_sig, CV_DIST_L2);
        return 1.0  / (1.0 + x * x);
      }
    }
    else {
      NODELET_ERROR("unknown coefficiet method: %d", coefficient_method_);
      return 0;
    }
  }

  void ColorHistogramLabelMatch::histogramCallback(
    const jsk_pcl_ros::ColorHistogram::ConstPtr& histogram_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //histogram_ = histogram_msg->histogram;
    histogram_ = cv::Mat(1, histogram_msg->histogram.size(), CV_32FC1);
    for (size_t i = 0; i < histogram_msg->histogram.size(); i++) {
      histogram_.at<float>(0, i) = histogram_msg->histogram[i];
    }
    cv::normalize(histogram_, histogram_, 1, histogram_.rows, cv::NORM_L2,
                  -1, cv::Mat());
  }

  void ColorHistogramLabelMatch::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    coefficient_method_ = config.coefficient_method;
    max_value_ = config.max_value;
    min_value_ = config.min_value;
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ColorHistogramLabelMatch, nodelet::Nodelet);
