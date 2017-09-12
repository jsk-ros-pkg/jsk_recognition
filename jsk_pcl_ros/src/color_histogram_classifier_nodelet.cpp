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
/*
 * color_histogram_classifier_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <limits>
#include <jsk_pcl_ros/color_histogram_classifier.h>
#include <jsk_recognition_msgs/ClassificationResult.h>

namespace jsk_pcl_ros
{
  void ColorHistogramClassifier::onInit()
  {
    DiagnosticNodelet::onInit();
    classifier_name_ = "color_histogram";

    if (!loadReference()) return;

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ColorHistogramClassifier::configCallback, this, _1, _2);
    srv_->setCallback(f);
    pub_class_ = advertise<jsk_recognition_msgs::ClassificationResult>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  bool ColorHistogramClassifier::loadReference()
  {

    // load label names
    std::vector<std::string> labels;
    pnh_->param("label_names", labels, std::vector<std::string>());
    if (labels.empty()) {
      NODELET_FATAL_STREAM("param ~label_names must not be empty");
      return false;
    }

    // load histograms
    for (size_t i = 0; i < labels.size(); ++i) {
      std::string name = "histograms/" + labels[i];
      NODELET_INFO_STREAM("Loading " << name);
      std::vector<float> hist;
      pnh_->param(name, hist, std::vector<float>());
      if (hist.empty()) {
        NODELET_ERROR_STREAM("Failed to load " << name);
      } else {
        label_names_.push_back(labels[i]);
        reference_histograms_.push_back(hist);
      }
    }

    // check bin size
    bin_size_ = reference_histograms_[0].size();
    for (size_t i = 0; i < label_names_.size(); ++i) {
      if (reference_histograms_[i].size() != bin_size_) {
        NODELET_FATAL_STREAM("size of histogram " << label_names_[i] << " is different from " << label_names_[0]);
        return false;
      }
    }

    NODELET_INFO_STREAM("Loaded " << label_names_.size() << " references");
    return true;
  }

  void ColorHistogramClassifier::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    compare_policy_ = jsk_recognition_utils::ComparePolicy(config.compare_policy);
    detection_threshold_ = config.detection_threshold;

    if (queue_size_ != config.queue_size) {
      queue_size_ = config.queue_size;
      if (isSubscribed()) {
        unsubscribe();
        subscribe();
      }
    }
  }

  void ColorHistogramClassifier::subscribe()
  {
    sub_hist_ = pnh_->subscribe("input", 1, &ColorHistogramClassifier::feature, this);
    sub_hists_ = pnh_->subscribe("input/array", 1, &ColorHistogramClassifier::features, this);
  }

  void ColorHistogramClassifier::unsubscribe()
  {
    sub_hist_.shutdown();
    sub_hists_.shutdown();
  }

  void ColorHistogramClassifier::computeDistance(const std::vector<float>& histogram,
                                                 std::vector<double>& distances) {
    distances.resize(reference_histograms_.size());
    for (size_t i = 0; i < reference_histograms_.size(); ++i) {
      jsk_recognition_utils::compareHistogram(
        histogram, reference_histograms_[i],
        compare_policy_, distances[i]);
    }
  }

  void ColorHistogramClassifier::feature(const jsk_recognition_msgs::ColorHistogram::ConstPtr& histogram)
  {
    boost::mutex::scoped_lock lock(mutex_);

    jsk_recognition_msgs::ClassificationResult result;
    result.header = histogram->header;
    result.classifier = classifier_name_;
    result.target_names = label_names_;

    std::vector<double> distances;
    computeDistance(histogram->histogram, distances);

    double max_prob = 0.0;
    int label;
    for (size_t i = 0; i < distances.size(); ++i) {
      double prob = distances[i];
      result.probabilities.push_back(prob);
      if (prob > max_prob) {
        max_prob = prob;
        label = i;
      }
    }

    if (max_prob >= detection_threshold_) {
      result.labels.push_back(label);
      result.label_names.push_back(label_names_[label]);
      result.label_proba.push_back(max_prob);
    } else {
      result.labels.push_back(-1);
      result.label_names.push_back(std::string());
      result.label_proba.push_back(0.0);
    }

    pub_class_.publish(result);
  }

  void ColorHistogramClassifier::features(const jsk_recognition_msgs::ColorHistogramArray::ConstPtr& histograms)
  {
    boost::mutex::scoped_lock lock(mutex_);

    jsk_recognition_msgs::ClassificationResult result;
    result.header = histograms->header;
    result.classifier = classifier_name_;
    result.target_names = label_names_;

    for (size_t i = 0; i < histograms->histograms.size(); ++i) {
      std::vector<double> distances;
      computeDistance(histograms->histograms[i].histogram, distances);

      double max_prob = 0.0;
      int label;
      for (size_t i = 0; i < distances.size(); ++i) {
        double prob = distances[i];
        result.probabilities.push_back(prob);
        if (prob > max_prob) {
          max_prob = prob;
          label = i;
        }
      }

      if (max_prob >= detection_threshold_) {
        result.labels.push_back(label);
        result.label_names.push_back(label_names_[label]);
        result.label_proba.push_back(max_prob);
      } else {
        result.labels.push_back(-1);
        result.label_names.push_back(std::string());
        result.label_proba.push_back(0.0);
      }
    }

    pub_class_.publish(result);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ColorHistogramClassifier, nodelet::Nodelet);
