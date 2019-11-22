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
 * color_histogram_filter_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <limits>
#include <jsk_pcl_ros/color_histogram_filter.h>

namespace jsk_pcl_ros
{
  void ColorHistogramFilter::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("reference_histogram", reference_histogram_, std::vector<float>());
    if (reference_histogram_.empty()) {
      NODELET_WARN_STREAM("reference histogram is not yet set. waiting ~input/reference topic");
    }

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ColorHistogramFilter::configCallback, this, _1, _2);
    srv_->setCallback(f);
    pub_histogram_ = advertise<jsk_recognition_msgs::ColorHistogramArray>(*pnh_, "output", 1);
    pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output/indices", 1);
    onInitPostProcess();
  }

  void ColorHistogramFilter::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    compare_policy_ = jsk_recognition_utils::ComparePolicy(config.compare_policy);
    distance_threshold_ = config.distance_threshold;
    flip_threshold_ = config.flip_threshold;
    if (queue_size_ != config.queue_size) {
      queue_size_ = config.queue_size;
      if (isSubscribed()) {
        unsubscribe();
        subscribe();
      }
    }
  }

  void ColorHistogramFilter::subscribe()
  {
    sub_reference_ = pnh_->subscribe("input/reference", 1,
                                     &ColorHistogramFilter::reference, this);
    sub_histogram_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
    sync_->connectInput(sub_histogram_, sub_indices_);
    sync_->registerCallback(boost::bind(&ColorHistogramFilter::feature, this, _1, _2));
  }

  void ColorHistogramFilter::unsubscribe()
  {
    sub_reference_.shutdown();
    sub_histogram_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void ColorHistogramFilter::feature(const jsk_recognition_msgs::ColorHistogramArray::ConstPtr &input_histogram,
                                     const jsk_recognition_msgs::ClusterPointIndices::ConstPtr &input_indices)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (reference_histogram_.empty()) {
      NODELET_WARN_THROTTLE(1.0, "reference histogram is empty");
      return;
    }

    // check histogram size
    size_t size = reference_histogram_.size();
    for (size_t i = 0; i < input_histogram->histograms.size(); ++i) {
      if (input_histogram->histograms[i].histogram.size() != size) {
        NODELET_ERROR_STREAM("size of histogram " << i << " is different from reference");
        return;
      }
    }

    jsk_recognition_msgs::ColorHistogramArray out_hist;
    jsk_recognition_msgs::ClusterPointIndices out_indices;

    out_hist.header = input_histogram->header;
    out_indices.header = input_indices->header;

    NODELET_DEBUG("distance_threshold: %f", distance_threshold_);
    for (size_t i = 0; i < input_histogram->histograms.size(); ++i) {
      double distance = 0.0;
      bool ok = jsk_recognition_utils::compareHistogram(
        input_histogram->histograms[i].histogram, reference_histogram_,
        compare_policy_, distance);
      if (flip_threshold_) ok &= distance_threshold_ < distance;
      else                 ok &= distance_threshold_ > distance;
      NODELET_DEBUG("#%lu: %f (%s)", i, distance, ok ? "OK" : "NG");
      if (ok) {
        out_hist.histograms.push_back(input_histogram->histograms[i]);
        out_indices.cluster_indices.push_back(input_indices->cluster_indices[i]);
      }
    }

    pub_histogram_.publish(out_hist);
    pub_indices_.publish(out_indices);
  }

  void ColorHistogramFilter::reference(const jsk_recognition_msgs::ColorHistogram::ConstPtr &input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    reference_histogram_ = input->histogram;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ColorHistogramFilter, nodelet::Nodelet);
