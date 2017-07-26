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

#include <algorithm>
#include <jsk_pcl_ros_utils/polygon_array_likelihood_filter.h>

namespace jsk_pcl_ros_utils
{
  void PolygonArrayLikelihoodFilter::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&PolygonArrayLikelihoodFilter::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_polygons_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output_polygons", 1);

    pnh_->param<bool>("use_coefficients", use_coefficients_, true);
    if (use_coefficients_) {
      pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
        *pnh_, "output_coefficients", 1);
    }

    onInitPostProcess();
  }

  void PolygonArrayLikelihoodFilter::subscribe()
  {
    if (use_coefficients_) {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
      sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
      sync_->connectInput(sub_polygons_, sub_coefficients_);
      sync_->registerCallback(boost::bind(
                                &PolygonArrayLikelihoodFilter::filter,
                                this, _1, _2));
    } else {
      sub_polygons_alone_ = pnh_->subscribe(
        "input_polygons", 1, &PolygonArrayLikelihoodFilter::filter, this);
    }
  }

  void PolygonArrayLikelihoodFilter::unsubscribe()
  {
    if (use_coefficients_) {
      sub_polygons_.unsubscribe();
      sub_coefficients_.unsubscribe();
    } else {
      sub_polygons_alone_.shutdown();
    }
  }

  void PolygonArrayLikelihoodFilter::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    threshold_ = config.threshold;
    negative_ = config.negative;
    if (queue_size_ != config.queue_size) {
      queue_size_ = config.queue_size;
      unsubscribe();
      subscribe();
    }
  }

  void PolygonArrayLikelihoodFilter::filter(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons)
  {
    jsk_recognition_msgs::ModelCoefficientsArray::Ptr dummy;
    dummy.reset();
    filter(polygons, dummy);
  }

  void PolygonArrayLikelihoodFilter::filter(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (polygons->polygons.size() != polygons->likelihood.size()) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "The size of polygons " << polygons->polygons.size()
                                << " must be same as the size of likelihood "
                                << polygons->likelihood.size());
      return;
    }
    if (use_coefficients_ &&
        polygons->polygons.size() != coefficients->coefficients.size()) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "The size of polygons " << polygons->polygons.size()
                                << "must be same as the size of coeeficients "
                                << coefficients->coefficients.size());
      return;
    }
    vital_checker_->poke();

    bool use_labels = polygons->polygons.size() == polygons->labels.size();

    std::vector<std::pair<double, int> > lookup_table;
    lookup_table.resize(polygons->polygons.size());
    for (int i = 0; i < polygons->polygons.size(); ++i) {
      lookup_table[i] = std::pair<double, int>(polygons->likelihood[i], i);
    }
    std::sort(lookup_table.rbegin(), lookup_table.rend());

    jsk_recognition_msgs::PolygonArray ret_polygons;
    jsk_recognition_msgs::ModelCoefficientsArray ret_coefficients;

    for (int i = 0; i < lookup_table.size(); ++i) {
      double likelihood = lookup_table[i].first;
      int idx = lookup_table[i].second;
      if ((!negative_ && likelihood >= threshold_) ||
          (negative_  && likelihood <  threshold_)) {
        ret_polygons.polygons.push_back(polygons->polygons[idx]);
        ret_polygons.likelihood.push_back(polygons->likelihood[idx]);
        if (use_labels) {
          ret_polygons.labels.push_back(polygons->labels[idx]);
        }
        if (use_coefficients_) {
          ret_coefficients.coefficients.push_back(coefficients->coefficients[idx]);
        }
      }
    }

    ret_polygons.header = polygons->header;
    pub_polygons_.publish(ret_polygons);
    if (use_coefficients_) {
      ret_coefficients.header = coefficients->header;
      pub_coefficients_.publish(ret_coefficients);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonArrayLikelihoodFilter, nodelet::Nodelet);
