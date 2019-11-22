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

#include "jsk_pcl_ros/bounding_box_filter.h"

namespace jsk_pcl_ros
{
  void BoundingBoxFilter::onInit()
  {
    ConnectionBasedNodelet::onInit();

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &BoundingBoxFilter::configCallback, this, _1, _2);
    srv_->setCallback (f);

    ////////////////////////////////////////////////////////
    // Publishers
    ////////////////////////////////////////////////////////
    pnh_->param("with_indices", with_indices_, true);
    filtered_box_pub_
      = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "output_box", 1);
    if (with_indices_) {
      filtered_indices_pub_
        = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output_indices", 1);
    }

    onInitPostProcess();
  }

  void BoundingBoxFilter::subscribe()
  {
    sub_box_.subscribe(*pnh_, "input_box", 1);
    if (with_indices_) {
      sub_indices_.subscribe(*pnh_, "input_indices", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_box_, sub_indices_);
      sync_->registerCallback(boost::bind(&BoundingBoxFilter::filterWithIndices, this, _1, _2));
    } else {
      sub_box_.registerCallback(boost::bind(&BoundingBoxFilter::filter, this, _1));
    }
  }

  void BoundingBoxFilter::unsubscribe()
  {
    sub_box_.unsubscribe();
    if (with_indices_) {
      sub_indices_.unsubscribe();
    }
  }

  void BoundingBoxFilter::filterBoundingBoxes(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg,
    std::vector<size_t>& keep)
  {
    for (size_t i = 0; i < box_array_msg->boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = box_array_msg->boxes[i];
      if (!filter_limit_negative_) {
        if (use_x_dimension_) {
          if (box.dimensions.x < x_dimension_min_ ||
              box.dimensions.x > x_dimension_max_) {
            continue;
          }
        }
        if (use_y_dimension_) {
          if (box.dimensions.y < y_dimension_min_ ||
              box.dimensions.y > y_dimension_max_) {
            continue;
          }
        }
        if (use_z_dimension_) {
          if (box.dimensions.z < z_dimension_min_ ||
              box.dimensions.z > z_dimension_max_) {
            continue;
          }
        }
      }
      else {
        if (use_x_dimension_) {
          if (box.dimensions.x > x_dimension_min_ &&
              box.dimensions.x < x_dimension_max_) {
            continue;
          }
        }
        if (use_y_dimension_) {
          if (box.dimensions.y > y_dimension_min_ &&
              box.dimensions.y < y_dimension_max_) {
            continue;
          }
        }
        if (use_z_dimension_) {
          if (box.dimensions.z > z_dimension_min_ &&
              box.dimensions.z < z_dimension_max_) {
            continue;
          }
        }
      }
      keep.push_back(i);
    }
  }

  void BoundingBoxFilter::filter(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    jsk_recognition_msgs::BoundingBoxArray filtered_box_array;
    filtered_box_array.header = box_array_msg->header;

    std::vector<size_t> keep;  // index of bounding box which will be kept
    filterBoundingBoxes(box_array_msg, keep);
    for (size_t j = 0; j < keep.size(); j++)
    {
      size_t i = keep[j];
      filtered_box_array.boxes.push_back(box_array_msg->boxes[i]);
    }

    // publish
    filtered_box_pub_.publish(filtered_box_array);

    // for diagnostic
    size_t pass_count = keep.size();
    size_t remove_count = box_array_msg->boxes.size() - pass_count;
    remove_counter_.add(remove_count);
    pass_counter_.add(pass_count);
  }

  void BoundingBoxFilter::filterWithIndices(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (box_array_msg->boxes.size() != indices_msg->cluster_indices.size()) {
      NODELET_ERROR(
        "the size of message ~input_box and ~input_indices are different");
      return;
    }
    vital_checker_->poke();

    jsk_recognition_msgs::BoundingBoxArray filtered_box_array;
    jsk_recognition_msgs::ClusterPointIndices filtered_indices;
    filtered_box_array.header = box_array_msg->header;
    filtered_indices.header = indices_msg->header;

    std::vector<size_t> keep;  // index of bounding box which will be kept
    filterBoundingBoxes(box_array_msg, keep);
    for (size_t j = 0; j < keep.size(); j++)
    {
      size_t i = keep[j];
      filtered_box_array.boxes.push_back(box_array_msg->boxes[i]);
      filtered_indices.cluster_indices.push_back(indices_msg->cluster_indices[i]);
    }

    // publish
    filtered_box_pub_.publish(filtered_box_array);
    filtered_indices_pub_.publish(filtered_indices);

    // for diagnostic
    size_t pass_count = keep.size();
    size_t remove_count = box_array_msg->boxes.size() - pass_count;
    remove_counter_.add(remove_count);
    pass_counter_.add(pass_count);
  }

  void BoundingBoxFilter::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    filter_limit_negative_ = config.filter_limit_negative;
    use_x_dimension_ = config.use_x_dimension;
    use_y_dimension_ = config.use_y_dimension;
    use_z_dimension_ = config.use_z_dimension;
    x_dimension_min_ = config.x_dimension_min;
    x_dimension_max_ = config.x_dimension_max;
    y_dimension_min_ = config.y_dimension_min;
    y_dimension_max_ = config.y_dimension_max;
    z_dimension_min_ = config.z_dimension_min;
    z_dimension_max_ = config.z_dimension_max;
  }
  
  void BoundingBoxFilter::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "BoundingBoxArray running");
      stat.add("Number of filtered box (Avg.)", remove_counter_.mean());
      stat.add("Number of passed box (Avg.)", pass_counter_.mean());
      jsk_topic_tools::addDiagnosticBooleanStat("Use x dimension", use_x_dimension_, stat);
      stat.add("minimum x dimension", x_dimension_min_);
      stat.add("maximum x dimension", x_dimension_max_);
      jsk_topic_tools::addDiagnosticBooleanStat("Use y dimension", use_y_dimension_, stat);
      stat.add("minimum y dimension", y_dimension_min_);
      stat.add("maximum y dimension", y_dimension_max_);
      jsk_topic_tools::addDiagnosticBooleanStat("Use z dimension", use_z_dimension_, stat);
      stat.add("minimum z dimension", z_dimension_min_);
      stat.add("maximum z dimension", z_dimension_max_);
      jsk_topic_tools::addDiagnosticBooleanStat("Filter limit negative",
                               filter_limit_negative_, stat);
    }
    DiagnosticNodelet::updateDiagnostic(stat);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::BoundingBoxFilter, nodelet::Nodelet);
