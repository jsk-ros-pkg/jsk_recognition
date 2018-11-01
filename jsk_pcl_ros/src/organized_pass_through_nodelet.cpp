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

#include "jsk_pcl_ros/organized_pass_through.h"
#include <jsk_topic_tools/diagnostic_utils.h>
#include <pcl/filters/extract_indices.h>
#include <jsk_recognition_utils/pcl_util.h>

namespace jsk_pcl_ros
{
  OrganizedPassThrough::OrganizedPassThrough():
    DiagnosticNodelet("OrganizedPassThrough")
  {
    
  }
  
  void OrganizedPassThrough::onInit()
  {
    DiagnosticNodelet::onInit();
    ////////////////////////////////////////////////////////
    // Publisher
    ////////////////////////////////////////////////////////
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &OrganizedPassThrough::configCallback, this, _1, _2);
    srv_->setCallback (f);
    onInitPostProcess();
  }

  void OrganizedPassThrough::subscribe()
  {
    ////////////////////////////////////////////////////////
    // Subscriber
    ////////////////////////////////////////////////////////
    sub_ = pnh_->subscribe("input", 1, &OrganizedPassThrough::filter, this);
  }

  void OrganizedPassThrough::unsubscribe()
  {
    sub_.shutdown();
  }

  void OrganizedPassThrough::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (config.filter_field == 0) {
      filter_field_ = FIELD_X;
    }
    else {
      filter_field_ = FIELD_Y;
    }
    min_index_ = config.min_index;
    max_index_ = config.max_index;
    filter_limit_negative_ = config.filter_limit_negative;
    keep_organized_ = config.keep_organized;
    remove_nan_ = config.remove_nan;
  }

  pcl::PointIndices::Ptr OrganizedPassThrough::filterIndices(const pcl::PointCloud<PointT>::Ptr& pc)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);

    if (filter_field_ == FIELD_X) {
      for (size_t j = 0; j < pc->height; j++) {
        for (size_t i = min_index_; i < max_index_ && i < pc->width; i++) {
          size_t idx = i + j * pc->width;
          if (!remove_nan_ || (remove_nan_ && !isPointNaN(pc->points[idx])))
            indices->indices.push_back(idx);
        }
      }
    }
    else if (filter_field_ == FIELD_Y) {
      for (size_t i = 0; i < pc->width; i++) {
        for (size_t j = min_index_; j < max_index_ && j < pc->height; j++) {
          size_t idx = i + j * pc->width;
          if (!remove_nan_ || (remove_nan_ && !isPointNaN(pc->points[idx])))
            indices->indices.push_back(idx);
        }
      }
    }
    return indices;
  }
  
  void OrganizedPassThrough::filter(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointIndices::Ptr indices = filterIndices(cloud);
    filtered_points_counter_.add(indices->indices.size());
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.setNegative(filter_limit_negative_);
    ex.setKeepOrganized(keep_organized_);
    pcl::PointCloud<PointT> output;
    ex.filter(output);
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(output, ros_output);
    ros_output.header = msg->header;
    pub_.publish(ros_output);
    diagnostic_updater_->update();
  }

  void OrganizedPassThrough::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   name_ + " running");
      stat.add("Filtered points (Avg.)", filtered_points_counter_.mean());
      if (filter_field_ == FIELD_X) {
        stat.add("filter field", "x");
      }
      else if (filter_field_ == FIELD_Y) {
        stat.add("filter field", "y");
      }
      stat.add("min index", min_index_);
      stat.add("max index", max_index_);
      jsk_topic_tools::addDiagnosticBooleanStat("keep organized",
                                                keep_organized_,
                                                stat);
      jsk_topic_tools::addDiagnosticBooleanStat("remove_nan",
                                                remove_nan_,
                                                stat);
      jsk_topic_tools::addDiagnosticBooleanStat("filter_limit_negative",
                                                filter_limit_negative_,
                                                stat);
    }
    DiagnosticNodelet::updateDiagnostic(stat);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OrganizedPassThrough, nodelet::Nodelet);
