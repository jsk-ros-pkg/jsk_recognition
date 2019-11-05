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

#include "jsk_pcl_ros_utils/cluster_point_indices_label_filter.h"

namespace jsk_pcl_ros_utils
{

  void ClusterPointIndicesLabelFilter::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);

    // dynamic_reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ClusterPointIndicesLabelFilter::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ClusterPointIndicesLabelFilter::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    label_value_ = config.label_value;
    queue_size_ = config.queue_size;
  }

  void ClusterPointIndicesLabelFilter::subscribe()
  {
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_labels_.subscribe(*pnh_, "input/labels", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(sub_indices_, sub_labels_);
      async_->registerCallback(boost::bind(&ClusterPointIndicesLabelFilter::filter,
                                           this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_indices_, sub_labels_);
      sync_->registerCallback(boost::bind(&ClusterPointIndicesLabelFilter::filter,
                                          this, _1, _2));
    }
  }

  void ClusterPointIndicesLabelFilter::unsubscribe()
  {
    sub_indices_.unsubscribe();
    sub_labels_.unsubscribe();
  }

  void ClusterPointIndicesLabelFilter::filter(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_msg,
    const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg)
  {
    jsk_recognition_msgs::ClusterPointIndices filtered_msg;
    filtered_msg.header = cluster_msg->header;
    // check if sizes match?
    for (size_t i = 0; i < label_msg->labels.size(); i++) {
      if(label_msg->labels[i].id == label_value_)
        filtered_msg.cluster_indices.push_back(cluster_msg->cluster_indices[i]);
    }
    pub_.publish(filtered_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::ClusterPointIndicesLabelFilter,
                        nodelet::Nodelet);
