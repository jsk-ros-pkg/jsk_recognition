// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
#include "jsk_pcl_ros_utils/cluster_point_indices_to_point_indices.h"
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <boost/assign.hpp>

namespace jsk_pcl_ros_utils
{

  void ClusterPointIndicesToPointIndices::onInit()
  {
    DiagnosticNodelet::onInit();
    // dynamic_reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ClusterPointIndicesToPointIndices::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<PCLIndicesMsg>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ClusterPointIndicesToPointIndices::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    index_ = config.index;
  }

  void ClusterPointIndicesToPointIndices::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1,
                           &ClusterPointIndicesToPointIndices::convert,
                           this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ClusterPointIndicesToPointIndices::unsubscribe()
  {
    sub_.shutdown();
  }

  void ClusterPointIndicesToPointIndices::convert(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_indices_msg)
  {
    vital_checker_->poke();
    PCLIndicesMsg indices_msg;
    indices_msg.header = cluster_indices_msg->header;

    int cluster_size = cluster_indices_msg->cluster_indices.size();
    if (index_ < 0) {
      for(int i = 0;i < cluster_size; ++i) {
        indices_msg.indices.insert(indices_msg.indices.end(),
                                   cluster_indices_msg->cluster_indices[i].indices.begin(),
                                   cluster_indices_msg->cluster_indices[i].indices.end());
      }
    } else if (index_ < cluster_size) {
      indices_msg.indices = cluster_indices_msg->cluster_indices[index_].indices;
    } else {
      NODELET_ERROR_THROTTLE(10, "Invalid ~index %d is specified for cluster size %d.", index_, cluster_size);
    }
    pub_.publish(indices_msg);
  }

}  // namespace jsk_pcl_ros_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::ClusterPointIndicesToPointIndices, nodelet::Nodelet);
