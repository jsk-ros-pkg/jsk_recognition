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

#ifndef JSK_PCL_ROS_UTILS_CLUSTER_POINT_INDICES_LABEL_FILTER_H_
#define JSK_PCL_ROS_UTILS_CLUSTER_POINT_INDICES_LABEL_FILTER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/LabelArray.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros_utils/ClusterPointIndicesLabelFilterConfig.h>

namespace jsk_pcl_ros_utils
{

  class ClusterPointIndicesLabelFilter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ClusterPointIndicesLabelFilter(): DiagnosticNodelet("ClusterPointIndicesLabelFilter")
    {
    }
    ~ClusterPointIndicesLabelFilter()
    {
      sync_.reset();
      srv_.reset();
    }
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::LabelArray> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::LabelArray> ApproximateSyncPolicy;
    typedef jsk_pcl_ros_utils::ClusterPointIndicesLabelFilterConfig Config;
  protected:
    // methods
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void filter(const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_msg,
                        const jsk_recognition_msgs::LabelArray::ConstPtr& label_msg);

    // ROS variables
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;

    ros::Publisher pub_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::LabelArray> sub_labels_;

    boost::mutex mutex_;

    // parameters
    bool approximate_sync_;
    int label_value_;
    int queue_size_;

  private:
  };
}

#endif
