// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#ifndef JSK_PCL_ROS_SELECTED_CLUSTER_PUBLISHER_H_
#define JSK_PCL_ROS_SELECTED_CLUSTER_PUBLISHER_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/Int32Stamped.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros
{
  class SelectedClusterPublisher: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, jsk_recognition_msgs::ClusterPointIndices, jsk_recognition_msgs::Int32Stamped> SyncPolicy;
  protected:
    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::Int32Stamped> sub_index_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

    virtual void extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                         const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices,
                         const jsk_recognition_msgs::Int32Stamped::ConstPtr& index);
    virtual void subscribe();
    virtual void unsubscribe();
  private:
    virtual void onInit();
    
    bool keep_organized_;
  };
  
}

#endif
