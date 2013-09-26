// -*- mode: C++ -*-
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#ifndef JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_
#define JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

namespace jsk_pcl_ros
{
  class ClusterPointIndicesDecomposer: public pcl_ros::PCLNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime< sensor_msgs::PointCloud2,
                                                       jsk_pcl_ros::ClusterPointIndices > SyncPolicy;
    ClusterPointIndicesDecomposer();
    virtual ~ClusterPointIndicesDecomposer();
    virtual void onInit();
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &point,
                         const jsk_pcl_ros::ClusterPointIndicesConstPtr &indices);
    virtual void sortIndicesOrder(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                  std::vector<pcl::IndicesPtr> indices_array,
                                  std::vector<pcl::IndicesPtr> &output_array);
  protected:
    boost::shared_ptr<ros::NodeHandle> pnh_;
    //ros::Subscriber sub_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_pcl_ros::ClusterPointIndices> sub_target_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    std::vector<ros::Publisher> publishers_;

    virtual void allocatePublishers(size_t num);
    
  };

}

#endif
