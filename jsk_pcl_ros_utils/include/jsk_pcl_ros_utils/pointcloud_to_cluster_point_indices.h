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

#ifndef JSK_PCL_ROS_UTILS_POINTCLOUD_TO_CLUSTER_POINT_INDICES_H_
#define JSK_PCL_ROS_UTILS_POINTCLOUD_TO_CLUSTER_POINT_INDICES_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <sensor_msgs/PointCloud2.h>

namespace jsk_pcl_ros_utils
{
  class PointCloudToClusterPointIndices: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    PointCloudToClusterPointIndices(): DiagnosticNodelet("PointCloudToClusterPointIndices") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void convert(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    ros::Subscriber sub_;
    ros::Publisher pub_;

    bool skip_nan_;

  private:
  };

}

#endif
