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

#ifndef JSK_PCL_ROS_VOXEL_GRID_DOWNSAMPLE_DECODER_H_
#define JSK_PCL_ROS_VOXEL_GRID_DOWNSAMPLE_DECODER_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "jsk_recognition_msgs/SlicedPointCloud.h"
// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>

#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros
{
  class VoxelGridDownsampleDecoder :
    public jsk_topic_tools::ConnectionBasedNodelet
  {
  protected:
    tf::TransformListener tf_listener;
    // utility
    int getPointcloudID(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input);
    int getPointcloudSequenceID(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input);
    std::string getPointcloudFrameId(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input);
    virtual void subscribe();
    virtual void unsubscribe();
  private:
    int latest_sequence_id_;
    int previous_id_;
    void publishBuffer();
    std::vector<jsk_recognition_msgs::SlicedPointCloudConstPtr> pc_buffer_;
    void pointCB(const jsk_recognition_msgs::SlicedPointCloudConstPtr &input);
    ros::Subscriber sub_;
    ros::Publisher pub_;
    virtual void onInit();
  };
}

#endif  // JSK_PCL_ROS_VOXEL_GRID_DOWNSAMPLE_MANAGER_H
