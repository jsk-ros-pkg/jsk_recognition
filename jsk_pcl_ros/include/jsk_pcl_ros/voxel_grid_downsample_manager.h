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

#ifndef JSK_PCL_ROS_VOXEL_GRID_DOWNSAMPLE_MANAGER_H_
#define JSK_PCL_ROS_VOXEL_GRID_DOWNSAMPLE_MANAGER_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"

// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros
{
  class VoxelGridDownsampleManager : public jsk_topic_tools::ConnectionBasedNodelet
  {
  protected:
    tf::TransformListener* tf_listener;
    std::vector<visualization_msgs::Marker::ConstPtr> grid_;
    void addGrid(const visualization_msgs::Marker::ConstPtr &new_box);
    virtual void subscribe();
    virtual void unsubscribe();
  private:
    void pointCB(const sensor_msgs::PointCloud2ConstPtr &input);
    void clearAll();
    void initializeGrid(void);
    ros::Subscriber sub_;
    ros::Subscriber bounding_box_sub_;
    ros::Publisher pub_;
    ros::Publisher pub_encoded_;
    size_t max_points_;
    double rate_;
    int sequence_id_;           // incremented by every add_box
    std::string base_frame_;
    virtual void onInit();
  };
}

#endif  // JSK_PCL_ROS_VOXEL_GRID_DOWNSAMPLE_MANAGER_H
