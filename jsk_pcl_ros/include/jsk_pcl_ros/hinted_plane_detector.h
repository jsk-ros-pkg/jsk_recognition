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

#ifndef JSK_PCL_ROS_HINTED_PLANE_DETECTOR_H_
#define JSK_PCL_ROS_HINTED_PLANE_DETECTOR_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>

#include <tf/transform_listener.h>

#include "jsk_pcl_ros/connection_based_nodelet.h"

namespace jsk_pcl_ros {
  class HintedPlaneDetector: public ConnectionBasedNodelet
  {
  public:
    virtual void onInit();
    
  protected:
    void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void hintCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void subscribe();
    void unsubscribe();
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
    std_msgs::Header input_header_;
    ros::Subscriber sub_input_;
    ros::Subscriber sub_hint_;

    ros::Publisher marker_pub_;
    ros::Publisher debug_hint_centroid_pub_;
    ros::Publisher debug_plane_points_pub_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
  };
}

#endif
