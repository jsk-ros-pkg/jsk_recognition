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

#ifndef JSK_PCL_ROS_KINFU_H_
#define JSK_PCL_ROS_KINFU_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/containers/initialization.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <tf/transform_broadcaster.h>

namespace jsk_pcl_ros
{

  /**
   * O := odometory origin
   * M := map origin
   * C := camera pose
   *
   * M(t0) 
   *      R = I
   * O(t0)
   * 
   * C(t0)
   *      R = K(t) : estimated from kinfu
   * C(t)
   *
   * M(t)       O(t0)
   *      R =        R K(t)
   * C(t)       C(t0)
   *
   * M(t)   O(t)      O(t0)
   *      R      R =       R K(t)
   * O(t)   C(t)      C(t0)
   *
   * M(t)      O(t0)       O(t)
   *      R =       R K(t)      R^(-1)
   * O(t)      C(t0)       C(t)
   *
   **/

  class Kinfu: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    Kinfu(): DiagnosticNodelet("Kinfu") {}

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void callback(const sensor_msgs::Image::ConstPtr& depth_image,
                          const sensor_msgs::Image::ConstPtr& rgb_image);
    virtual void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    
    pcl::gpu::kinfuLS::KinfuTracker* kinfu_;
    pcl::gpu::kinfuLS::KinfuTracker::DepthMap depth_device_;
    pcl::gpu::kinfuLS::KinfuTracker::View colors_device_;
    pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device_;
    sensor_msgs::CameraInfo::ConstPtr info_msg_;
    ros::Subscriber sub_info_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_cloud_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_color_image_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::mutex mutex_;
    std::string parent_frame_id_;
    std::string child_frame_id_;
    std::string kinfu_origin_frame_id_;
    // transformation from odom to camera frame_id at t_0
    Eigen::Affine3f initial_camera_pose_;
    Eigen::Affine3f initial_kinfu_pose_;
    bool initial_camera_pose_acquired_;
    float volume_size_;
    float shift_distance_;
    int snapshot_rate_;
    bool initialized_;
    tf::TransformListener* tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
  };
}

#endif
