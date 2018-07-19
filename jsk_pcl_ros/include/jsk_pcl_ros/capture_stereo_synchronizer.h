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

#ifndef JSK_PCL_ROS_CAPTURE_STEREO_SYNCHRONIZER_H_
#define JSK_PCL_ROS_CAPTURE_STEREO_SYNCHRONIZER_H_




#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_msgs/PointIndices.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  class CaptureStereoSynchronizer: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<CaptureStereoSynchronizer> Ptr;
    typedef message_filters::sync_policies::ExactTime<
      geometry_msgs::PoseStamped, // pose of checker board
      sensor_msgs::Image,         // mask image
      pcl_msgs::PointIndices,     // mask indices
      sensor_msgs::Image,         // left image rect
      sensor_msgs::CameraInfo,    // left cmaera info
      sensor_msgs::CameraInfo,    // right camera info
      stereo_msgs::DisparityImage // stereo disparity
      > SyncPolicy;
    CaptureStereoSynchronizer(): DiagnosticNodelet("CaptureStereoSynchronizer") { }
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void republish(
      const geometry_msgs::PoseStamped::ConstPtr& pose, // pose of checker board
      const sensor_msgs::Image::ConstPtr& mask,         // mask image
      const PCLIndicesMsg::ConstPtr& mask_indices, // mask indices
      const sensor_msgs::Image::ConstPtr& left_image, // left image rect
      const sensor_msgs::CameraInfo::ConstPtr& left_cam_info, // left cmaera info
      const sensor_msgs::CameraInfo::ConstPtr& right_cam_info, // right camera info
      const stereo_msgs::DisparityImage::ConstPtr& disparity // stereo disparity
      );

    // check is there near pose or not
    // if there is a near pose, return true
    virtual bool checkNearPose(
      const geometry_msgs::Pose& new_pose);
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    int counter_;
    ros::Publisher pub_count_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_mask_;
    ros::Publisher pub_mask_indices_;
    ros::Publisher pub_left_image_;
    ros::Publisher pub_left_cam_info_;
    ros::Publisher pub_right_cam_info_;
    ros::Publisher pub_disparity_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_;
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_;
    message_filters::Subscriber<PCLIndicesMsg> sub_mask_indices_;
    message_filters::Subscriber<sensor_msgs::Image> sub_left_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_left_cam_info_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_right_cam_info_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disparity_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    std::vector<geometry_msgs::Pose> poses_;
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    double rotational_bin_size_;
    double positional_bin_size_;
  private:
  
  };
}
#endif
