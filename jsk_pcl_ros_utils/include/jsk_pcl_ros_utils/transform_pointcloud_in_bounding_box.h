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


#ifndef JSK_PCL_ROS_UTILS_TRANSFORM_POINTCLOUD_IN_BOUNDING_BOX_H_
#define JSK_PCL_ROS_UTILS_TRANSFORM_POINTCLOUD_IN_BOUNDING_BOX_H_

#include <pcl_ros/pcl_nodelet.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "jsk_recognition_utils/tf_listener_singleton.h"
#include <eigen_conversions/eigen_msg.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/common/transforms.h>

namespace jsk_pcl_ros_utils
{
  template <class PointT>
  void transformPointcloudInBoundingBox(
    const jsk_recognition_msgs::BoundingBox& box_msg,
    const sensor_msgs::PointCloud2& cloud_msg,
    pcl::PointCloud<PointT>& output,
    Eigen::Affine3f& offset,
    tf::TransformListener& tf_listener)
  {
    geometry_msgs::PoseStamped box_pose;
    box_pose.header = box_msg.header;
    box_pose.pose = box_msg.pose;
    // transform box_pose into msg frame
    geometry_msgs::PoseStamped box_pose_respected_to_cloud;
    tf_listener.transformPose(cloud_msg.header.frame_id,
                                box_pose,
                                box_pose_respected_to_cloud);
    // convert the pose into eigen
    Eigen::Affine3d box_pose_respected_to_cloud_eigend;
    tf::poseMsgToEigen(box_pose_respected_to_cloud.pose,
                       box_pose_respected_to_cloud_eigend);
    Eigen::Affine3d box_pose_respected_to_cloud_eigend_inversed
      = box_pose_respected_to_cloud_eigend.inverse();
    Eigen::Matrix4f box_pose_respected_to_cloud_eigen_inversed_matrixf;
    Eigen::Matrix4d box_pose_respected_to_cloud_eigen_inversed_matrixd
      = box_pose_respected_to_cloud_eigend_inversed.matrix();
    jsk_recognition_utils::convertMatrix4<Eigen::Matrix4d, Eigen::Matrix4f>(
      box_pose_respected_to_cloud_eigen_inversed_matrixd,
      box_pose_respected_to_cloud_eigen_inversed_matrixf);
    offset = Eigen::Affine3f(box_pose_respected_to_cloud_eigen_inversed_matrixf);
    pcl::PointCloud<PointT> input;
    pcl::fromROSMsg(cloud_msg, input);
    pcl::transformPointCloud(input, output, offset);
  }
  
  class TransformPointcloudInBoundingBox: public pcl_ros::PCLNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::BoundingBox > SyncPolicy;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void transform(const sensor_msgs::PointCloud2::ConstPtr& msg,
                           const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg);
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> sub_box_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_offset_pose_;
    tf::TransformListener* tf_listener_;
  private:
    
  };
}

#endif
