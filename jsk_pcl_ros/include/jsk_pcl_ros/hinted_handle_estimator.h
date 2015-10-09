// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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


#ifndef JSK_PCL_ROS_HINTED_HANDLE_ESTIMATOR_H_
#define JSK_PCL_ROS_HINTED_HANDLE_ESTIMATOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/SimpleHandle.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


class handle_model
{
public:
  double finger_l;
  double finger_d;
  double finger_w;
  double arm_l;
  double arm_d;
  double arm_w;
  handle_model(){
  }
};

tf::Transform pose_to_tf(geometry_msgs::Pose pose){
  return tf::Transform(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}
geometry_msgs::Pose tf_to_pose(tf::Transform transform){
  geometry_msgs::Pose pose;
  tf::Quaternion q;
  transform.getBasis().getRotation(q);
  pose.orientation.x = q.getX(); pose.orientation.y=q.getY(); pose.orientation.z=q.getZ(), pose.orientation.w=q.getW();
  pose.position.x=transform.getOrigin().getX(), pose.position.y=transform.getOrigin().getY(), pose.position.z=transform.getOrigin().getZ();
  return pose;
}
geometry_msgs::Pose change_pose(geometry_msgs::Pose base_pose, geometry_msgs::Pose child_pose){
  return tf_to_pose(pose_to_tf(base_pose)*pose_to_tf(child_pose));
}

visualization_msgs::Marker make_box(float s_x, float s_y, float s_z, float r, float g, float b, float x, float y, float z)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = s_x;
  marker.scale.y = s_y;
  marker.scale.z = s_z;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1.0;
  return marker;
}

visualization_msgs::MarkerArray make_handle_array(geometry_msgs::PoseStamped pose, handle_model handle){
  visualization_msgs::MarkerArray markerArray_msg;
  visualization_msgs::Marker marker_msg = make_box(handle.finger_l, handle.finger_w, handle.finger_d, 1.0, 0, 0, handle.finger_l*0.5, handle.arm_w*0.5 -handle.finger_w*0.5, 0);
  marker_msg.pose = change_pose(pose.pose, marker_msg.pose);
  marker_msg.header = pose.header;
  marker_msg.ns = std::string("debug_grasp_model");
  marker_msg.id = 1;
  markerArray_msg.markers.push_back(marker_msg);
  marker_msg = make_box(handle.finger_l, handle.finger_w, handle.finger_d, 0, 1.0, 0, handle.finger_l*0.5, -handle.arm_w*0.5 +handle.finger_w*0.5, 0);
  marker_msg.pose = change_pose(pose.pose, marker_msg.pose);
  marker_msg.header = pose.header;
  marker_msg.ns = std::string("debug_grasp_model");
  marker_msg.id = 2;
  markerArray_msg.markers.push_back(marker_msg);
  marker_msg = make_box(handle.arm_l-handle.finger_l, handle.arm_w, handle.arm_d, 0, 0, 1.0, -(handle.arm_l-handle.finger_l)*0.5, 0, 0);
  marker_msg.pose = change_pose(pose.pose, marker_msg.pose);
  marker_msg.header = pose.header;
  marker_msg.ns = std::string("debug_grasp_model");
  marker_msg.id = 3;
  markerArray_msg.markers.push_back(marker_msg);
  return markerArray_msg;
}

namespace jsk_pcl_ros
{
  class HintedHandleEstimator: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    HintedHandleEstimator(): DiagnosticNodelet("HintedHandleEstimator") {}
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      geometry_msgs::PointStamped 
      > SyncPolicy;
    tf::TransformListener listener_;
    handle_model handle;
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void estimate(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const geometry_msgs::PointStampedConstPtr &point_msg);

    boost::mutex mutex_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_length_;
    ros::Publisher pub_handle_;
    ros::Publisher pub_debug_marker_;
    ros::Publisher pub_debug_marker_array_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<geometry_msgs::PointStamped> sub_point_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
  private:
    
  };
}

#endif
