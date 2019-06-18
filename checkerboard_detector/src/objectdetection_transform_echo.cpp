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

#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <posedetection_msgs/ObjectDetection.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/exceptions.h>

tf::TransformListener* g_listener_;
ros::Publisher pub_pose_;
std::string frame_id_;

void getTransformation(const geometry_msgs::Pose& pose1,
                       const geometry_msgs::Pose& pose2,
                       Eigen::Vector3d& pos,
                       Eigen::Quaterniond& rot)
{
  Eigen::Affine3d affine1, affine2;
  tf::poseMsgToEigen(pose1, affine1);
  tf::poseMsgToEigen(pose2, affine2);
    
  // compute transformation between two transform...
  Eigen::Affine3d transform = affine1 * affine2.inverse();
  pos = Eigen::Vector3d(transform.translation());
  rot = Eigen::Quaterniond(transform.rotation());
}


void callback(const posedetection_msgs::ObjectDetection::ConstPtr& detection1,
              const posedetection_msgs::ObjectDetection::ConstPtr& detection2)
{
  if (detection1->objects.size() > 0 && detection2->objects.size() > 0) {
    geometry_msgs::Pose pose1 = detection1->objects[0].pose;
    geometry_msgs::Pose pose2 = detection2->objects[0].pose;
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    getTransformation(pose1, pose2, pos, rot);
    ROS_INFO("without tf resolving::");
    ROS_INFO("  pos: [%f %f %f]", pos[0], pos[1], pos[2]);
    ROS_INFO("  rot: [%f %f %f %f]", rot.x(), rot.y(), rot.z(), rot.w());
    if (detection1->header.frame_id != detection2->header.frame_id) {
      geometry_msgs::PoseStamped pose2_stamped, pose2_stamped_transformed;
      pose2_stamped.header = detection2->header;
      pose2_stamped.pose = pose2;
      try {
        g_listener_->transformPose(detection1->header.frame_id,
                                   pose2_stamped,
                                   pose2_stamped_transformed);
        geometry_msgs::Pose pose2_transformed = pose2_stamped_transformed.pose;
        Eigen::Vector3d pos_transformed;
        Eigen::Quaterniond rot_transformed;
        getTransformation(pose1, pose2_transformed, pos_transformed, rot_transformed);

        geometry_msgs::PoseStamped pose_out;
        pose_out.header.stamp = detection2->header.stamp;
        pose_out.header.frame_id = frame_id_;
        pose_out.pose.position.x = pos_transformed[0];
        pose_out.pose.position.y = pos_transformed[1];
        pose_out.pose.position.z = pos_transformed[2];
        pose_out.pose.orientation.x = rot_transformed.x();
        pose_out.pose.orientation.y = rot_transformed.y();
        pose_out.pose.orientation.z = rot_transformed.z();
        pose_out.pose.orientation.w = rot_transformed.w();
        pub_pose_.publish(pose_out);

        ROS_INFO("with tf resolving::");
        ROS_INFO("  pos: [%f %f %f]", pos_transformed[0], pos_transformed[1], pos_transformed[2]);
        ROS_INFO("  rot: [%f %f %f %f]", rot_transformed.x(), rot_transformed.y(), rot_transformed.z(), rot_transformed.w());
      }
      catch(tf2::ConnectivityException &e) {
        ROS_ERROR("failed to resolve tf: %s", e.what());
      }
      catch(tf2::InvalidArgumentException &e) {
        ROS_ERROR("failed to resolve tf: %s", e.what());
      }
      catch(tf2::TransformException &e) {
        ROS_ERROR("failed to resolve tf: %s", e.what());
      }
    }
    
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "objectdetection_transform_echo");
  g_listener_ = new tf::TransformListener();
  typedef message_filters::sync_policies::ApproximateTime<
    posedetection_msgs::ObjectDetection, posedetection_msgs::ObjectDetection>
    SyncPolicy;

  ros::NodeHandle nh;

  nh.param<std::string>("frame_id", frame_id_, "");

  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

  message_filters::Subscriber<posedetection_msgs::ObjectDetection>
    detection1_sub(nh, "detection1", 1);
  message_filters::Subscriber<posedetection_msgs::ObjectDetection>
    detection2_sub(nh, "detection2", 1);

  message_filters::Synchronizer<SyncPolicy> sync(
    SyncPolicy(1000),
    detection1_sub, detection2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
  return 0;
}
