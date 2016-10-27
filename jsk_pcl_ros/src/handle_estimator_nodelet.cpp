// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/handle_estimator.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_pcl_ros
{
  void HandleEstimator::onInit()
  {
    ConnectionBasedNodelet::onInit();
    output_buf.resize(100);

    pnh_->param("gripper_size", gripper_size_, 0.08); // defaults to pr2 gripper size
    pnh_->param("approach_offset", approach_offset_, 0.1); // default to 10 cm
    pnh_->param("angle_divide_num", angle_divide_num_, 6); // even will be better
    pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "output", 1);
    pub_best_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output_best", 1);
    pub_selected_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output_selected", 1);

    pub_preapproach_ = advertise<geometry_msgs::PoseArray>(*pnh_, "output_preapproach", 1);
    pub_selected_preapproach_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output_selected_preapproach", 1);

    onInitPostProcess();
  }

  void HandleEstimator::subscribe()
  {
    sub_index_ = pnh_->subscribe<jsk_recognition_msgs::Int32Stamped>("selected_index", 1, boost::bind( &HandleEstimator::selectedIndexCallback, this, _1));
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_box_.subscribe(*pnh_, "input_box", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_box_);
    sync_->registerCallback(boost::bind(&HandleEstimator::estimate, this, _1, _2));
  }

  void HandleEstimator::unsubscribe()
  {
    sub_index_.shutdown();
    sub_input_.unsubscribe();
    sub_box_.unsubscribe();
  }
  
  void HandleEstimator::estimate(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                 const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg)
  {
    // pack dimensions into vector
    std::vector<double> dimensions;
    dimensions.push_back(box_msg->dimensions.x);
    dimensions.push_back(box_msg->dimensions.y);
    dimensions.push_back(box_msg->dimensions.z);
    size_t longest_index = 0;
    for (size_t i = 1; i < 3; i++) {
      if (dimensions[i] > dimensions[longest_index]) {
        longest_index = i;
      }
    }
    NODELET_INFO_STREAM("longest index is: " << longest_index);
    HandleType handle_type;
    // detect the handle type
    if (longest_index == 2) {
      if (dimensions[0] < gripper_size_ || dimensions[1] < gripper_size_) {
        handle_type = HANDLE_SMALL_ENOUGH_STAND_ON_PLANE;
      }
      else {
        handle_type = NO_HANDLE;
      }
    }
    else {
      if (longest_index == 0) {
        if (dimensions[1] < gripper_size_ || dimensions[2] < gripper_size_) {
          handle_type = HANDLE_SMALL_ENOUGH_LIE_ON_PLANE_X_LONGEST;
        }
        else {
          handle_type = NO_HANDLE;
        }
      }
      else {                    // 1
        if (dimensions[0] < gripper_size_ || dimensions[2] < gripper_size_) {
          handle_type = HANDLE_SMALL_ENOUGH_LIE_ON_PLANE_Y_LONGEST;
        }
        else {
          handle_type = NO_HANDLE;
        }
      }
      
    }
    estimateHandle(handle_type, cloud_msg, box_msg);
  }

  void HandleEstimator::estimateHandle(const HandleType& handle_type,
                                       const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                       const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg)
  {
    if (handle_type == NO_HANDLE) {
      NODELET_ERROR("failed to estimate handle");
    }
    else if (handle_type == HANDLE_SMALL_ENOUGH_LIE_ON_PLANE_Y_LONGEST) {
      handleSmallEnoughLieOnPlane(cloud_msg, box_msg, true);
    }
    else if (handle_type == HANDLE_SMALL_ENOUGH_LIE_ON_PLANE_X_LONGEST) {
      handleSmallEnoughLieOnPlane(cloud_msg, box_msg, false);
    }
    else if (handle_type == HANDLE_SMALL_ENOUGH_STAND_ON_PLANE) {
      handleSmallEnoughStandOnPlane(cloud_msg, box_msg);
    }
  }

  void HandleEstimator::handleSmallEnoughStandOnPlane(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg)
  {
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::PoseArray prepose_array;
    pose_array.header = box_msg->header;
    prepose_array.header = box_msg->header;
    Eigen::Affine3d center;
    tf::poseMsgToEigen(box_msg->pose, center);
    Eigen::Vector3d z = center.rotation().col(2);
    Plane p(Eigen::Vector3f(z[0], z[1], z[2]), 0);
    Eigen::Vector3d ray = center.translation().normalized();
    Eigen::Vector3d ray_projected;
    p.project(ray, ray_projected);
    ray_projected.normalize();
    
    Eigen::Matrix3d center_refined_mat;
    center_refined_mat.col(0) = ray_projected;
    center_refined_mat.col(2) = z;
    center_refined_mat.col(1) = z.cross(ray_projected);
    Eigen::Affine3d center_refined
      = Eigen::Translation3d(center.translation()) * Eigen::Quaterniond(center_refined_mat);

    const double angle_margin = 0.2;
    const double start_angle = M_PI / 2.0 + angle_margin;
    const double end_angle = M_PI + M_PI / 2.0 - angle_margin;
    for (size_t i = 0; i < angle_divide_num_; i++) {
      const double theta = (end_angle - start_angle) / angle_divide_num_ * i + start_angle;
      Eigen::Vector3d offset_vec = Eigen::Vector3d(cos(theta), sin(theta), 0);
      Eigen::Vector3d pre_approach_pos
        = (center_refined * (approach_offset_ * offset_vec));
      Eigen::Matrix3d orientation;
      Eigen::Vector3d x_vec
        = (center_refined.translation() - pre_approach_pos).normalized();
      Eigen::Vector3d y_vec = z.cross(x_vec);
      
      orientation.col(0) = x_vec;
      orientation.col(1) = y_vec;
      orientation.col(2) = z;

      Eigen::Affine3d pre_grasp_pose = Eigen::Translation3d(pre_approach_pos) * Eigen::Quaterniond(orientation);
      Eigen::Affine3d grasp_pose = Eigen::Translation3d(center.translation()) * Eigen::Quaterniond(orientation);
      geometry_msgs::Pose ros_prepose;
      geometry_msgs::Pose ros_pose;
      tf::poseEigenToMsg(pre_grasp_pose, ros_prepose);
      tf::poseEigenToMsg(grasp_pose, ros_pose);
      prepose_array.poses.push_back(ros_prepose);
      pose_array.poses.push_back(ros_pose);
    }
    pub_.publish(pose_array);
    pub_preapproach_.publish(prepose_array);
    geometry_msgs::PoseStamped best;
    best.header = pose_array.header;
    best.pose = pose_array.poses[pose_array.poses.size() / 2];
    pub_best_.publish(best);

    output_buf.push_front(boost::make_tuple(pose_array, prepose_array));
  }
  
  void HandleEstimator::handleSmallEnoughLieOnPlane(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg,
    bool y_longest)
  {
    geometry_msgs::PoseArray pose_array;
    geometry_msgs::PoseArray prepose_array;
    pose_array.header = box_msg->header;
    prepose_array.header = box_msg->header;
    // center
    Eigen::Affine3d center;
    tf::poseMsgToEigen(box_msg->pose, center);
    const double angle_margin = 0.2;
    const double start_angle = angle_margin;
    const double end_angle = M_PI - angle_margin;
    
    for (size_t i = 0; i <= angle_divide_num_; i++) { // angle_divide_num samples
      const double theta = (end_angle - start_angle) / angle_divide_num_ * i + start_angle;
      // x-z plane
      Eigen::Vector3d offset_vec;
      if (y_longest) {
        offset_vec = Eigen::Vector3d(sin(theta), 0, cos(theta));
      }
      else {
        offset_vec = Eigen::Vector3d(0, cos(theta), sin(theta));
      }
      Eigen::Vector3d pre_approach_pos
        = (center * (approach_offset_ * offset_vec));
      // compute unit vectors to construct orientation
      Eigen::Matrix3d orientation;
      if (y_longest) {
        Eigen::Vector3d x_vec
          = (center.translation() - pre_approach_pos).normalized();
        Eigen::Vector3d z_vec = center.rotation().col(1);
        Eigen::Vector3d y_vec = z_vec.cross(x_vec);
        orientation.col(0) = x_vec;
        orientation.col(1) = y_vec;
        orientation.col(2) = z_vec;
      }
      else {
        Eigen::Vector3d x_vec = (center.translation() - pre_approach_pos).normalized();
        Eigen::Vector3d z_vec = center.rotation().col(0);
        Eigen::Vector3d y_vec = z_vec.cross(x_vec);
        orientation.col(0) = x_vec;
        orientation.col(1) = y_vec;
        orientation.col(2) = z_vec;
      }
      
      Eigen::Affine3d pre_grasp_pose = Eigen::Translation3d(pre_approach_pos) * Eigen::Quaterniond(orientation);
      Eigen::Affine3d grasp_pose = Eigen::Translation3d(center.translation()) * Eigen::Quaterniond(orientation);
      geometry_msgs::Pose ros_prepose, ros_pose;
      tf::poseEigenToMsg(pre_grasp_pose, ros_prepose);
      tf::poseEigenToMsg(grasp_pose, ros_pose);
      pose_array.poses.push_back(ros_pose);
      prepose_array.poses.push_back(ros_prepose);
    }
    pub_.publish(pose_array);
    pub_preapproach_.publish(prepose_array);
    geometry_msgs::PoseStamped best;
    best.header = pose_array.header;
    best.pose = pose_array.poses[pose_array.poses.size() / 2];
    pub_best_.publish(best);

    output_buf.push_front(boost::make_tuple(pose_array, prepose_array));
  }
  
  void HandleEstimator::selectedIndexCallback( const jsk_recognition_msgs::Int32StampedConstPtr &index){
    boost::circular_buffer<boost::tuple<geometry_msgs::PoseArray, geometry_msgs::PoseArray> >::iterator it = output_buf.begin();
    while (it != output_buf.end()) {
      geometry_msgs::PoseArray pose_array = it->get<0>();
      geometry_msgs::PoseArray prepose_array = it->get<1>();

      if(pose_array.header.stamp == index->header.stamp){
	geometry_msgs::PoseStamped ps;
	ps.header = pose_array.header;
	ps.pose = pose_array.poses[index->data];
	pub_selected_.publish(ps);

	ps.header = prepose_array.header;
	ps.pose = prepose_array.poses[index->data];
	pub_selected_preapproach_.publish(ps);
	break;
      }
      ++it;
    }
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HandleEstimator, nodelet::Nodelet);
