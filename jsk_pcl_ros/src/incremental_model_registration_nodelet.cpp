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
#define BOOST_PARAMETER_MAX_ARITY 7

#include <sstream>
#include "jsk_pcl_ros/incremental_model_registration.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <jsk_recognition_msgs/ICPAlign.h>

namespace jsk_pcl_ros
{
  CapturedSamplePointCloud::CapturedSamplePointCloud()
  {

  }

  CapturedSamplePointCloud::CapturedSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                         const Eigen::Affine3f& pose):
    original_cloud_(cloud), original_pose_(pose),
    refined_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  CapturedSamplePointCloud::getOriginalPointCloud()
  {
    return original_cloud_;
  }

  Eigen::Affine3f CapturedSamplePointCloud::getOriginalPose()
  {
    return original_pose_;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  CapturedSamplePointCloud::getRefinedPointCloud()
  {
    return refined_cloud_;
  }

  Eigen::Affine3f CapturedSamplePointCloud::getRefinedPose()
  {
    return refined_pose_;
  }

  void CapturedSamplePointCloud::setRefinedPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB> cloud)
  {
    *refined_cloud_ = cloud;   // copying
  }

  void CapturedSamplePointCloud::setRefinedPose(
    Eigen::Affine3f pose)
  {
    refined_pose_ = Eigen::Affine3f(pose);
  }
  
  void IncrementalModelRegistration::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("frame_id", frame_id_,
                std::string("multisense/left_camera_optical_frame"));
    start_registration_srv_
      = pnh_->advertiseService(
        "start_registration", &IncrementalModelRegistration::startRegistration,
        this);
    pub_cloud_non_registered_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output/non_registered", 1);
    pub_registered_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output/registered", 1);
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_pose_.subscribe(*pnh_, "input/pose", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_indices_, sub_pose_);
    sync_->registerCallback(boost::bind(
                              &IncrementalModelRegistration::newsampleCallback,
                              this, _1, _2, _3));
    onInitPostProcess();
  }

  void IncrementalModelRegistration::transformPointCloudRepsectedToPose(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    Eigen::Affine3f posef;
    tf::poseMsgToEigen(pose_msg->pose, posef);
    Eigen::Affine3f transform = posef.inverse();
    
    pcl::transformPointCloud<pcl::PointXYZRGB>(
      *input, *output, transform);
  }
  
  void IncrementalModelRegistration::newsampleCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const pcl_msgs::PointIndices::ConstPtr& indices_msg,
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr
      indices (new pcl::PointIndices);
    indices->indices = indices_msg->indices;
    pcl::ExtractIndices<pcl::PointXYZRGB> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.filter(*filtered_cloud);
    transformPointCloudRepsectedToPose(
      filtered_cloud, transformed_cloud, pose_msg);
    Eigen::Affine3f initial_pose;
    if (samples_.size() == 0) {
      // first pointcloud, it will be the `origin`
      tf::poseMsgToEigen(pose_msg->pose, origin_);
      initial_pose = origin_;
    }
    else {
      // compute transformation from origin_
      Eigen::Affine3f offset;
      tf::poseMsgToEigen(pose_msg->pose, offset);
      initial_pose = origin_.inverse() * offset;
    }
    CapturedSamplePointCloud::Ptr sample (new CapturedSamplePointCloud(transformed_cloud, initial_pose));
    samples_.push_back(sample);
    
    all_cloud_ = all_cloud_ + *transformed_cloud;
    sensor_msgs::PointCloud2 ros_all_cloud;
    pcl::toROSMsg(all_cloud_, ros_all_cloud);
    ros_all_cloud.header = cloud_msg->header;
    pub_cloud_non_registered_.publish(ros_all_cloud);
  }

  void IncrementalModelRegistration::callICP(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
    Eigen::Affine3f& output_transform)
  {
    ros::ServiceClient icp
      = pnh_->serviceClient<jsk_recognition_msgs::ICPAlign>("icp_service");
    sensor_msgs::PointCloud2 reference_ros, target_ros;
    pcl::toROSMsg(*reference, reference_ros);
    pcl::toROSMsg(*target, target_ros);
    ros::Time now = ros::Time::now();
    reference_ros.header.stamp = target_ros.header.stamp = now;
    reference_ros.header.frame_id = target_ros.header.frame_id = "map";
    jsk_recognition_msgs::ICPAlign srv;
    srv.request.reference_cloud = reference_ros;
    srv.request.target_cloud = target_ros;
    icp.call(srv);
    tf::poseMsgToEigen(srv.response.result.pose,
                            output_transform);
  }
  
  bool IncrementalModelRegistration::startRegistration(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    if (samples_.size() <= 1) {
      ROS_ERROR("no enough samples");
      return false;
    }
    ROS_INFO("Starting registration %lu samples", samples_.size());
    // setup initial
    CapturedSamplePointCloud::Ptr initial_sample = samples_[0];
    initial_sample->setRefinedPointCloud(
      *(initial_sample->getOriginalPointCloud()));
    initial_sample->setRefinedPose(initial_sample->getOriginalPose());
    for (size_t i = 0; i < samples_.size() - 1; i++) {
      CapturedSamplePointCloud::Ptr from_sample = samples_[i];
      CapturedSamplePointCloud::Ptr to_sample = samples_[i+1];
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_cloud
        = from_sample->getRefinedPointCloud();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud
        = to_sample->getOriginalPointCloud();
      Eigen::Affine3f transform;
      callICP(reference_cloud, target_cloud, transform);
      to_sample->setRefinedPose(to_sample->getOriginalPose() * transform);
      // transform pointcloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud
        (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud<pcl::PointXYZRGB>(
        *target_cloud, *transformed_cloud, transform);
      to_sample->setRefinedPointCloud(*transformed_cloud);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registered_cloud
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_registered_cloud
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < samples_.size(); i++) {
      *registered_cloud = *(samples_[i]->getRefinedPointCloud()) + *registered_cloud;
      *non_registered_cloud = *(samples_[i]->getOriginalPointCloud()) + *non_registered_cloud;
    }
    sensor_msgs::PointCloud2 registered_ros_cloud, nonregistered_ros_cloud;
    pcl::toROSMsg(*registered_cloud, registered_ros_cloud);
    registered_ros_cloud.header.stamp = ros::Time::now();
    registered_ros_cloud.header.frame_id = frame_id_;
    pub_registered_.publish(registered_ros_cloud);
    pcl::toROSMsg(*non_registered_cloud, nonregistered_ros_cloud);
    nonregistered_ros_cloud.header.stamp = ros::Time::now();
    nonregistered_ros_cloud.header.frame_id = frame_id_;
    pub_cloud_non_registered_.publish(nonregistered_ros_cloud);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::IncrementalModelRegistration, nodelet::Nodelet);
