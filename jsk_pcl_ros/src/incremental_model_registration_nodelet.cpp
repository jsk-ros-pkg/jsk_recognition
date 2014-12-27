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
#define BOOST_PARAMETER_MAX_ARITY 7

#include <sstream>
#include "jsk_pcl_ros/incremental_model_registration.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

namespace jsk_pcl_ros
{
  SampleData::SampleData()
  {

  }

  SampleData::SampleData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                         const Eigen::Affine3f& pose):
    original_cloud_(cloud), original_pose_(pose)
  {
    
  }

  void IncrementalModelRegistration::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_cloud_non_registered_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output/non_registered", 1);
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_pose_.subscribe(*pnh_, "input/pose", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_indices_, sub_pose_);
    sync_->registerCallback(boost::bind(
                              &IncrementalModelRegistration::newsampleCallback,
                              this, _1, _2, _3));
  }

  void IncrementalModelRegistration::transformPointCloudRepsectedToPose(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    Eigen::Affine3f posef;
    tf::poseMsgToEigen(pose_msg->pose, posef);
    // hard coded!
    Eigen::Affine3f offset = Eigen::Affine3f::Identity() * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
    Eigen::Affine3f transform = offset.inverse() * posef.inverse();
    
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
    SampleData::Ptr sample (new SampleData(transformed_cloud, initial_pose));
    samples_.push_back(sample);
    
    all_cloud_ = all_cloud_ + *transformed_cloud;
    sensor_msgs::PointCloud2 ros_all_cloud;
    pcl::toROSMsg(all_cloud_, ros_all_cloud);
    ros_all_cloud.header = cloud_msg->header;
    pub_cloud_non_registered_.publish(ros_all_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::IncrementalModelRegistration, nodelet::Nodelet);
