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

#include "jsk_pcl_ros/icp_registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <eigen_conversions/eigen_msg.h>

namespace jsk_pcl_ros
{
  void ICPRegistration::onInit()
  {
    PCLNodelet::onInit();
    
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &ICPRegistration::configCallback, this, _1, _2);
    srv_->setCallback (f);

    
    pub_result_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>(
      "output_pose", 1);
    pub_result_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "output", 1);
    ////////////////////////////////////////////////////////
    // subscription
    ////////////////////////////////////////////////////////
    sub_reference_ = pnh_->subscribe("input_reference", 1,
                                     &ICPRegistration::referenceCallback,
                                     this);
    sub_ = pnh_->subscribe("input", 1,
                           &ICPRegistration::align,
                           this);
  }

  
  void ICPRegistration::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    algorithm_ = config.algorithm;
    max_iteration_ = config.max_iteration;
    correspondence_distance_ = config.correspondence_distance;
    transform_epsilon_ = config.transform_epsilon;
    euclidean_fittness_epsilon_ = config.euclidean_fittness_epsilon;
    rotation_epsilon_ = config.rotation_epsilon;
    maximum_optimizer_iterations_ = config.maximum_optimizer_iterations;
  }
  
  void ICPRegistration::align(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!reference_cloud_) {
      NODELET_FATAL("no reference is specified");
      return;
    }
    
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    // icp.setInputSource(cloud);
    // icp.setInputTarget(reference_cloud_);
    if (algorithm_ == 1) {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setRotationEpsilon(rotation_epsilon_);
      gicp.setCorrespondenceRandomness(correspondence_randomness_);
      gicp.setMaximumOptimizerIterations(maximum_optimizer_iterations_);
      icp = gicp;
    }
    icp.setInputSource(reference_cloud_);
    icp.setInputTarget(cloud);
    icp.setMaxCorrespondenceDistance (correspondence_distance_);
    icp.setMaximumIterations (max_iteration_);
    icp.setTransformationEpsilon (transform_epsilon_);
    icp.setEuclideanFitnessEpsilon (euclidean_fittness_epsilon_);
    pcl::PointCloud<PointT> final;
    icp.align(final);
    NODELET_INFO_STREAM("ICP converged: " << icp.hasConverged());
    NODELET_INFO_STREAM("ICP score: " << icp.getFitnessScore());
    
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    Eigen::Matrix4d transformation_d;
    convertMatrix4<Eigen::Matrix4f, Eigen::Matrix4d>(
      transformation, transformation_d);
    Eigen::Affine3d transform_affine (transformation_d);
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    tf::poseEigenToMsg(transform_affine, pose.pose);
    pub_result_pose_.publish(pose);
    // convert Eigen Matrix4f to Matrix4d
    sensor_msgs::PointCloud2 ros_final;
    pcl::toROSMsg(final, ros_final);
    ros_final.header = msg->header;
    pub_result_cloud_.publish(ros_final);
  }
  
  void ICPRegistration::referenceCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    {
      boost::mutex::scoped_lock lock(mutex_);
      reference_cloud_ = cloud;
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
typedef jsk_pcl_ros::ICPRegistration ICPRegistration;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ICPRegistration, ICPRegistration, nodelet::Nodelet);
