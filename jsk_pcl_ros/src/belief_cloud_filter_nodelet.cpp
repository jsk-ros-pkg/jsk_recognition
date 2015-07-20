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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/belief_cloud_filter.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace jsk_pcl_ros
{

  void BeliefCloudFilter::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("initial_model", initial_model_, std::string("none"));
    if (!setupInitialModel()) {
      JSK_NODELET_FATAL("Failed to setup initial model");
      return;
    }
    pnh_->param("belief_cloud_dx", belief_cloud_dx_, 0.05);
    pnh_->param("belief_cloud_dy", belief_cloud_dy_, 0.05);
    pnh_->param("belief_cloud_dz", belief_cloud_dz_, 0.05);
    pnh_->param("belief_cloud_size_x", belief_cloud_size_x_, 5.0);
    pnh_->param("belief_cloud_size_y", belief_cloud_size_y_, 5.0);
    pnh_->param("belief_cloud_size_z", belief_cloud_size_z_, 5.0);
    pnh_->param("cutoff_threshold", cutoff_threshold_, 0.1);
    pub_initial_model_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "initial_model", 1);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
  }
  
  void BeliefCloudFilter::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &BeliefCloudFilter::filter, this);
  }

  void BeliefCloudFilter::unsubscribe()
  {
    sub_.shutdown();
  }

  bool BeliefCloudFilter::setupInitialModel()
  {
    if (initial_model_ == belief_cloud_initial_model::gaussian) {
      if (jsk_topic_tools::readVectorParameter(*pnh_, "gaussian_means", gaussian_means_) &&
          jsk_topic_tools::readVectorParameter(*pnh_, "gaussian_covariances", gaussian_covariances_) &&
          gaussian_means_.size() == 3 &&
          gaussian_covariances_.size() == 3) {
        return true;
      }
      else {
        JSK_NODELET_FATAL("~gaussian_means and ~gaussian_covariances should be provided");
        return false;
      }
    }
    else {
      JSK_NODELET_FATAL("%s is not supported as initial model", initial_model_.c_str());
      return false;
    }
  }

  void BeliefCloudFilter::filter(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr belief_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZ p = cloud->points[i];
      if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
        continue;
      }
      double prob = gaussian(p.x, gaussian_means_[0], gaussian_covariances_[0]) * 
        gaussian(p.y, gaussian_means_[1], gaussian_covariances_[1]) * 
        gaussian(p.z, gaussian_means_[2], gaussian_covariances_[2]);
      if (prob > cutoff_threshold_) {
        pcl::PointXYZI b;
        b.x = p.x;
        b.y = p.y;
        b.z = p.z;
        b.intensity = prob;
        belief_cloud->points.push_back(b);
      }
    }
    sensor_msgs::PointCloud2 ros_belief_cloud;
    pcl::toROSMsg(*belief_cloud, ros_belief_cloud);
    ros_belief_cloud.header = cloud_msg->header;
    pub_.publish(ros_belief_cloud);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr belief_density_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (double x = - belief_cloud_size_x_; x <= belief_cloud_size_x_; x += belief_cloud_dx_) {
      for (double y = - belief_cloud_size_y_; y <= belief_cloud_size_y_; y += belief_cloud_dy_) {
        for (double z = - belief_cloud_size_z_; z <= belief_cloud_size_z_; z += belief_cloud_dz_) {
          double prob = gaussian(x, gaussian_means_[0], gaussian_covariances_[0]) * 
            gaussian(y, gaussian_means_[1], gaussian_covariances_[1]) * 
            gaussian(z, gaussian_means_[2], gaussian_covariances_[2]);
          if (prob > cutoff_threshold_) {
            pcl::PointXYZI b;
            b.x = x;
            b.y = y;
            b.z = z;
            b.intensity = prob;
            belief_density_cloud->points.push_back(b);
          }
        }
      }
    }
    sensor_msgs::PointCloud2 ros_belief_density_cloud;
    pcl::toROSMsg(*belief_density_cloud, ros_belief_density_cloud);
    ros_belief_density_cloud.header = cloud_msg->header;
    pub_initial_model_.publish(ros_belief_density_cloud);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::BeliefCloudFilter, nodelet::Nodelet);
