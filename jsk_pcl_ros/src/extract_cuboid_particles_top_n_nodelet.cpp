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
#include "jsk_pcl_ros/extract_cuboid_particles_top_n.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/pcl_ros_util.h"

namespace jsk_pcl_ros
{
  // Do nothing
  void ExtractCuboidParticlesTopN::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ExtractCuboidParticlesTopN::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<pcl_msgs::PointIndices>(*pnh_, "output", 1);
    pub_pose_array_ = advertise<jsk_recognition_msgs::WeightedPoseArray>(*pnh_, "output/pose_array", 1);
    pub_box_array_ = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "output/box_array", 1);
    onInitPostProcess();
  }

  void ExtractCuboidParticlesTopN::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ExtractCuboidParticlesTopN::extract, this);
  }

  void ExtractCuboidParticlesTopN::unsubscribe()
  {
    sub_.shutdown();
  }

  void ExtractCuboidParticlesTopN::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    top_n_ratio_ = config.top_n_ratio;
  }

  void ExtractCuboidParticlesTopN::extract(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    typename pcl::PointCloud<pcl::tracking::ParticleCuboid>::Ptr cloud(new pcl::PointCloud<pcl::tracking::ParticleCuboid>);
    typename pcl::PointCloud<pcl::tracking::ParticleCuboid>::Ptr cloud_filtered(new pcl::PointCloud<pcl::tracking::ParticleCuboid>);
    pcl::fromROSMsg(*msg, *cloud);
    // Sort particles
    std::sort(cloud->points.begin(), cloud->points.end(),
    compareParticleWeight<pcl::tracking::ParticleCuboid>);
    // Take top-n points
    double sum = 0;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    size_t i = 0;
    while (sum < top_n_ratio_ && i < cloud->points.size()) {
            //new_particles.push_back(cloud->points[i]);
        indices->indices.push_back((int)i);
        sum += cloud->points[i].weight;
        i++;
    }
    pcl::ExtractIndices<pcl::tracking::ParticleCuboid> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.filter(*cloud_filtered);
    publishPointIndices(pub_, *indices, msg->header);
    publishBoxArray(*cloud_filtered, msg->header);
    publishPoseArray(*cloud_filtered, msg->header);
  }

  void ExtractCuboidParticlesTopN::publishBoxArray(
    const pcl::PointCloud<pcl::tracking::ParticleCuboid>& particles,
    const std_msgs::Header& header)
  {
    if (pub_box_array_.getNumSubscribers() > 0) {
      jsk_recognition_msgs::BoundingBoxArray box_array;
      box_array.header = header;
      box_array.boxes.resize(particles.points.size());
      for (size_t i = 0; i < particles.points.size(); i++) {
        pcl::tracking::ParticleCuboid p = particles.points[i];
        jsk_recognition_msgs::BoundingBox box;
        box.header = header;
        Eigen::Affine3f pose = p.toEigenMatrix();
        tf::poseEigenToMsg(pose, box.pose);
        box.dimensions.x = p.dx;
        box.dimensions.y = p.dy;
        box.dimensions.z = p.dz;
        box.value = p.weight;
        box_array.boxes[i] = box;
      }
      pub_box_array_.publish(box_array);
    }
  }
  
  void ExtractCuboidParticlesTopN::publishPoseArray(
    const pcl::PointCloud<pcl::tracking::ParticleCuboid>& particles,
    const std_msgs::Header& header)
  {
    if (pub_pose_array_.getNumSubscribers() > 0) {
      jsk_recognition_msgs::WeightedPoseArray weighted_pose_array;
      weighted_pose_array.header = header;
      weighted_pose_array.poses.header = header;
      for (size_t i = 0; i < particles.points.size(); i++) {
        pcl::tracking::ParticleCuboid p = particles.points[i];
        geometry_msgs::Pose ros_pose;
        Eigen::Affine3f pose = p.toEigenMatrix();
        tf::poseEigenToMsg(pose, ros_pose);
        weighted_pose_array.poses.poses.push_back(ros_pose);
        weighted_pose_array.weights.push_back(p.weight);
      }
      pub_pose_array_.publish(weighted_pose_array);
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ExtractCuboidParticlesTopN, nodelet::Nodelet);
