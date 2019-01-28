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
#include "jsk_pcl_ros/normal_estimation_omp.h"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

namespace jsk_pcl_ros
{
  void NormalEstimationOMP::onInit()
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    DiagnosticNodelet::onInit();
    pnh_->param("number_of_threads", num_of_threads_, 0);
    NODELET_DEBUG_STREAM("num_of_threads: " << num_of_threads_);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&NormalEstimationOMP::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_with_xyz_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_with_xyz", 1);
    pub_latest_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/latest_time", 1);
    pub_average_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/average_time", 1);
    onInitPostProcess();
  }

  void NormalEstimationOMP::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &NormalEstimationOMP::estimate, this);
  }

  void NormalEstimationOMP::unsubscribe()
  {
    sub_.shutdown();
  }

  void NormalEstimationOMP::configCallback(
    Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    k_ = config.k_search;
    search_radius_ = config.radius_search;
  }

  void NormalEstimationOMP::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    {
      jsk_recognition_utils::ScopedWallDurationReporter r
        = timer_.reporter(pub_latest_time_, pub_average_time_);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *cloud);
      pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> impl(num_of_threads_);
      impl.setInputCloud(cloud);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr
        tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
      impl.setSearchMethod (tree);
      impl.setKSearch(k_);
      impl.setRadiusSearch(search_radius_);
      pcl::PointCloud<pcl::Normal>::Ptr
        normal_cloud(new pcl::PointCloud<pcl::Normal>);
      impl.compute(*normal_cloud);

      sensor_msgs::PointCloud2 ros_normal_cloud;
      pcl::toROSMsg(*normal_cloud, ros_normal_cloud);
      ros_normal_cloud.header = cloud_msg->header;
      pub_.publish(ros_normal_cloud);

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        normal_xyz(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      normal_xyz->points.resize(cloud->points.size());
      for (size_t i = 0; i < normal_xyz->points.size(); i++) {
        pcl::PointXYZRGBNormal p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        p.rgb = cloud->points[i].rgb;
        p.normal_x = normal_cloud->points[i].normal_x;
        p.normal_y = normal_cloud->points[i].normal_y;
        p.normal_z = normal_cloud->points[i].normal_z;
        normal_xyz->points[i] = p;
      }

      sensor_msgs::PointCloud2 ros_normal_xyz_cloud;
      pcl::toROSMsg(*normal_xyz, ros_normal_xyz_cloud);
      ros_normal_xyz_cloud.header = cloud_msg->header;
      pub_with_xyz_.publish(ros_normal_xyz_cloud);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::NormalEstimationOMP, nodelet::Nodelet);
