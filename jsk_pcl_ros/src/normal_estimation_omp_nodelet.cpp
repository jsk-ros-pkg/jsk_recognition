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
#include <pcl/registration/lum.h>

namespace jsk_pcl_ros
{
  void NormalEstimationOMP::onInit()
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&NormalEstimationOMP::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_with_xyz_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_with_xyz", 1);
    pub_pose_with_covariance_stamped_ = advertise<geometry_msgs::PoseWithCovarianceStamped>(*pnh_, "output_pose_with_covariance_stamped", 1);
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> impl;
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
    Eigen::Vector3f ave = Eigen::Vector3f::Zero();
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < normal_xyz->points.size(); i++) {
      pcl::PointXYZRGBNormal p;
      p.x = cloud->points[i].x;
      p.y = cloud->points[i].y;
      p.z = cloud->points[i].z;
      p.normal_x = normal_cloud->points[i].normal_x;
      p.normal_y = normal_cloud->points[i].normal_y;
      p.normal_z = normal_cloud->points[i].normal_z;
      normal_xyz->points[i] = p;
      Eigen::Vector3f nv = Eigen::Vector3f(normal_cloud->points[i].normal_x, normal_cloud->points[i].normal_y, normal_cloud->points[i].normal_z);
      ave += nv;
      cov += nv * nv.transpose();
    }

    ave = ave / normal_xyz->points.size();
    cov = cov / normal_xyz->points.size() - ave * ave.transpose(); /* V[X] = E[X^2] - E[X]^2 */
    sensor_msgs::PointCloud2 ros_normal_xyz_cloud;
    pcl::toROSMsg(*normal_xyz, ros_normal_xyz_cloud);
    ros_normal_xyz_cloud.header = cloud_msg->header;
    pub_with_xyz_.publish(ros_normal_xyz_cloud);
    geometry_msgs::PoseWithCovarianceStamped pose_with_covariance_stamped_msg;
    {
      Eigen::Matrix3f covariance_matrix;
      Eigen::Vector4f centroid;
      pcl::computeMeanAndCovarianceMatrix(*cloud, covariance_matrix, centroid);
      /* set geometry_msgs::PoseWithCovarianceStamped.pose.pose */
      {
        geometry_msgs::Point tmp_p;
        geometry_msgs::Quaternion tmp_quat;
        tmp_p.x = centroid(0);
        tmp_p.y = centroid(1);
        tmp_p.z = centroid(2);
        Eigen::Vector3f ex, ey, ez;
        ez = ave;
        ex = Eigen::Vector3f::UnitY().cross(ez);
        ey = ez.cross(ex);
        Eigen::Matrix3f m;
        m << ex.normalized(), ey.normalized(), ez.normalized();
        Eigen::Quaternionf quat = Eigen::Quaternionf(m);
        tmp_quat.x = quat.x();
        tmp_quat.y = quat.y();
        tmp_quat.z = quat.z();
        tmp_quat.w = quat.w();
        pose_with_covariance_stamped_msg.pose.pose.position = tmp_p;
        pose_with_covariance_stamped_msg.pose.pose.orientation = tmp_quat;
      }
      /* set geometry_msgs::PoseWithCovarianceStamped.pose.covariance */
      {
        Eigen::Matrix6f tmp_cov = Eigen::Matrix6f::Zero();
        tmp_cov.block<3, 3>(0, 0) = covariance_matrix;
        tmp_cov.block<3, 3>(3, 3) = cov;
        for (size_t i = 0; i < 6; i++) {
          for (size_t j = 0; j < 6; j++) {
            pose_with_covariance_stamped_msg.pose.covariance[i*6+j] = tmp_cov(i, j);
          }
        }
      }
    }
    pose_with_covariance_stamped_msg.header = cloud_msg->header;
    pub_pose_with_covariance_stamped_.publish(pose_with_covariance_stamped_msg);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::NormalEstimationOMP, nodelet::Nodelet);
