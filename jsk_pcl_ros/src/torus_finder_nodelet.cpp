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
#include "jsk_pcl_ros/torus_finder.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

namespace jsk_pcl_ros
{
  void TorusFinder::onInit()
  {
    DiagnosticNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    pnh_->param("use_hint", use_hint_, false);
    if (use_hint_) {
      if (pnh_->hasParam("initial_axis_hint")) {
        std::vector<double> axis;
        jsk_topic_tools::readVectorParameter(*pnh_, "initial_axis_hint", axis);
        if (axis.size() == 3) {
          hint_axis_[0] = axis[0];
          hint_axis_[1] = axis[1];
          hint_axis_[2] = axis[2];
        }
        else {
          hint_axis_[0] = 0;
          hint_axis_[1] = 0;
          hint_axis_[2] = 1;
        }
      }
    }
    pnh_->param("use_normal", use_normal_, false);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&TorusFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_torus_ = advertise<jsk_recognition_msgs::Torus>(*pnh_, "output", 1);
    pub_torus_array_ = advertise<jsk_recognition_msgs::TorusArray>(*pnh_, "output/array", 1);
    pub_torus_with_failure_ = advertise<jsk_recognition_msgs::Torus>(*pnh_, "output/with_failure", 1);
    pub_torus_array_with_failure_ = advertise<jsk_recognition_msgs::TorusArray>(*pnh_, "output/with_failure/array", 1);
    pub_inliers_ = advertise<PCLIndicesMsg>(*pnh_, "output/inliers", 1);
    pub_pose_stamped_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose", 1);
    pub_coefficients_ = advertise<PCLModelCoefficientMsg>(
      *pnh_, "output/coefficients", 1);
    pub_latest_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/latest_time", 1);
    pub_average_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/average_time", 1);

    done_initialization_ = true;
    onInitPostProcess();
  }

  void TorusFinder::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_radius_ = config.min_radius;
    max_radius_ = config.max_radius;
    outlier_threshold_ = config.outlier_threshold;
    max_iterations_ = config.max_iterations;
    min_size_ = config.min_size;
    eps_hint_angle_ = config.eps_hint_angle;
    algorithm_ = config.algorithm;
    voxel_grid_sampling_ = config.voxel_grid_sampling;
    voxel_size_ = config.voxel_size;
  }

  void TorusFinder::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &TorusFinder::segment, this);
    sub_points_ = pnh_->subscribe("input/polygon", 1, &TorusFinder::segmentFromPoints, this);
  }

  void TorusFinder::unsubscribe()
  {
    sub_.shutdown();
    sub_points_.shutdown();
  }

  void TorusFinder::segmentFromPoints(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg)
  {
    if (!done_initialization_) {
      return;
    }
    // Convert to pointcloud and call it
    // No normal
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud
      (new pcl::PointCloud<pcl::PointNormal>);
    for (size_t i = 0; i < polygon_msg->polygon.points.size(); i++) {
      geometry_msgs::Point32 p = polygon_msg->polygon.points[i];
      pcl::PointNormal pcl_point;
      pcl_point.x = p.x;
      pcl_point.y = p.y;
      pcl_point.z = p.z;
      cloud->points.push_back(pcl_point);
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = polygon_msg->header;
    segment(boost::make_shared<sensor_msgs::PointCloud2>(ros_cloud));
  }
  
  void TorusFinder::segment(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    if (!done_initialization_) {
      return;
    }
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud
      (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    jsk_recognition_utils::ScopedWallDurationReporter r
      = timer_.reporter(pub_latest_time_, pub_average_time_);
    if (voxel_grid_sampling_) {
      pcl::PointCloud<pcl::PointNormal>::Ptr downsampled_cloud
      (new pcl::PointCloud<pcl::PointNormal>);
      pcl::VoxelGrid<pcl::PointNormal> sor;
      sor.setInputCloud (cloud);
      sor.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
      sor.filter (*downsampled_cloud);
      cloud = downsampled_cloud;
    }
    
    pcl::SACSegmentation<pcl::PointNormal> seg;
    if (use_normal_) {
      pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg_normal;
      seg_normal.setInputNormals(cloud);
      seg = seg_normal;
    }

    
    seg.setOptimizeCoefficients (true);
    seg.setInputCloud(cloud);
    seg.setRadiusLimits(min_radius_, max_radius_);
    if (algorithm_ == "RANSAC") {
      seg.setMethodType(pcl::SAC_RANSAC);
    }
    else if (algorithm_ == "LMEDS") {
      seg.setMethodType(pcl::SAC_LMEDS);
    }
    else if (algorithm_ == "MSAC") {
      seg.setMethodType(pcl::SAC_MSAC);
    }
    else if (algorithm_ == "RRANSAC") {
      seg.setMethodType(pcl::SAC_RRANSAC);
    }
    else if (algorithm_ == "RMSAC") {
      seg.setMethodType(pcl::SAC_RMSAC);
    }
    else if (algorithm_ == "MLESAC") {
      seg.setMethodType(pcl::SAC_MLESAC);
    }
    else if (algorithm_ == "PROSAC") {
      seg.setMethodType(pcl::SAC_PROSAC);
    }
    
    seg.setDistanceThreshold (outlier_threshold_);
    seg.setMaxIterations (max_iterations_);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    if (use_hint_) {
      seg.setAxis(hint_axis_);
      seg.setEpsAngle(eps_hint_angle_);
    }
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);
    NODELET_INFO("input points: %lu", cloud->points.size());
    if (inliers->indices.size() > min_size_) {
      // check direction. Torus should direct to origin of pointcloud
      // always.
      Eigen::Vector3f dir(coefficients->values[4],
                          coefficients->values[5],
                          coefficients->values[6]);
      if (dir.dot(Eigen::Vector3f::UnitZ()) < 0) {
        dir = - dir;
      }
      
      Eigen::Affine3f pose = Eigen::Affine3f::Identity();
      Eigen::Vector3f pos = Eigen::Vector3f(coefficients->values[0],
                                            coefficients->values[1],
                                            coefficients->values[2]);
      Eigen::Quaternionf rot;
      rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), dir);
      pose = pose * Eigen::Translation3f(pos) * Eigen::AngleAxisf(rot);
      PCLIndicesMsg ros_inliers;
      ros_inliers.indices = inliers->indices;
      ros_inliers.header = cloud_msg->header;
      pub_inliers_.publish(ros_inliers);
      PCLModelCoefficientMsg ros_coefficients;
      ros_coefficients.values = coefficients->values;
      ros_coefficients.header = cloud_msg->header;
      pub_coefficients_.publish(ros_coefficients);
      jsk_recognition_msgs::Torus torus_msg;
      torus_msg.header = cloud_msg->header;
      tf::poseEigenToMsg(pose, torus_msg.pose);
      torus_msg.small_radius = 0.01;
      torus_msg.large_radius = coefficients->values[3];
      pub_torus_.publish(torus_msg);
      jsk_recognition_msgs::TorusArray torus_array_msg;
      torus_array_msg.header = cloud_msg->header;
      torus_array_msg.toruses.push_back(torus_msg);
      pub_torus_array_.publish(torus_array_msg);
      // publish pose stamped
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = torus_msg.header;
      pose_stamped.pose = torus_msg.pose;
      pub_pose_stamped_.publish(pose_stamped);
      pub_torus_array_with_failure_.publish(torus_array_msg);
      pub_torus_with_failure_.publish(torus_msg);
    }
    else {
      jsk_recognition_msgs::Torus torus_msg;
      torus_msg.header = cloud_msg->header;
      torus_msg.failure = true;
      pub_torus_with_failure_.publish(torus_msg);
      jsk_recognition_msgs::TorusArray torus_array_msg;
      torus_array_msg.header = cloud_msg->header;
      torus_array_msg.toruses.push_back(torus_msg);
      pub_torus_array_with_failure_.publish(torus_array_msg);
      NODELET_INFO("failed to find torus");
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::TorusFinder, nodelet::Nodelet);
