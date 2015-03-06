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
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace jsk_pcl_ros
{
  void TorusFinder::onInit()
  {
    DiagnosticNodelet::onInit();
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
    pub_inliers_ = advertise<PCLIndicesMsg>(*pnh_, "output/inliers", 1);
    pub_pose_stamped_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose", 1);
    pub_coefficients_ = advertise<PCLModelCoefficientMsg>(
      *pnh_, "output/coefficients", 1);
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
  }

  void TorusFinder::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &TorusFinder::segment, this);
  }

  void TorusFinder::unsubscribe()
  {
    sub_.shutdown();
  }

  void TorusFinder::segment(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud
      (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::SACSegmentation<pcl::PointNormal> seg;
    if (use_normal_) {
      pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::PointNormal> seg_normal;
      seg_normal.setInputNormals(cloud);
      seg = seg_normal;
    }
    seg.setOptimizeCoefficients (true);
    seg.setInputCloud(cloud);
    seg.setRadiusLimits(min_radius_, max_radius_);
    seg.setMethodType(pcl::SAC_RANSAC);
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
    }
    else {
      
      NODELET_INFO("failed to find torus");
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::TorusFinder, nodelet::Nodelet);
