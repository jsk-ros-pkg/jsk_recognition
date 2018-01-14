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

#include "jsk_pcl_ros_utils/pose_with_covariance_stamped_to_gaussian_pointcloud.h"
#include <sensor_msgs/PointCloud2.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros_utils
{
  void PoseWithCovarianceStampedToGaussianPointCloud::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PoseWithCovarianceStampedToGaussianPointCloud::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void PoseWithCovarianceStampedToGaussianPointCloud::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &PoseWithCovarianceStampedToGaussianPointCloud::convert, this);
  }

  void PoseWithCovarianceStampedToGaussianPointCloud::unsubscribe()
  {
    sub_.shutdown();
  }

  float PoseWithCovarianceStampedToGaussianPointCloud::gaussian(const Eigen::Vector2f& input,
                                                                const Eigen::Vector2f& mean,
                                                                const Eigen::Matrix2f& S,
                                                                const Eigen::Matrix2f& S_inv)
  {
    Eigen::Vector2f diff = input - mean;
    if (normalize_method_ == "normalize_area") {
      return normalize_value_ * 1 / (2 * M_PI * sqrt(S.determinant())) * exp(- 0.5 * (diff.transpose() * S_inv * diff)[0]);
    }
    else if (normalize_method_ == "normalize_height") {
      return normalize_value_ * exp(- 0.5 * (diff.transpose() * S_inv * diff)[0]);
    }
  }

  void PoseWithCovarianceStampedToGaussianPointCloud::convert(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector2f mean;
    Eigen::Matrix2f S;
    if (cut_plane_ == "xy" || cut_plane_ == "flipped_xy") {
      mean = Eigen::Vector2f(msg->pose.pose.position.x, msg->pose.pose.position.y);
      S << msg->pose.covariance[0], msg->pose.covariance[1], 
        msg->pose.covariance[6], msg->pose.covariance[7];
    }
    else if (cut_plane_ == "yz" || cut_plane_ == "flipped_yz") {
      mean = Eigen::Vector2f(msg->pose.pose.position.y, msg->pose.pose.position.z);
      S << msg->pose.covariance[7], msg->pose.covariance[8], 
        msg->pose.covariance[13], msg->pose.covariance[14];
    }
    else if (cut_plane_ == "zx" || cut_plane_ == "flipped_zx") {
      mean = Eigen::Vector2f(msg->pose.pose.position.z, msg->pose.pose.position.x);
      S << msg->pose.covariance[14], msg->pose.covariance[12], 
        msg->pose.covariance[0], msg->pose.covariance[2];
    }
    double max_sigma = 0;
    for (size_t i = 0; i < 2; i++) {
      for (size_t j = 0; j < 2; j++) {
        double sigma = sqrt(S(i, j));
        if (max_sigma < sigma) {
          max_sigma = sigma;
        }
      }
    }
    Eigen::Matrix2f S_inv = S.inverse();
    double dx = 6.0 * max_sigma / sampling_num_;
    double dy = 6.0 * max_sigma / sampling_num_;
    for (double x = - 3.0 * max_sigma; x <= 3.0 * max_sigma; x += dx) {
      for (double y = - 3.0 * max_sigma; y <= 3.0 * max_sigma; y += dy) {
        Eigen::Vector2f diff(x, y);
        Eigen::Vector2f input = diff + mean;
        float z = gaussian(input, mean, S, S_inv);
        pcl::PointXYZ p;
        if (cut_plane_ == "xy") {
          p.x = input[0];
          p.y = input[1];
          p.z = z + msg->pose.pose.position.z;
        }
        else if (cut_plane_ == "yz") {
          p.y = input[0];
          p.z = input[1];
          p.x = z + msg->pose.pose.position.x;
        }
        else if (cut_plane_ == "zx") {
          p.z = input[0];
          p.x = input[1];
          p.y = z + msg->pose.pose.position.y;
        }
        else if (cut_plane_ == "flipped_xy") {
          p.x = input[0];
          p.y = input[1];
          p.z = -z + msg->pose.pose.position.z;
        }
        else if (cut_plane_ == "flipped_yz") {
          p.y = input[0];
          p.z = input[1];
          p.x = -z + msg->pose.pose.position.x;
        }
        else if (cut_plane_ == "flipped_zx") {
          p.z = input[0];
          p.x = input[1];
          p.y = -z + msg->pose.pose.position.y;
        }
        cloud->points.push_back(p);
      }
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = msg->header;
    pub_.publish(ros_cloud);
  }
  
  void PoseWithCovarianceStampedToGaussianPointCloud::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cut_plane_ = config.cut_plane;
    sampling_num_ = config.sampling_num;
    normalize_value_ = config.normalize_value;
    normalize_method_ = config.normalize_method;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PoseWithCovarianceStampedToGaussianPointCloud,
                        nodelet::Nodelet);
