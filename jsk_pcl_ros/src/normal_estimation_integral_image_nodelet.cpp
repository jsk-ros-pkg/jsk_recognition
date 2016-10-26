// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "jsk_pcl_ros/normal_estimation_integral_image.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/features/integral_image_normal.h>


namespace jsk_pcl_ros
{
  void NormalEstimationIntegralImage::onInit()
  {
    ConnectionBasedNodelet::onInit();

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&NormalEstimationIntegralImage::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_with_xyz_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_with_xyz", 1);

    onInitPostProcess();
  }

  void NormalEstimationIntegralImage::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &NormalEstimationIntegralImage::compute, this);
  }

  void NormalEstimationIntegralImage::unsubscribe()
  {
    sub_input_.shutdown();
  }

  void NormalEstimationIntegralImage::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    max_depth_change_factor_ = config.max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    depth_dependent_smoothing_ = config.depth_dependent_smoothing;
    estimation_method_ = config.estimation_method;
    border_policy_ignore_ = config.border_policy_ignore;
  }

  void NormalEstimationIntegralImage::compute(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *input);
    pcl::PointCloud<pcl::Normal> output;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    if (estimation_method_ == 0) {
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    }
    else if (estimation_method_ == 1) {
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    }
    else if (estimation_method_ == 2) {
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    }
    else {
      NODELET_FATAL("unknown estimation method: %d", estimation_method_);
      return;
    }

    if (border_policy_ignore_) {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::BORDER_POLICY_IGNORE);
    }
    else {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::BORDER_POLICY_MIRROR);
    }
    
    ne.setMaxDepthChangeFactor(max_depth_change_factor_);
    ne.setNormalSmoothingSize(normal_smoothing_size_);
    ne.setDepthDependentSmoothing(depth_dependent_smoothing_);
    ne.setInputCloud(input);
    ne.compute(output);
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(output, ros_output);
    pub_.publish(ros_output);

    // concatenate
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr concat(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    concat->points.resize(output.points.size());
    for (size_t i = 0; i < output.points.size(); i++) {
      pcl::PointXYZRGBNormal point;
      point.x = input->points[i].x;
      point.y = input->points[i].y;
      point.z = input->points[i].z;
      point.rgb = input->points[i].rgb;
      point.normal_x = output.points[i].normal_x;
      point.normal_y = output.points[i].normal_y;
      point.normal_z = output.points[i].normal_z;
      point.curvature = output.points[i].curvature;
      concat->points[i] = point;
    }
    concat->header = input->header;
    sensor_msgs::PointCloud2 ros_output_with_xyz;
    pcl::toROSMsg(*concat, ros_output_with_xyz);
    pub_with_xyz_.publish(ros_output_with_xyz);
  }
  
}


PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::NormalEstimationIntegralImage,
                        nodelet::Nodelet);
