// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/color_histogram_matcher.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/centroid.h>

#include <geometry_msgs/PoseStamped.h>

namespace jsk_pcl_ros
{
  void ColorHistogramMatcher::onInit()
  {
    PCLNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ColorHistogramMatcher::configCallback, this, _1, _2);
    srv_->setCallback (f);

    policy_ = USE_HUE_AND_SATURATION;
    reference_set_ = false;
    // setup publishers
    all_histogram_pub_
      = pnh_->advertise<jsk_pcl_ros::ColorHistogramArray>("output_histograms", 1);
    best_pub_
      = pnh_->advertise<geometry_msgs::PoseStamped>("best_match", 1);
    reference_histogram_pub_
      = pnh_->advertise<jsk_pcl_ros::ColorHistogram>("output_reference", 1);
    result_pub_
      = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>("output", 1);
    reference_sub_ = pnh_->subscribe("input_reference_cloud", 1,
                                     &ColorHistogramMatcher::reference,
                                     this);
    reference_histogram_sub_ = pnh_->subscribe(
      "input_reference", 1,
      &ColorHistogramMatcher::referenceHistogram,
      this);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_indices_);
    sync_->registerCallback(boost::bind(&ColorHistogramMatcher::feature,
                                        this, _1, _2));
  }

  void ColorHistogramMatcher::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    coefficient_thr_ = config.coefficient_thr;
    bin_size_ = config.bin_size;
    ComparePolicy new_histogram;
    if (config.histogram_method == 0) {
      new_histogram = USE_HUE;
    }
    else if (config.histogram_method == 1) {
      new_histogram = USE_SATURATION;
    }
    else if (config.histogram_method == 2) {
      new_histogram = USE_VALUE;
    }
    else if (config.histogram_method == 3) {
      new_histogram = USE_HUE_AND_SATURATION;
    }
    else {
      ROS_WARN("unknown histogram method");
      return;
    }
    if (new_histogram != policy_) {
      policy_ = new_histogram;
      reference_set_ = false;
      ROS_WARN("histogram method is reset, please specify histogram again");
    }
  }

  
  void ColorHistogramMatcher::feature(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
      const jsk_pcl_ros::ClusterPointIndices::ConstPtr& input_indices)
  {
    boost::mutex::scoped_lock(mutex_);
    if (!reference_set_) {
      NODELET_WARN("reference histogram is not available yet");
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*pcl_cloud, *hsv_cloud);
    for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
      hsv_cloud->points[i].x = pcl_cloud->points[i].x;
      hsv_cloud->points[i].y = pcl_cloud->points[i].y;
      hsv_cloud->points[i].z = pcl_cloud->points[i].z;
    }
    // compute histograms first
    std::vector<std::vector<float> > histograms;
    histograms.resize(input_indices->cluster_indices.size());
    
    pcl::ExtractIndices<pcl::PointXYZHSV> extract;
    extract.setInputCloud(hsv_cloud);
    // for debug
    jsk_pcl_ros::ColorHistogramArray histogram_array;
    histogram_array.header = input_cloud->header;
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr > segmented_clouds;
    for (size_t i = 0; i < input_indices->cluster_indices.size(); i++) {
      pcl::IndicesPtr indices (new std::vector<int>(input_indices->cluster_indices[i].indices));
      extract.setIndices(indices);
      pcl::PointCloud<pcl::PointXYZHSV> segmented_cloud;
      extract.filter(segmented_cloud);
      segmented_clouds.push_back(segmented_cloud.makeShared());
      std::vector<float> histogram;
      computeHistogram(segmented_cloud, histogram, policy_);
      histograms[i] = histogram;
      ColorHistogram ros_histogram;
      ros_histogram.header = input_cloud->header;
      ros_histogram.histogram = histogram;
      histogram_array.histograms.push_back(ros_histogram);
    }
    all_histogram_pub_.publish(histogram_array);
    
    // compare histograms
    jsk_pcl_ros::ClusterPointIndices result;
    result.header = input_indices->header;
    double best_coefficient = - DBL_MAX;
    int best_index = -1;
    for (size_t i = 0; i < input_indices->cluster_indices.size(); i++) {
      const double coefficient = bhattacharyyaCoefficient(histograms[i], reference_histogram_);
      NODELET_DEBUG_STREAM("coefficient: " << i << "::" << coefficient);
      if (coefficient > coefficient_thr_) {
        result.cluster_indices.push_back(input_indices->cluster_indices[i]);
        if (best_coefficient < coefficient) {
          best_coefficient = coefficient;
          best_index = i;
        }
      }
    }
    NODELET_DEBUG("best coefficients: %f, %d", best_coefficient, best_index);
    result_pub_.publish(result);
    if (best_index != -1) {
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr best_cloud
        = segmented_clouds[best_index];
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*best_cloud, center);
      geometry_msgs::PoseStamped best_pose;
      best_pose.header = input_cloud->header;
      best_pose.pose.position.x = center[0];
      best_pose.pose.position.y = center[1];
      best_pose.pose.position.z = center[2];
      best_pose.pose.orientation.w = 1.0;
      best_pub_.publish(best_pose);
    }
  }

  double ColorHistogramMatcher::bhattacharyyaCoefficient(const std::vector<float>& a, const std::vector<float>& b)
  {
    if (a.size() != b.size()) {
      NODELET_ERROR("the bin size of histograms do not match");
      return 0.0;
    }
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
      sum += sqrt(a[i] * b[i]);
    }
    return sum;
  }
  
  void ColorHistogramMatcher::computeHistogram(
    const pcl::PointCloud<pcl::PointXYZHSV>& cloud,
    std::vector<float>& output,
    const ComparePolicy policy)
  {
    if (policy == USE_HUE_AND_SATURATION) {
      std::vector<float> hue, saturation;
      computeHistogram(cloud, hue, USE_HUE);
      computeHistogram(cloud, saturation, USE_SATURATION);
      
      output.resize(hue.size() + saturation.size());
      for (size_t i = 0; i < hue.size(); i++) {
        output[i] = hue[i];
      }
      for (size_t j = hue.size(); j < hue.size() + saturation.size(); j++) {
        output[j] = saturation[j - hue.size()];
      }
    }
    else {
      double val_max, val_min;
      if (policy == USE_HUE) {
        val_max = 360.0;
        val_min = 0.0;
      }
      else {
        val_max = 1.0;
        val_min = 0.0;
      }
      output.resize(bin_size_, 0);
      for (size_t i = 0; i < cloud.points.size(); i++) {
        pcl::PointXYZHSV output_point = cloud.points[i];
        // ratil
        double val;
        if (policy == USE_HUE) {
          val = output_point.h;
        }
        else if (policy == USE_SATURATION) {
          val = output_point.s;
        }
        else if (policy == USE_VALUE) {
          val = output_point.v;
        }
        int index = int((val - val_min) / (val_max - val_min) * bin_size_);
        if (index >= bin_size_) {
          index = bin_size_ - 1;
        }
        output[index] += 1.0;
      }
    }
    // normalize
    double sum = 0;
    for (size_t i = 0; i < output.size(); i++) {
      sum += output[i];
    }
    for (size_t i = 0; i < output.size(); i++) {
      if (sum != 0.0) {
        output[i] /= sum;
      }
      else {
        output[i] = 0.0;
      }
    }
  }
  
  void ColorHistogramMatcher::reference(
    const sensor_msgs::PointCloud2::ConstPtr& input_cloud)
  {
    boost::mutex::scoped_lock(mutex_);
    std::vector<float> hist;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*input_cloud, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZHSV> hsv_cloud;
    pcl::PointCloudXYZRGBtoXYZHSV(pcl_cloud, hsv_cloud);
    computeHistogram(hsv_cloud, hist, policy_);
    reference_histogram_ = hist;
    NODELET_INFO("update reference");
    reference_set_ = true;
    ColorHistogram ros_histogram;
    ros_histogram.header = input_cloud->header;
    ros_histogram.histogram = reference_histogram_;
    reference_histogram_pub_.publish(ros_histogram);
  }
  
  void ColorHistogramMatcher::referenceHistogram(
    const jsk_pcl_ros::ColorHistogram::ConstPtr& input_histogram)
  {
    boost::mutex::scoped_lock(mutex_);
    NODELET_INFO("update reference");
    reference_histogram_ = input_histogram->histogram;
    reference_histogram_pub_.publish(input_histogram);
    reference_set_ = true;
  }
  
}

typedef jsk_pcl_ros::ColorHistogramMatcher ColorHistogramMatcher;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ColorHistogramMatcher, ColorHistogramMatcher, nodelet::Nodelet);
