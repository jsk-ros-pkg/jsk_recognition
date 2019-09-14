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

#include "jsk_pcl_ros/color_histogram_matcher.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE (<, 1, 7, 2)
#include <jsk_pcl_ros/pcl/point_types_conversion.h>
#else
#include <pcl/point_types_conversion.h>
#endif

namespace jsk_pcl_ros
{
  void ColorHistogramMatcher::onInit()
  {
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ColorHistogramMatcher::configCallback, this, _1, _2);
    srv_->setCallback (f);

    policy_ = USE_HUE_AND_SATURATION;
    reference_set_ = false;
    // setup publishers
    all_histogram_pub_
      = advertise<jsk_recognition_msgs::ColorHistogramArray>(
        *pnh_, "output_histograms", 1);
    best_pub_
      = advertise<geometry_msgs::PoseStamped>(*pnh_, "best_match", 1);
    reference_histogram_pub_
      = advertise<jsk_recognition_msgs::ColorHistogram>(*pnh_, "output_reference", 1);
    result_pub_
      = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    coefficient_points_pub_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "coefficient_points", 1);
    onInitPostProcess();
 }

  void ColorHistogramMatcher::subscribe()
  {
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

  void ColorHistogramMatcher::unsubscribe()
  {
    reference_sub_.shutdown();
    reference_histogram_sub_.shutdown();
    sub_input_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void ColorHistogramMatcher::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    coefficient_thr_ = config.coefficient_thr;
    bin_size_ = config.bin_size;
    publish_colored_cloud_ = config.publish_colored_cloud;
    power_ = config.power;
    color_min_coefficient_ = config.color_min_coefficient;
    color_max_coefficient_ = config.color_max_coefficient;
    show_method_ = config.show_method;
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
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices)
  {
    boost::mutex::scoped_lock lock(mutex_);
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
    unsigned long point_all_size=0;
    pcl::ExtractIndices<pcl::PointXYZHSV> extract;
    extract.setInputCloud(hsv_cloud);
    // for debug
    jsk_recognition_msgs::ColorHistogramArray histogram_array;
    histogram_array.header = input_cloud->header;
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr > segmented_clouds;
    for (size_t i = 0; i < input_indices->cluster_indices.size(); i++) {
      pcl::IndicesPtr indices (new std::vector<int>(input_indices->cluster_indices[i].indices));
      point_all_size+=indices->size();
      extract.setIndices(indices);
      pcl::PointCloud<pcl::PointXYZHSV> segmented_cloud;
      extract.filter(segmented_cloud);
      segmented_clouds.push_back(segmented_cloud.makeShared());
      std::vector<float> histogram;
      computeHistogram(segmented_cloud, histogram, policy_);
      histograms[i] = histogram;
      jsk_recognition_msgs::ColorHistogram ros_histogram;
      ros_histogram.header = input_cloud->header;
      ros_histogram.histogram = histogram;
      histogram_array.histograms.push_back(ros_histogram);
    }
    all_histogram_pub_.publish(histogram_array);

    // compare histograms
    jsk_recognition_msgs::ClusterPointIndices result;
    result.header = input_indices->header;
    double best_coefficient = - DBL_MAX;
    int best_index = -1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(publish_colored_cloud_){
      output_cloud->width=point_all_size;
      output_cloud->height=1; 
      output_cloud->resize(point_all_size);
    }
    unsigned long count_points=0;
    for (size_t i = 0; i < input_indices->cluster_indices.size(); i++) {
      const double coefficient = bhattacharyyaCoefficient(histograms[i], reference_histogram_);
      NODELET_DEBUG_STREAM("coefficient: " << i << "::" << coefficient);
      if(publish_colored_cloud_){
	int tmp_point_size = input_indices->cluster_indices[i].indices.size();
	double color_standard;
	if(color_min_coefficient_ > coefficient){
	  color_standard = 0;
	} else if(color_max_coefficient_ < coefficient){
	  color_standard = 1;
	} else{
	  color_standard = (coefficient - color_min_coefficient_) / (color_max_coefficient_ - color_min_coefficient_);
	}
	double color_standard_powered = 1;
	for (int k=0; k<power_; k++){
	  color_standard_powered *= color_standard;
	}
	unsigned char color_r, color_g, color_b;
	switch(show_method_){
	case 0:
	  color_r=(int)(255*color_standard_powered);
	  color_g=0;
	  color_b=(int)(255*(1-color_standard_powered));
	  break;
	case 1:
	  // like thermo
	  int color_index = (int)(color_standard_powered*1280);
	  switch(color_index/256){
	  case 0:
	    color_r=0; color_g=0; color_b=color_index;
	    break;
	  case 1:
	  color_r=color_index-256; color_g=0; color_b=255;
	  break;
	  case 2:
	    color_r=255; color_g=0; color_b=255-(color_index-256*2);
	    break;
	  case 3:
	    color_r=255; color_g=color_index-256*3; color_b=0;
	    break;
	  case 4:
	    color_r=255; color_g=255; color_b=color_index-256*4;
	    break;
	  case 5:
	    color_r=255; color_g=255; color_b=255;
	    break;
	  }
	  break;
	}
	for(int j=0; j<tmp_point_size; j++){
	  output_cloud->points[j+count_points].x=segmented_clouds[i]->points[j].x;
	  output_cloud->points[j+count_points].y=segmented_clouds[i]->points[j].y;
	  output_cloud->points[j+count_points].z=segmented_clouds[i]->points[j].z;
	  output_cloud->points[j+count_points].r=color_r;
	  output_cloud->points[j+count_points].g=color_g;
	  output_cloud->points[j+count_points].b=color_b;
	}
	count_points+=tmp_point_size;
      }
      if (coefficient > coefficient_thr_) {
        result.cluster_indices.push_back(input_indices->cluster_indices[i]);
	if (best_coefficient < coefficient) {
          best_coefficient = coefficient;
          best_index = i;
        }
      }
    }
    NODELET_DEBUG("best coefficients: %f, %d", best_coefficient, best_index);
    //show coefficience with points
    sensor_msgs::PointCloud2 p_msg;
    if(publish_colored_cloud_){
      pcl::toROSMsg(*output_cloud, p_msg);
      p_msg.header=input_cloud->header;
      coefficient_points_pub_.publish(p_msg);
    }
    result_pub_.publish(result);
    if (best_index != -1) {
      pcl::PointCloud<pcl::PointXYZHSV>::Ptr best_cloud
        = segmented_clouds[best_index];

      best_cloud->is_dense = false;
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
    boost::mutex::scoped_lock lock(mutex_);
    std::vector<float> hist;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*input_cloud, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZHSV> hsv_cloud;
    pcl::PointCloudXYZRGBtoXYZHSV(pcl_cloud, hsv_cloud);
    computeHistogram(hsv_cloud, hist, policy_);
    reference_histogram_ = hist;
    NODELET_INFO("update reference");
    reference_set_ = true;
    jsk_recognition_msgs::ColorHistogram ros_histogram;
    ros_histogram.header = input_cloud->header;
    ros_histogram.histogram = reference_histogram_;
    reference_histogram_pub_.publish(ros_histogram);
  }
  
  void ColorHistogramMatcher::referenceHistogram(
    const jsk_recognition_msgs::ColorHistogram::ConstPtr& input_histogram)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_INFO("update reference");
    reference_histogram_ = input_histogram->histogram;
    reference_histogram_pub_.publish(input_histogram);
    reference_set_ = true;
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ColorHistogramMatcher,
                        nodelet::Nodelet);
