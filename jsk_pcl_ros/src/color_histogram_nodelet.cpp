// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
/*
 * color_histogram_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_pcl_ros/color_histogram.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE (<, 1, 7, 2)
#include <jsk_pcl_ros/pcl/point_types_conversion.h>
#else
#include <pcl/point_types_conversion.h>
#endif

namespace jsk_pcl_ros
{
  void ColorHistogram::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ColorHistogram::configCallback, this, _1, _2);
    srv_->setCallback(f);
    pub_histogram_ = advertise<jsk_recognition_msgs::ColorHistogramArray>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ColorHistogram::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bin_size_ = config.bin_size;
    histogram_policy_ = jsk_recognition_utils::HistogramPolicy(config.histogram_policy);
    white_threshold_ = config.white_threshold;
    black_threshold_ = config.black_threshold;
    if (queue_size_ != config.queue_size) {
      queue_size_ = config.queue_size;
      if (isSubscribed()) {
        unsubscribe();
        subscribe();
      }
    }
  }

  void ColorHistogram::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
    sync_->connectInput(sub_cloud_, sub_indices_);
    sync_->registerCallback(boost::bind(&ColorHistogram::feature,
                                        this, _1, _2));
  }

  void ColorHistogram::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void ColorHistogram::feature(
    const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices)
  {
    boost::mutex::scoped_lock lock(mutex_);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input_cloud, *rgb_cloud);

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*rgb_cloud, *hsv_cloud);

    for (size_t i = 0; i < rgb_cloud->points.size(); i++) {
      hsv_cloud->points[i].x = rgb_cloud->points[i].x;
      hsv_cloud->points[i].y = rgb_cloud->points[i].y;
      hsv_cloud->points[i].z = rgb_cloud->points[i].z;
    }

    pcl::ExtractIndices<pcl::PointXYZHSV> extract;
    extract.setInputCloud(hsv_cloud);

    jsk_recognition_msgs::ColorHistogramArray histogram_array;
    histogram_array.histograms.resize(input_indices->cluster_indices.size());
    histogram_array.header = input_cloud->header;
    for (size_t i = 0; i < input_indices->cluster_indices.size(); ++i) {
      pcl::IndicesPtr indices(new std::vector<int>(input_indices->cluster_indices[i].indices));
      extract.setIndices(indices);
      pcl::PointCloud<pcl::PointXYZHSV> segmented_cloud;
      extract.filter(segmented_cloud);
      histogram_array.histograms[i].header = input_cloud->header;
      if (histogram_policy_ == jsk_recognition_utils::HUE) {
        jsk_recognition_utils::computeColorHistogram1d(segmented_cloud,
                                                       histogram_array.histograms[i].histogram,
                                                       bin_size_,
                                                       white_threshold_,
                                                       black_threshold_);
      } else if (histogram_policy_ == jsk_recognition_utils::HUE_AND_SATURATION) {
        jsk_recognition_utils::computeColorHistogram2d(segmented_cloud,
                                                       histogram_array.histograms[i].histogram,
                                                       bin_size_,
                                                       white_threshold_,
                                                       black_threshold_);
      } else {
        NODELET_FATAL("Invalid histogram policy");
        return;
      }
    }
    pub_histogram_.publish(histogram_array);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ColorHistogram, nodelet::Nodelet);
