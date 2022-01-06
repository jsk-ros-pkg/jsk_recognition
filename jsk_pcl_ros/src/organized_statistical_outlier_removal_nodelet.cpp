// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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

#include "jsk_pcl_ros/organized_statistical_outlier_removal.h"
#include <jsk_topic_tools/diagnostic_utils.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <jsk_recognition_utils/pcl_util.h>

namespace jsk_pcl_ros
{
  OrganizedStatisticalOutlierRemoval::OrganizedStatisticalOutlierRemoval():
    DiagnosticNodelet("OrganizedStatisticalOutlierRemoval")
  {

  }

  void OrganizedStatisticalOutlierRemoval::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pnh_->param("use_cluster_point_indices", use_cpi_, false);
    pnh_->param("approximate_sync", use_async_, false);
    pnh_->param("queue_size", queue_size_, 100);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &OrganizedStatisticalOutlierRemoval::configCallback, this, _1, _2);
    srv_->setCallback (f);
    onInitPostProcess();
  }

  void OrganizedStatisticalOutlierRemoval::subscribe()
  {
    if (use_cpi_)
    {
      sub_cloud_.subscribe(*pnh_, "input", 1);
      sub_cpi_.subscribe(*pnh_, "input/cluster_indices", 1);
      if (use_async_)
      {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
        async_->connectInput(sub_cloud_, sub_cpi_);
        async_->registerCallback(
          boost::bind(
            &OrganizedStatisticalOutlierRemoval::filterCloudWithClusterPointIndices, this, _1, _2));
      }
      else
      {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_cloud_, sub_cpi_);
        sync_->registerCallback(
          boost::bind(
            &OrganizedStatisticalOutlierRemoval::filterCloudWithClusterPointIndices, this, _1, _2));
      }
    }
    else
    {
      sub_ = pnh_->subscribe("input", 1, &OrganizedStatisticalOutlierRemoval::filterCloud, this);
    }
  }

  void OrganizedStatisticalOutlierRemoval::unsubscribe()
  {
    sub_.shutdown();
  }

  void OrganizedStatisticalOutlierRemoval::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    keep_organized_ = config.keep_organized;
    negative_ = config.negative;
    mean_k_ = config.mean_k;
    std_mul_ = config.stddev;
  }

  void OrganizedStatisticalOutlierRemoval::filter(
          pcl::PointCloud<PointT>::Ptr cloud,
          pcl::PointCloud<PointT>::Ptr cloud_filtered,
          bool keep_organized)
  {
#if PCL_VERSION_COMPARE (<, 1, 9, 0)
    if (keep_organized) {
      // Initialize the spatial locator
      pcl::search::Search<PointT>::Ptr tree;
      tree.reset (new pcl::search::OrganizedNeighbor<PointT> ());
      tree->setInputCloud (cloud);

      // Allocate enough space to hold the results
      std::vector<int> indices (cloud->size ());
      std::vector<int> nn_indices (mean_k_);
      std::vector<float> nn_dists (mean_k_);
      std::vector<float> distances (indices.size ());
      int valid_distances = 0;

      for (size_t i = 0; i < indices.size (); ++i) indices[i] = i;

      // Go over all the points and calculate the mean or smallest distance
      for (size_t cp = 0; cp < indices.size (); ++cp)
      {
        if (!pcl_isfinite (cloud->points[indices[cp]].x) ||
            !pcl_isfinite (cloud->points[indices[cp]].y) ||
            !pcl_isfinite (cloud->points[indices[cp]].z))
        {
          distances[cp] = 0;
          continue;
        }
        if (tree->nearestKSearch (indices[cp], mean_k_, nn_indices, nn_dists) == 0)
        {
          distances[cp] = 0;
          continue;
        }
        // Minimum distance (if mean_k_ == 2) or mean distance
        double dist_sum = 0;
        for (int j = 1; j < mean_k_; ++j)
          dist_sum += sqrt (nn_dists[j]);
        distances[cp] = static_cast<float> (dist_sum / (mean_k_ - 1));
        valid_distances++;
      }

      // Estimate the mean and the standard deviation of the distance vector
      double sum = 0, sq_sum = 0;
      for (size_t i = 0; i < distances.size (); ++i)
      {
        sum += distances[i];
        sq_sum += distances[i] * distances[i];
      }
      double mean = sum / static_cast<double>(valid_distances);
      double variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
      double stddev = sqrt (variance);
      //getMeanStd (distances, mean, stddev);

      double distance_threshold = mean + std_mul_ * stddev; // a distance that is bigger than this signals an outlier

      pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2);
      pcl::PCLPointCloud2::Ptr pcl_cloud_filtered (new pcl::PCLPointCloud2);
      pcl::toPCLPointCloud2(*cloud, *pcl_cloud);
      pcl_cloud_filtered->is_bigendian = pcl_cloud->is_bigendian;
      pcl_cloud_filtered->fields = pcl_cloud->fields;
      pcl_cloud_filtered->point_step = pcl_cloud->point_step;
      pcl_cloud_filtered->data.resize (pcl_cloud->data.size ());
      pcl_cloud_filtered->width = pcl_cloud->width;
      pcl_cloud_filtered->height = pcl_cloud->height;
      pcl_cloud_filtered->row_step = pcl_cloud_filtered->point_step *  pcl_cloud_filtered->width;

      // Build a new cloud by neglecting outliers
      int nr_p = 0;
      for (int cp = 0; cp < static_cast<int> (indices.size ()); ++cp)
      {
        bool remove_point = false;
        if (negative_)
          remove_point = (distances[cp] <= distance_threshold);
        else
          remove_point = (distances[cp] > distance_threshold);
        if (remove_point)
        {
          /* Set the current point to NaN. */
          *(reinterpret_cast<float*>(&pcl_cloud_filtered->data[nr_p * pcl_cloud_filtered->point_step])+0) = std::numeric_limits<float>::quiet_NaN();
          *(reinterpret_cast<float*>(&pcl_cloud_filtered->data[nr_p * pcl_cloud_filtered->point_step])+1) = std::numeric_limits<float>::quiet_NaN();
          *(reinterpret_cast<float*>(&pcl_cloud_filtered->data[nr_p * pcl_cloud_filtered->point_step])+2) = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
          memcpy (&pcl_cloud_filtered->data[nr_p * pcl_cloud_filtered->point_step],
                  &pcl_cloud->data[indices[cp] * pcl_cloud_filtered->point_step],
                  pcl_cloud_filtered->point_step);
        }
        nr_p++;
      }
      pcl::fromPCLPointCloud2(*pcl_cloud_filtered, *cloud_filtered);
    }
    else
    {
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK (mean_k_);
      sor.setStddevMulThresh (std_mul_);
      sor.setNegative (negative_);
      sor.filter (*cloud_filtered);
    }
#else
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK (mean_k_);
    sor.setStddevMulThresh (std_mul_);
    sor.setNegative (negative_);
    sor.setKeepOrganized (keep_organized);
    sor.filter (*cloud_filtered);
#endif
  }

  void OrganizedStatisticalOutlierRemoval::filterCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    sensor_msgs::PointCloud2 output;

    if (keep_organized_&& msg->is_dense) {
      NODELET_ERROR("keep_organized parameter is true, but input pointcloud is not organized.");
    }
    bool keep_organized = keep_organized_ && !msg->is_dense;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    // filter
    OrganizedStatisticalOutlierRemoval::filter(cloud, cloud_filtered, keep_organized);

    pcl::toROSMsg(*cloud_filtered, output);
#if PCL_VERSION_COMPARE (<, 1, 9, 0)
    if (keep_organized) {
      // Copy the common fields
      output.is_bigendian = msg->is_bigendian;
      output.fields = msg->fields;
      output.point_step = msg->point_step;
      output.data.resize (msg->data.size ());
      output.width = msg->width;
      output.height = msg->height;
      output.row_step = msg->point_step * msg->width;
    }
#endif
    output.header = msg->header;
    output.is_dense = !keep_organized;
    pub_.publish(output);
    diagnostic_updater_->update();
  }

  void OrganizedStatisticalOutlierRemoval::filterCloudWithClusterPointIndices(
          const sensor_msgs::PointCloud2::ConstPtr& msg,
          const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cpi_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    sensor_msgs::PointCloud2 output;

    if (keep_organized_&& msg->is_dense) {
      NODELET_ERROR("keep_organized parameter is true, but input pointcloud is not organized.");
    }
    bool keep_organized = keep_organized_ && !msg->is_dense;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    cloud_filtered->points.resize(cloud->points.size());
    PointT nan_point;
    nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<float>::quiet_NaN();
    std::fill(cloud_filtered->points.begin(), cloud_filtered->points.end(), nan_point);

    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(cloud);
    ex.setKeepOrganized(keep_organized);
    ex.setNegative(false);

    for (size_t i = 0; i < cpi_msg->cluster_indices.size(); i++)
    {
      pcl::IndicesPtr indices;
      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
      pcl::PointCloud<PointT>::Ptr cluster_cloud_filtered(new pcl::PointCloud<PointT>);
      indices.reset (new std::vector<int> (cpi_msg->cluster_indices[i].indices));
      ex.setIndices(indices);
      ex.filter(*cluster_cloud);
      OrganizedStatisticalOutlierRemoval::filter(cluster_cloud, cluster_cloud_filtered, keep_organized);
      for (size_t j = 0; j < indices->size(); j++)
      {
        int ind = (*indices)[j];
        if (!std::isnan(cluster_cloud_filtered->points[ind].x) &&
            !std::isnan(cluster_cloud_filtered->points[ind].y) &&
            !std::isnan(cluster_cloud_filtered->points[ind].z))
        {
          cloud_filtered->points[ind].x = cluster_cloud_filtered->points[ind].x;
          cloud_filtered->points[ind].y = cluster_cloud_filtered->points[ind].y;
          cloud_filtered->points[ind].z = cluster_cloud_filtered->points[ind].z;
          cloud_filtered->points[ind].rgb = cluster_cloud_filtered->points[ind].rgb;
        }
      }

    }

    pcl::toROSMsg(*cloud_filtered, output);
#if PCL_VERSION_COMPARE (<, 1, 9, 0)
    if (keep_organized) {
      // Copy the common fields
      output.is_bigendian = msg->is_bigendian;
      output.fields = msg->fields;
      output.point_step = msg->point_step;
      output.data.resize (msg->data.size ());
      output.width = msg->width;
      output.height = msg->height;
      output.row_step = output.point_step * output.width;
    }
#endif
    output.header = msg->header;
    output.is_dense = !keep_organized;
    pub_.publish(output);
    diagnostic_updater_->update();
  }

  void OrganizedStatisticalOutlierRemoval::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   name_ + " running");
      jsk_topic_tools::addDiagnosticBooleanStat("keep organized",
                                                keep_organized_,
                                                stat);
      jsk_topic_tools::addDiagnosticBooleanStat("negative",
                                                negative_,
                                                stat);
      stat.add("mean k", mean_k_);
      stat.add("stddev", std_mul_);
    }
    DiagnosticNodelet::updateDiagnostic(stat);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OrganizedStatisticalOutlierRemoval, nodelet::Nodelet);
