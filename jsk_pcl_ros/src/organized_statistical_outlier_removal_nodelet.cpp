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
#include <pcl_conversions/pcl_conversions.h>
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
    if (use_cpi_)
    {
      sub_cloud_.unsubscribe();
      sub_cpi_.unsubscribe();
    }
    else
    {
      sub_.shutdown();
    }
  }

  void OrganizedStatisticalOutlierRemoval::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    keep_organized_ = config.keep_organized;
    negative_ = config.negative;
    mean_k_ = config.mean_k;
    std_mul_ = config.stddev;
  }

    // pcl removed the method by 1.13, no harm in defining it ourselves to use below
#if __cplusplus >= 201103L
#define pcl_isfinite(x) std::isfinite(x)
#endif

  void OrganizedStatisticalOutlierRemoval::filter(
          const pcl::PCLPointCloud2::Ptr pcl_cloud,
          pcl::PointIndices::Ptr pcl_indices_filtered)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_cloud, *cloud);
    pcl_indices_filtered->indices = OrganizedStatisticalOutlierRemoval::getFilteredIndices(cloud);
  }

  void OrganizedStatisticalOutlierRemoval::filter(
          const pcl::PCLPointCloud2::Ptr pcl_cloud,
          const pcl::PointIndices::Ptr pcl_indices,
          pcl::PointIndices::Ptr pcl_indices_filtered)
  {
    pcl::PCLPointCloud2::Ptr pcl_cloud_ex (new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
    ex.setInputCloud(pcl_cloud);
    ex.setKeepOrganized(false);
    ex.setNegative(false);
    ex.setIndices(pcl_indices);
    ex.filter(*pcl_cloud_ex);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ex(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_cloud_ex, *cloud_ex);

    const std::vector<int> indices_ex_filtered = OrganizedStatisticalOutlierRemoval::getFilteredIndices(cloud_ex);
    std::vector<int> indices_filtered;
    indices_filtered.resize(indices_ex_filtered.size());
    for (size_t i = 0; i < indices_ex_filtered.size(); i++)
    {
      indices_filtered[i] = pcl_indices->indices[indices_ex_filtered[i]];
    }
    pcl_indices_filtered->indices = indices_filtered;
  }

  std::vector<int> OrganizedStatisticalOutlierRemoval::getFilteredIndices(
          const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    std::vector<int> indices_filtered;
    indices_filtered.resize(cloud->size());
#if PCL_VERSION_COMPARE (<, 1, 9, 0)
    // Initialize the spatial locator
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
    tree->setInputCloud (cloud);

    // Allocate enough space to hold the results
    std::vector<int> indices (cloud->size());
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

    // Build a new cloud by neglecting outliers
    for (int cp = 0; cp < static_cast<int> (indices.size ()); ++cp)
    {
      bool remove_point = false;
      if (negative_)
      {
        remove_point = (distances[cp] <= distance_threshold);
      }
      else
      {
        remove_point = (distances[cp] > distance_threshold);
      }
      if (!remove_point)
      {
        indices_filtered.push_back(indices[cp]);
      }
    }
#else
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_mul_);
    sor.setNegative(negative_);
    sor.setKeepOrganized (true);
    sor.filter(indices_filtered);
#endif
    return indices_filtered;
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
    pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg, *pcl_cloud);

    // filter
    pcl::PointIndices::Ptr pcl_indices_filtered (new pcl::PointIndices());
    OrganizedStatisticalOutlierRemoval::filter(pcl_cloud, pcl_indices_filtered);
    pcl::PCLPointCloud2::Ptr pcl_cloud_filtered (new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
    ex.setInputCloud(pcl_cloud);
    ex.setKeepOrganized(keep_organized);
    ex.setNegative(false);
    ex.setIndices(pcl_indices_filtered);
    ex.filter(*pcl_cloud_filtered);
    pcl_conversions::fromPCL(*pcl_cloud_filtered, output);
#if PCL_VERSION_COMPARE (<, 1, 9, 0)
    if (keep_organized) {
      // Copy the common fields
      output.is_bigendian = msg->is_bigendian;
      output.fields = msg->fields;
      output.point_step = msg->point_step;
      output.data.resize(msg->data.size());
      output.width = msg->width;
      output.height = msg->height;
    }
#endif
    output.header = msg->header;
    output.row_step = output.point_step * output.width;
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

    if (keep_organized_ && msg->is_dense) {
      NODELET_ERROR("keep_organized parameter is true, but input pointcloud is not organized.");
    }
    bool keep_organized = keep_organized_ && !msg->is_dense;
    pcl::PCLPointCloud2::Ptr pcl_cloud (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr pcl_cloud_filtered (new pcl::PCLPointCloud2);
    pcl::PointIndices::Ptr pcl_indices_filtered (new pcl::PointIndices());
    pcl_conversions::toPCL(*msg, *pcl_cloud);

    for (size_t i = 0; i < cpi_msg->cluster_indices.size(); i++)
    {
      pcl::PointIndices::Ptr pcl_cluster_indices (new pcl::PointIndices());
      pcl_conversions::toPCL(cpi_msg->cluster_indices[i], *pcl_cluster_indices);

      pcl::PointIndices::Ptr pcl_cluster_indices_filtered (new pcl::PointIndices());
      OrganizedStatisticalOutlierRemoval::filter(pcl_cloud,
                                                 pcl_cluster_indices,
                                                 pcl_cluster_indices_filtered);
      pcl_indices_filtered->indices.insert(pcl_indices_filtered->indices.end(),
                                           pcl_cluster_indices_filtered->indices.begin(),
                                           pcl_cluster_indices_filtered->indices.end());
    }
    pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
    ex.setInputCloud(pcl_cloud);
    ex.setKeepOrganized(keep_organized);
    ex.setNegative(false);
    ex.setIndices(pcl_indices_filtered);
    ex.filter(*pcl_cloud_filtered);
    pcl_conversions::fromPCL(*pcl_cloud_filtered, output);
#if PCL_VERSION_COMPARE (<, 1, 9, 0)
    if (keep_organized) {
      // Copy the common fields
      output.is_bigendian = msg->is_bigendian;
      output.fields = msg->fields;
      output.point_step = msg->point_step;
      output.data.resize(msg->data.size());
      output.width = msg->width;
      output.height = msg->height;
    }
#endif
    output.header = msg->header;
    output.row_step = output.point_step * output.width;
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
