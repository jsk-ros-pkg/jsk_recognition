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

#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/euclidean_cluster_extraction_nodelet.h" 
#include <jsk_recognition_utils/pcl_util.h>

using namespace std;
using namespace pcl;

namespace jsk_pcl_ros
{

  void EuclideanClustering::downsample_cloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& sampled_cloud,
    std::vector<std::vector<int> >& sampled_to_original_indices,
    std::vector<int>& original_to_sampled_indices,
    double leaf_size)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setSaveLeafLayout(true);
    voxel.setInputCloud(original_cloud);
    voxel.filter(*sampled_cloud);
    sampled_to_original_indices.resize(original_cloud->points.size());
    original_to_sampled_indices.resize(original_cloud->points.size());
    std::fill(original_to_sampled_indices.begin(),
              original_to_sampled_indices.end(),
              -1);
    std::fill(sampled_to_original_indices.begin(),
              sampled_to_original_indices.end(),
              std::vector<int>());
    for (size_t i = 0; i < original_cloud->points.size(); ++i) {
      pcl::PointXYZ p = original_cloud->points[i];
      if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
        continue;
      }
      int index = voxel.getCentroidIndex(p);
      if (index == -1) {
        continue;
      }
      original_to_sampled_indices[i] = index;
      sampled_to_original_indices[index].push_back(i);
    }
  }

  void EuclideanClustering::clusteringClusterIndices(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    std::vector<pcl::PointIndices::Ptr> &cluster_indices,
    std::vector<pcl::PointIndices> &clustered_indices) {

    boost::mutex::scoped_lock lock(mutex_);

    clustered_indices.clear();


    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    if (downsample_) {
      downsample_cloud(cloud,
                       preprocessed_cloud,
                       downsample_to_original_indices_,
                       original_to_downsample_indices_,
                       leaf_size_);

    } else {
      preprocessed_cloud = cloud;
    }

    if (preprocessed_cloud->points.size() == 0) {
      return;
    }

    for(pcl::PointIndices::Ptr point_indices : cluster_indices){
      pcl::PointIndices::Ptr nonnan_indices (new pcl::PointIndices);
      for(int original_index : point_indices->indices) {
        int index;
        if (downsample_) {
          index = original_to_downsample_indices_[original_index];
        } else {
          index = original_index;
        }
        if (index == -1) {
          continue;
        }

        pcl::PointXYZ p = preprocessed_cloud->points[index];
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
          nonnan_indices->indices.push_back(index);
        }
      }
      removeDuplicatedIndices(nonnan_indices);

      pcl::PointIndices result_point_indices;
      if (nonnan_indices->indices.size() > 0) {
        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        {
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud(preprocessed_cloud);
          extract.setIndices (nonnan_indices);
          extract.setNegative(false);
          extract.filter (*filtered_cloud);
        }

        EuclideanClusterExtraction<pcl::PointXYZ> impl;
        if (filtered_cloud->points.size() > 0) {
          jsk_topic_tools::ScopedTimer timer = kdtree_acc_.scopedTimer();
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
          tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
          pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
          tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
          tree->setInputCloud(filtered_cloud);
          impl.setClusterTolerance(tolerance);
          impl.setMinClusterSize(minsize_);
          impl.setMaxClusterSize(maxsize_);
          impl.setSearchMethod(tree);
          impl.setInputCloud(filtered_cloud);
        }

        std::vector<pcl::PointIndices> output_indices;
        {
          jsk_topic_tools::ScopedTimer timer = segmentation_acc_.scopedTimer();
          impl.extract(output_indices);
          if (downsample_) {
            for (size_t i = 0; i < output_indices.size(); ++i) {
              pcl::PointIndices::Ptr tmp_indices(new pcl::PointIndices);
              for (size_t j = 0; j < output_indices.at(i).indices.size(); ++j) {
                tmp_indices->indices.insert(
                  tmp_indices->indices.end(),
                  downsample_to_original_indices_[nonnan_indices->indices[output_indices.at(i).indices[j]]].begin(),
                  downsample_to_original_indices_[nonnan_indices->indices[output_indices.at(i).indices[j]]].end());
              }
              removeDuplicatedIndices(tmp_indices);
              output_indices[i] = *tmp_indices;
            }
          } else {
            for (size_t i = 0; i < output_indices.size(); ++i) {
              pcl::PointIndices tmp_indices;
              tmp_indices.indices.resize( output_indices.at(i).indices.size());
              for (size_t j = 0; j < output_indices.at(i).indices.size(); ++j) {
                tmp_indices.indices[j] = nonnan_indices->indices[output_indices.at(i).indices[j]];
              }
              output_indices[i] = tmp_indices;
            }
          }
        }
        std::vector<int> result_indices;
        std::vector<int> examine_indices;
        switch (cluster_filter_type_) {
          case jsk_pcl_ros::EuclideanClustering_All:  {
            for (size_t i_e = 0; i_e < output_indices.size(); ++i_e) {
              examine_indices.push_back(i_e);
            }
            break;
          }
          case jsk_pcl_ros::EuclideanClustering_MaxSize: {
            // take maximum size of cluster
            int size = 0;
            int index = 0;
            for(size_t i=0; i < output_indices.size(); i++){
              if(output_indices[i].indices.size() > size){
                size = output_indices[i].indices.size();
                index = i;
              }
            }
            examine_indices.push_back(index);
            break;
          }
        }
        for (int index : examine_indices) {
          result_point_indices.indices = output_indices.at(index).indices;
          clustered_indices.push_back(result_point_indices);
        }
      } else {
        clustered_indices.push_back(result_point_indices);
      }
    }
  }

  void EuclideanClustering::extract(
    const sensor_msgs::PointCloud2ConstPtr &input)
  {
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::PointIndices::Ptr whole_indices(new pcl::PointIndices);
    whole_indices->indices.resize(input->height * input->width);
    for (size_t i = 0; i < input->height * input->width; ++i) {
      whole_indices->indices[i] = i;
    }
    std::vector<pcl::PointIndices::Ptr> cluster_indices{whole_indices};
    std::vector<pcl::PointIndices> clustered_indices;
    clusteringClusterIndices(cloud,
                             cluster_indices,
                             clustered_indices);

    // Publish result indices
    jsk_recognition_msgs::ClusterPointIndices result;
    result.cluster_indices.resize(clustered_indices.size());
    cluster_counter_.add(clustered_indices.size());
    result.header = input->header;
    if (cogs_.size() != 0 && cogs_.size() == clustered_indices.size()) {
      // tracking the labels
      //ROS_INFO("computing distance matrix");
      // compute distance matrix
      // D[i][j] --> distance between the i-th previous cluster
      //             and the current j-th cluster
      Vector4fVector new_cogs;
      computeCentroidsOfClusters(new_cogs, cloud, clustered_indices);
      double D[cogs_.size() * new_cogs.size()];
      computeDistanceMatrix(D, cogs_, new_cogs);
      std::vector<int> pivot_table = buildLabelTrackingPivotTable(D, cogs_, new_cogs, label_tracking_tolerance);
      if (pivot_table.size() != 0) {
        clustered_indices = pivotClusterIndices(pivot_table, clustered_indices);
      }
    }
    Vector4fVector tmp_cogs;
    computeCentroidsOfClusters(tmp_cogs, cloud, clustered_indices); // NB: not efficient
    cogs_ = tmp_cogs;
      
    for (size_t i = 0; i < clustered_indices.size(); i++) {
#if ROS_VERSION_MINIMUM(1, 10, 0)
      // hydro and later
      result.cluster_indices[i].header
        = pcl_conversions::fromPCL(clustered_indices[i].header);
#else
      // groovy
      result.cluster_indices[i].header = clustered_indices[i].header;
#endif
      result.cluster_indices[i].indices = clustered_indices[i].indices;
    }

    result_pub_.publish(result);
    
    jsk_recognition_msgs::Int32Stamped::Ptr cluster_num_msg (new jsk_recognition_msgs::Int32Stamped);
    cluster_num_msg->header = input->header;
    cluster_num_msg->data = clustered_indices.size();
    cluster_num_pub_.publish(cluster_num_msg);

    diagnostic_updater_->update();
  }

  void EuclideanClustering::multi_extract(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_cluster_indices,
    const sensor_msgs::PointCloud2::ConstPtr& input) {
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);

    std::vector<pcl::PointIndices::Ptr> cluster_indices =
      pcl_conversions::convertToPCLPointIndices(input_cluster_indices->cluster_indices);
    std::vector<pcl::PointIndices> clustered_indices;
    clusteringClusterIndices(cloud,
                             cluster_indices,
                             clustered_indices);

    // Publish result indices
    jsk_recognition_msgs::ClusterPointIndices result;
    result.cluster_indices.resize(clustered_indices.size());
    cluster_counter_.add(clustered_indices.size());
    result.header = input->header;

    for (size_t i = 0; i < clustered_indices.size(); i++) {
#if ROS_VERSION_MINIMUM(1, 10, 0)
      // hydro and later
      result.cluster_indices[i].header
        = pcl_conversions::fromPCL(clustered_indices[i].header);
#else
      // groovy
      result.cluster_indices[i].header = clustered_indices[i].header;
#endif
      result.cluster_indices[i].indices = clustered_indices[i].indices;
    }

    result_pub_.publish(result);

    jsk_recognition_msgs::Int32Stamped::Ptr cluster_num_msg (new jsk_recognition_msgs::Int32Stamped);
    cluster_num_msg->header = input->header;
    cluster_num_msg->data = clustered_indices.size();
    cluster_num_pub_.publish(cluster_num_msg);

    diagnostic_updater_->update();
  }

  bool EuclideanClustering::serviceCallback(
    jsk_recognition_msgs::EuclideanSegment::Request &req,
    jsk_recognition_msgs::EuclideanSegment::Response &res)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.input, *cloud);

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
    pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif

    vector<pcl::PointIndices> cluster_indices;
    EuclideanClusterExtraction<pcl::PointXYZ> impl;
    double tor;
    if ( req.tolerance < 0) {
      tor = tolerance;
    } else {
      tor = req.tolerance;
    }
    impl.setClusterTolerance (tor);
    impl.setMinClusterSize (minsize_);
    impl.setMaxClusterSize (maxsize_);
    impl.setSearchMethod (tree);
    impl.setInputCloud (cloud);
    impl.extract (cluster_indices);

    res.output.resize( cluster_indices.size() );
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 7 )
    pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(req.input, *pcl_cloud);
    pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
    ex.setInputCloud(pcl_cloud);
#else
    pcl::ExtractIndices<sensor_msgs::PointCloud2> ex;
    ex.setInputCloud ( boost::make_shared< sensor_msgs::PointCloud2 > (req.input) );
#endif
    for ( size_t i = 0; i < cluster_indices.size(); i++ ) {
      ex.setIndices ( boost::make_shared< pcl::PointIndices > (cluster_indices[i]) );
      ex.setNegative ( false );
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 7 )
      pcl::PCLPointCloud2 output_cloud;
      ex.filter ( output_cloud );
      pcl_conversions::fromPCL(output_cloud, res.output[i]);
#else
      ex.filter ( res.output[i] );
#endif
    }
    return true;
  }

  void EuclideanClustering::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("multi", multi_, false);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 20);

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EuclideanClustering::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    ////////////////////////////////////////////////////////
    // Publisher
    ////////////////////////////////////////////////////////
    result_pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices> (*pnh_, "output", 1);
    cluster_num_pub_ = advertise<jsk_recognition_msgs::Int32Stamped> (*pnh_, "cluster_num", 1);
    service_ = pnh_->advertiseService(pnh_->resolveName("euclidean_clustering"),
                                      &EuclideanClustering::serviceCallback, this);

    onInitPostProcess();
  }

  void EuclideanClustering::subscribe()
  {
    ////////////////////////////////////////////////////////
    // Subscription
    ////////////////////////////////////////////////////////
    if (multi_) {
      sub_cluster_indices_.subscribe(*pnh_, "input/cluster_indices", 1);
      sub_point_cloud_.subscribe(*pnh_, "input", 1);
      if (approximate_sync_) {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
        async_->connectInput(sub_cluster_indices_, sub_point_cloud_);
        async_->registerCallback(boost::bind(&EuclideanClustering::multi_extract, this, _1, _2));
      } else {
        sync_  = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_cluster_indices_, sub_point_cloud_);
        sync_->registerCallback(boost::bind(&EuclideanClustering::multi_extract, this, _1, _2));
      }
    } else {
      sub_input_ = pnh_->subscribe("input", 1, &EuclideanClustering::extract, this);
    }
  }

  void EuclideanClustering::unsubscribe()
  {
    if (multi_) {
      sub_cluster_indices_.unsubscribe();
      sub_point_cloud_.unsubscribe();
    } else {
      sub_input_.shutdown();
    }
  }
  
  void EuclideanClustering::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(
      diagnostic_msgs::DiagnosticStatus::OK,
      "EuclideanSegmentation running");

      jsk_topic_tools::addDiagnosticInformation(
        "Kdtree Construction", kdtree_acc_, stat);
      jsk_topic_tools::addDiagnosticInformation(
        "Euclidean Segmentation", segmentation_acc_, stat);
      stat.add("Cluster Num (Avg.)", cluster_counter_.mean());
      stat.add("Max Size of the cluster", maxsize_);
      stat.add("Min Size of the cluster", minsize_);
      stat.add("Cluster tolerance", tolerance);
      stat.add("Tracking tolerance", label_tracking_tolerance);
      stat.add("MultiEuclideanClustering", multi_);
      stat.add("Downsample enable", downsample_);
      if (downsample_) {
        stat.add("Leaf size", leaf_size_);
      }
    }
    DiagnosticNodelet::updateDiagnostic(stat);
  }
  
  void EuclideanClustering::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    tolerance = config.tolerance;
    label_tracking_tolerance = config.label_tracking_tolerance;
    maxsize_ = config.max_size;
    minsize_ = config.min_size;
    cluster_filter_type_ = config.cluster_filter;
    // downsample group
    downsample_ = config.downsample_enable;
    leaf_size_ = config.leaf_size;
  }

  std::vector<pcl::PointIndices> EuclideanClustering::pivotClusterIndices(
    std::vector<int>& pivot_table,
    std::vector<pcl::PointIndices>& cluster_indices)
  {
    std::vector<pcl::PointIndices> new_cluster_indices;
    new_cluster_indices.resize(pivot_table.size());
    for (size_t i = 0; i < pivot_table.size(); i++) {
      new_cluster_indices[i] = cluster_indices[pivot_table[i]];
    }
    return new_cluster_indices;
  }
  std::vector<int> EuclideanClustering::buildLabelTrackingPivotTable(
    double* D, Vector4fVector cogs, Vector4fVector new_cogs,
    double label_tracking_tolerance)
  {
    std::vector<int> pivot_table;
    // initialize pivot table
    pivot_table.resize(cogs.size());
    for (size_t i = 0; i < pivot_table.size(); i++)
      pivot_table[i] = i;
    for (size_t pivot_counter = 0; pivot_counter < pivot_table.size();
         pivot_counter++)
    {
      double minimum_distance = DBL_MAX;
      size_t minimum_previous_index = 0;
      size_t minimum_next_index = 0;
      for (size_t i = 0; i < cogs.size(); i++)
      {
        for (size_t j = 0; j < new_cogs.size(); j++)
        {
          double distance = D[i * cogs.size() + j];
          //ROS_INFO("distance %lux%lu: %f", i, j, distance);
          if (distance < minimum_distance)
          {
            minimum_distance = distance;
            minimum_previous_index = i;
            minimum_next_index = j;
          }
        }
      }
      if (minimum_distance > label_tracking_tolerance)
      {
        // ROS_WARN("minimum tracking distance exceeds tolerance: %f > %f",
        //          minimum_distance, label_tracking_tolerance);
        std::vector<int> dummy;
        return dummy;
      }
      pivot_table[minimum_previous_index] = minimum_next_index;
      // fill the D matrix with DBL_MAX
      for (size_t j = 0; j < new_cogs.size(); j++)
      {
        D[minimum_previous_index * cogs.size() + j] = DBL_MAX;
      }
    }
    return pivot_table;
  }

  void EuclideanClustering::computeDistanceMatrix(
    double* D,
    Vector4fVector& old_cogs,
    Vector4fVector& new_cogs)
  {
    for (size_t i = 0; i < old_cogs.size(); i++) {
      Eigen::Vector4f previous_cog = old_cogs[i];
      for (size_t j = 0; j < new_cogs.size(); j++) {
        Eigen::Vector4f next_cog = new_cogs[j];
        double distance = (next_cog - previous_cog).norm();
        D[i * old_cogs.size() + j] = distance;
      }
    }
  }

  void EuclideanClustering::removeDuplicatedIndices(
    pcl::PointIndices::Ptr indices)
  {
    std::sort(indices->indices.begin(), indices->indices.end());
    indices->indices.erase(
      std::unique(indices->indices.begin(), indices->indices.end()),
      indices->indices.end());
  }

  void EuclideanClustering::computeCentroidsOfClusters(
    Vector4fVector& ret,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::vector<pcl::PointIndices> cluster_indices)
  {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    ret.resize(cluster_indices.size());
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
      // build pointcloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
      for (size_t j = 0; j < cluster_indices[i].indices.size(); j++)
      {
        segmented_indices->indices.push_back(cluster_indices[i].indices[j]);
      }
      extract.setIndices(segmented_indices);
      extract.filter(*segmented_cloud);
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*segmented_cloud, center);
      ret[i] = center;
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EuclideanClustering, nodelet::Nodelet);

