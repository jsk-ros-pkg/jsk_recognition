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

#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/euclidean_cluster_extraction_nodelet.h" 

namespace jsk_pcl_ros
{

  void EuclideanClustering::extract(
    const sensor_msgs::PointCloud2ConstPtr &input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    EuclideanClusterExtraction<pcl::PointXYZ> impl;
    {
      jsk_topic_tools::ScopedTimer timer = kdtree_acc_.scopedTimer();
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
      tree->setInputCloud (cloud);
      impl.setClusterTolerance (tolerance);
      impl.setMinClusterSize (minsize_);
      impl.setMaxClusterSize (maxsize_);
      impl.setSearchMethod (tree);
      impl.setInputCloud (cloud);
    }
    
    {
      jsk_topic_tools::ScopedTimer timer = segmentation_acc_.scopedTimer();
      impl.extract (cluster_indices);
    }
    
    // Publish result indices
    jsk_pcl_ros::ClusterPointIndices result;
    result.cluster_indices.resize(cluster_indices.size());
    cluster_counter_.add(cluster_indices.size());
    result.header = input->header;
    if (cogs_.size() != 0 && cogs_.size() == cluster_indices.size()) {
      // tracking the labels
      //ROS_INFO("computing distance matrix");
      // compute distance matrix
      // D[i][j] --> distance between the i-th previous cluster
      //             and the current j-th cluster
      Vector4fVector new_cogs;
      computeCentroidsOfClusters(new_cogs, cloud, cluster_indices);
      double D[cogs_.size() * new_cogs.size()];
      computeDistanceMatrix(D, cogs_, new_cogs);
      std::vector<int> pivot_table = buildLabelTrackingPivotTable(D, cogs_, new_cogs, label_tracking_tolerance);
      if (pivot_table.size() != 0) {
        cluster_indices = pivotClusterIndices(pivot_table, cluster_indices);
      }
    }
    Vector4fVector tmp_cogs;
    computeCentroidsOfClusters(tmp_cogs, cloud, cluster_indices); // NB: not efficient
    cogs_ = tmp_cogs;
      
    for (size_t i = 0; i < cluster_indices.size(); i++) {
#if ROS_VERSION_MINIMUM(1, 10, 0)
      // hydro and later
      result.cluster_indices[i].header
        = pcl_conversions::fromPCL(cluster_indices[i].header);
#else
      // groovy
      result.cluster_indices[i].header = cluster_indices[i].header;
#endif
      result.cluster_indices[i].indices = cluster_indices[i].indices;
    }

    result_pub_.publish(result);
    
    jsk_pcl_ros::Int32Stamped::Ptr cluster_num_msg (new jsk_pcl_ros::Int32Stamped);
    cluster_num_msg->header = input->header;
    cluster_num_msg->data = cluster_indices.size();
    cluster_num_pub_.publish(cluster_num_msg);

    diagnostic_updater_->update();
  }


  bool EuclideanClustering::serviceCallback(
    jsk_pcl_ros::EuclideanSegment::Request &req,
    jsk_pcl_ros::EuclideanSegment::Response &res)
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
    PCLNodelet::onInit();
    
    ////////////////////////////////////////////////////////
    // diagnostics
    ////////////////////////////////////////////////////////
    diagnostic_updater_.reset(
      new TimeredDiagnosticUpdater(*pnh_, ros::Duration(1.0)));
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(
      getName() + "::EuclideanClustering",
      boost::bind(
        &EuclideanClustering::updateDiagnostic,
        this,
        _1));
    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    diagnostic_updater_->start();
    
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
    result_pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices> ("output", 1);
    cluster_num_pub_ = pnh_->advertise<jsk_pcl_ros::Int32Stamped> ("cluster_num", 1);

    ////////////////////////////////////////////////////////
    // Subscription
    ////////////////////////////////////////////////////////
    sub_input_ = pnh_->subscribe("input", 1, &EuclideanClustering::extract, this);
    service_ = pnh_->advertiseService(pnh_->resolveName("euclidean_clustering"),
                                      &EuclideanClustering::serviceCallback, this);
  }

  void EuclideanClustering::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(
      diagnostic_msgs::DiagnosticStatus::OK,
      "EuclideanSegmentation running");

      addDiagnosticInformation(
        "Kdtree Construction", kdtree_acc_, stat);
      addDiagnosticInformation(
        "Euclidean Segmentation", segmentation_acc_, stat);
      stat.add("Cluster Num (Avg.)", cluster_counter_.mean());
      stat.add("Max Size of the cluster", maxsize_);
      stat.add("Min Size of the cluster", minsize_);
      stat.add("Cluster tolerance", tolerance);
      stat.add("Tracking tolerance", label_tracking_tolerance);
    }
    else {
      addDiagnosticErrorSummary(
        "EuclideanClustering", vital_checker_, stat);
    }
  }
  
  void EuclideanClustering::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    tolerance = config.tolerance;
    label_tracking_tolerance = config.label_tracking_tolerance;
    maxsize_ = config.max_size;
    minsize_ = config.min_size;
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
typedef jsk_pcl_ros::EuclideanClustering EuclideanClustering;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, EuclideanClustering, EuclideanClustering, nodelet::Nodelet);

