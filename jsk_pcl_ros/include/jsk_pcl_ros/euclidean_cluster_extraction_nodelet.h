// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_EUCLIDEAN_CLUSTER_EXTRACTION_NODELET_H_
#define JSK_PCL_ROS_EUCLIDEAN_CLUSTER_EXTRACTION_NODELET_H_

#include <ros/ros.h>
#include <ros/names.h>

#include <std_msgs/ColorRGBA.h>

#include <dynamic_reconfigure/server.h>

#include <pcl_ros/pcl_nodelet.h>

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/EuclideanSegment.h"
#include "jsk_recognition_msgs/Int32Stamped.h"

#include "jsk_pcl_ros/EuclideanClusteringConfig.h"
#include <Eigen/StdVector>
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/time_accumulator.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>

namespace jsk_pcl_ros
{
  class EuclideanClustering : public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::ClusterPointIndices,
      sensor_msgs::PointCloud2> SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      jsk_recognition_msgs::ClusterPointIndices,
      sensor_msgs::PointCloud2> ApproximateSyncPolicy;

    typedef jsk_pcl_ros::EuclideanClusteringConfig Config;
    typedef std::vector<Eigen::Vector4f,
                        Eigen::aligned_allocator<Eigen::Vector4f> >
    Vector4fVector;
    EuclideanClustering() : DiagnosticNodelet("EuclideanClustering") {}
  protected:
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
   
    void configCallback (Config &config, uint32_t level);
    ros::Publisher result_pub_;
    ros::Subscriber sub_input_;
    ros::Publisher cluster_num_pub_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;

    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_cluster_indices_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_cloud_;

    ros::ServiceServer service_;

    double tolerance;
    double label_tracking_tolerance;
    int minsize_;
    int maxsize_;
    bool downsample_;
    double leaf_size_;
    bool multi_;
    bool approximate_sync_;
    int queue_size_;
    int cluster_filter_type_;

    std::vector<std::vector<int> > downsample_to_original_indices_;
    std::vector<int> original_to_downsample_indices_;
    
    jsk_topic_tools::TimeAccumulator segmentation_acc_;
    jsk_topic_tools::TimeAccumulator kdtree_acc_;
    jsk_recognition_utils::Counter cluster_counter_;

    // the list of COGs of each cluster
    Vector4fVector cogs_;
    
    virtual void onInit();
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &input);
    virtual void multi_extract(const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_indices,
                               const sensor_msgs::PointCloud2::ConstPtr& input);
    virtual void clusteringClusterIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                          std::vector<pcl::PointIndices::Ptr> &cluster_indices,
                                          std::vector<pcl::PointIndices> &clustered_indices);
    bool serviceCallback(jsk_recognition_msgs::EuclideanSegment::Request &req,
                         jsk_recognition_msgs::EuclideanSegment::Response &res);
    virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual std::vector<pcl::PointIndices> pivotClusterIndices(
      std::vector<int>& pivot_table,
      std::vector<pcl::PointIndices>& cluster_indices);
      
    virtual std::vector<int> buildLabelTrackingPivotTable(
      double* D,
      Vector4fVector cogs,
      Vector4fVector new_cogs,
      double label_tracking_tolerance);
    
    virtual void computeDistanceMatrix(
      double* D,
      Vector4fVector& old_cogs,
      Vector4fVector& new_cogs);

    virtual void
    computeCentroidsOfClusters(Vector4fVector& ret,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               std::vector<pcl::PointIndices> cluster_indices);
    virtual void removeDuplicatedIndices(pcl::PointIndices::Ptr indices);

    void downsample_cloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& sampled_cloud,
      std::vector<std::vector<int> >& sampled_to_original_indices,
      std::vector<int>& original_to_sampled_indices,
      double leaf_size);

    virtual void subscribe();
    virtual void unsubscribe();
  };
    
}

#endif
