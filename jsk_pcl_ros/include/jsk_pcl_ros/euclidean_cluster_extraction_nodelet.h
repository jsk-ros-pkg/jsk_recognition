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
#include <pcl/common/centroid.h>

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "jsk_pcl_ros/EuclideanSegment.h"
#include "jsk_pcl_ros/Int32Stamped.h"

#include "jsk_pcl_ros/EuclideanClusteringConfig.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <Eigen/StdVector>
#include "jsk_pcl_ros/pcl_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include <jsk_topic_tools/time_accumulator.h>

#include "jsk_pcl_ros/connection_based_nodelet.h"

using namespace std;
using namespace pcl;

namespace jsk_pcl_ros
{
  class EuclideanClustering : public ConnectionBasedNodelet
  {
  public:
    typedef jsk_pcl_ros::EuclideanClusteringConfig Config;
    typedef std::vector<Eigen::Vector4f,
                        Eigen::aligned_allocator<Eigen::Vector4f> >
    Vector4fVector;
    
  protected:
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
   
    void configCallback (Config &config, uint32_t level);
    ros::Publisher result_pub_;
    ros::Subscriber sub_input_;
    ros::Publisher cluster_num_pub_;

    ros::ServiceServer service_;

    double tolerance;
    double label_tracking_tolerance;
    int minsize_;
    int maxsize_;
    
    jsk_pcl_ros::TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    jsk_topic_tools::VitalChecker::Ptr vital_checker_;
    jsk_topic_tools::TimeAccumulator segmentation_acc_;
    jsk_topic_tools::TimeAccumulator kdtree_acc_;
    Counter cluster_counter_;
    
    // the list of COGs of each cluster
    Vector4fVector cogs_;
    
    virtual void onInit();
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &input);
    bool serviceCallback(jsk_pcl_ros::EuclideanSegment::Request &req,
                         jsk_pcl_ros::EuclideanSegment::Response &res);
    void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
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

    virtual void subscribe();
    virtual void unsubscribe();
  };
    
}

#endif
