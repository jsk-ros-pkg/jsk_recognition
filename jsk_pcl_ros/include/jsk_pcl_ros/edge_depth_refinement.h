// -*- mode: c++ -*-
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


#ifndef JSK_PCL_ROS_EDGE_DEPTH_REFINEMENT_H_
#define JSK_PCL_ROS_EDGE_DEPTH_REFINEMENT_H_

#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/SegmentArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include <jsk_pcl_ros/EdgeDepthRefinementConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/tuple/tuple.hpp>

#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros
{
  class EdgeDepthRefinement: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices > SyncPolicy;
    typedef pcl::PointXYZRGB PointT;
    typedef jsk_pcl_ros::EdgeDepthRefinementConfig Config;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    
    virtual void refine(
      const sensor_msgs::PointCloud2ConstPtr &point,
      const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices);
    
    virtual void removeOutliersByLine(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<int>& indices,
      pcl::PointIndices& inliers,
      pcl::ModelCoefficients& coefficients);
    
    virtual void removeOutliers(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<PCLIndicesMsg>& indices,
      std::vector<pcl::PointIndices::Ptr>& output_inliers,
      std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients);
    
    virtual void removeDuplicatedEdges(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<pcl::PointIndices::Ptr> inliers,
      const std::vector<pcl::ModelCoefficients::Ptr> coefficients,
      std::vector<pcl::PointIndices::Ptr>& output_inliers,
      std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients);
    
    virtual jsk_recognition_utils::Line::Ptr lineFromCoefficients(
      const pcl::ModelCoefficients::Ptr coefficients);
    
    virtual jsk_recognition_utils::Segment::Ptr segmentFromIndices(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<int>& indices,
      const jsk_recognition_utils::Line::Ptr& line);
    
    virtual void publishIndices(
      ros::Publisher& pub,
      ros::Publisher& pub_coefficients,
      ros::Publisher& pub_edges,
      const std::vector<pcl::PointIndices::Ptr> inliers,
      const std::vector<pcl::ModelCoefficients::Ptr> coefficients,
      const std_msgs::Header& header);
    
    virtual boost::tuple<int, int> findMinMaxIndex(
      const int width, const int height,
      const std::vector<int>& indices);
    
    virtual void integrateDuplicatedIndices(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::set<int>& duplicated_set,
      const std::vector<pcl::PointIndices::Ptr> all_inliers,
      pcl::PointIndices::Ptr& output_indices);
    
    virtual void configCallback (Config &config, uint32_t level);

    virtual void subscribe();
    virtual void unsubscribe();
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher pub_indices_, pub_outlier_removed_indices_;
    ros::Publisher pub_coefficients_, pub_outlier_removed_coefficients_;
    ros::Publisher pub_edges_, pub_outlier_removed_edges_;
    boost::mutex mutex_;
    ////////////////////////////////////////////////////////
    // outlier removal
    ////////////////////////////////////////////////////////
    double outlier_distance_threshold_;
    int min_inliers_;
    ////////////////////////////////////////////////////////
    // duplication removal
    ////////////////////////////////////////////////////////
    double duplication_angle_threshold_;
    double duplication_distance_threshold_;
  private:
    
  };
}

#endif
