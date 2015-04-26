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


#ifndef JSK_PCL_ROS_ORGANIZED_EDGE_DETECTOR_H_
#define JSK_PCL_ROS_ORGANIZED_EDGE_DETECTOR_H_

#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/OrganizedEdgeDetectorConfig.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include "jsk_topic_tools/connection_based_nodelet.h"

namespace jsk_pcl_ros
{
  class OrganizedEdgeDetector: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef jsk_pcl_ros::OrganizedEdgeDetectorConfig Config;
    
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void configCallback (Config &config, uint32_t level);
    virtual void estimate(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void estimateNormal(
      const pcl::PointCloud<PointT>::Ptr& input,
      pcl::PointCloud<pcl::Normal>::Ptr output,
      const std_msgs::Header& header);
    virtual void estimateEdge(
      const pcl::PointCloud<PointT>::Ptr& input,
      const pcl::PointCloud<pcl::Normal>::Ptr& normal,
      pcl::PointCloud<pcl::Label>::Ptr& output,
      std::vector<pcl::PointIndices>& label_indices);
    virtual void publishIndices(
      ros::Publisher& pub,
      ros::Publisher& pub_indices,
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<int>& indices,
      const std_msgs::Header& header);
    virtual void estimateStraightEdges(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<int>& indices,
      const std_msgs::Header& header,
      std::vector<std::vector<int> >& output_indices);
    virtual void publishStraightEdges(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std_msgs::Header& header,
      const std::vector<std::vector<int> > indices);

    virtual void subscribe();
    virtual void unsubscribe();
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Publisher pub_nan_boundary_edges_indices_,
      pub_occluding_edges_indices_, pub_occluded_edges_indices_,
      pub_curvature_edges_indices_, pub_rgb_edges_indices_, pub_all_edges_indices_;
    ros::Publisher pub_nan_boundary_edges_,
      pub_occluding_edges_, pub_occluded_edges_,
      pub_curvature_edges_, pub_rgb_edges_, pub_all_edges_;
    ros::Publisher pub_normal_;
    ros::Publisher pub_straight_edges_indices_;
    image_transport::Publisher pub_edge_image_, pub_hough_image_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    ////////////////////////////////////////////////////////
    // parameters for normal estimation
    ////////////////////////////////////////////////////////
    int estimation_method_;
    bool border_policy_ignore_;
    double max_depth_change_factor_;
    double normal_smoothing_size_;
    bool depth_dependent_smoothing_;
    bool publish_normal_;
    ////////////////////////////////////////////////////////
    // parameters for edge detection
    ////////////////////////////////////////////////////////
    double depth_discontinuation_threshold_;
    int max_search_neighbors_;
    bool use_nan_boundary_;
    bool use_occluding_;
    bool use_occluded_;
    bool use_curvature_;
    bool use_rgb_;
    
    ////////////////////////////////////////////////////////
    // straight line detection
    ////////////////////////////////////////////////////////
    bool use_straightline_detection_;
    double rho_;
    double theta_;
    int    straightline_threshold_;
    double min_line_length_;
    double max_line_gap_;
    bool publish_debug_image_;

  private:
    
  };
}

#endif
