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

#include "jsk_pcl_ros/organized_edge_detector.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"

#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include "jsk_pcl_ros/pcl_util.h"

namespace jsk_pcl_ros
{
  void OrganizedEdgeDetector::onInit()
  {
    PCLNodelet::onInit();
    ////////////////////////////////////////////////////////
    // indices publishers
    ////////////////////////////////////////////////////////
    pub_nan_boundary_edges_indices_
      = pnh_->advertise<PCLIndicesMsg>("output_nan_boundary_edge_indices", 1);
    pub_occluding_edges_indices_
      = pnh_->advertise<PCLIndicesMsg>("output_occluding_edge_indices", 1);
    pub_occluded_edges_indices_
      = pnh_->advertise<PCLIndicesMsg>("output_occluded_edge_indices", 1);
    pub_curvature_edges_indices_
      = pnh_->advertise<PCLIndicesMsg>("output_curvature_edge_indices", 1);
    pub_rgb_edges_indices_
      = pnh_->advertise<PCLIndicesMsg>("output_rgb_edge_indices", 1);
    pub_all_edges_indices_
      = pnh_->advertise<PCLIndicesMsg>("output_indices", 1);
    ////////////////////////////////////////////////////////
    // pointcloud publishers
    ////////////////////////////////////////////////////////
    pub_normal_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_normal", 1);
    pub_nan_boundary_edges_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output_nan_boundary_edge", 1);
    pub_occluding_edges_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output_occluding_edge", 1);
    pub_occluded_edges_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output_occluded_edge", 1);
    pub_curvature_edges_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output_curvature_edge", 1);
    pub_rgb_edges_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output_rgb_edge", 1);
    pub_all_edges_
      = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    ////////////////////////////////////////////////////////
    // setup dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OrganizedEdgeDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);

    
    sub_ = pnh_->subscribe("input", 1, &OrganizedEdgeDetector::estimate, this);
  }

  void OrganizedEdgeDetector::estimateNormal(
    const pcl::PointCloud<PointT>::Ptr& input,
    pcl::PointCloud<pcl::Normal>::Ptr output,
    const std_msgs::Header& header)
  {
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    if (estimation_method_ == 0) {
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    }
    else if (estimation_method_ == 1) {
     ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    }
    else if (estimation_method_ == 2) {
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    }
    else {
      NODELET_FATAL("unknown estimation method: %d", estimation_method_);
      return;
    }

    if (border_policy_ignore_) {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_IGNORE);
    }
    else {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_MIRROR);
    }

    ne.setMaxDepthChangeFactor(max_depth_change_factor_);
    ne.setNormalSmoothingSize(normal_smoothing_size_);
    ne.setDepthDependentSmoothing(depth_dependent_smoothing_);
    ne.setInputCloud(input);
    ne.compute(*output);
    if (publish_normal_) {
      sensor_msgs::PointCloud2 ros_output;
      pcl::toROSMsg(*output, ros_output);
      ros_output.header = header;
      pub_normal_.publish(ros_output);
    }
  }

  void OrganizedEdgeDetector::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    max_depth_change_factor_ = config.max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    depth_dependent_smoothing_ = config.depth_dependent_smoothing;
    estimation_method_ = config.estimation_method;
    border_policy_ignore_ = config.border_policy_ignore;
    max_search_neighbors_ = config.max_search_neighbors;
    depth_discontinuation_threshold_ = config.depth_discontinuation_threshold;
    publish_normal_ = config.publish_normal;
    use_nan_boundary_ = config.use_nan_boundary;
    use_occluding_ = config.use_occluding;
    use_occluded_ = config.use_occluded;
    use_curvature_ = config.use_curvature;
    use_rgb_ = config.use_rgb;
  }
  
  void OrganizedEdgeDetector::estimateEdge(
    const pcl::PointCloud<PointT>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& normal,
    pcl::PointCloud<pcl::Label>::Ptr& output,
    std::vector<pcl::PointIndices>& label_indices)
  {
    pcl::OrganizedEdgeFromRGBNormals<PointT, pcl::Normal, pcl::Label> oed;
    oed.setDepthDisconThreshold (depth_discontinuation_threshold_);
    oed.setMaxSearchNeighbors (max_search_neighbors_);
    int flags = 0;
    if (use_nan_boundary_) {
      flags |= oed.EDGELABEL_NAN_BOUNDARY;
    }
    if (use_occluding_) {
      flags |= oed.EDGELABEL_OCCLUDING;
    }
    if (use_occluded_) {
      flags |= oed.EDGELABEL_OCCLUDED;
    }
    if (use_curvature_) {
      flags |= oed.EDGELABEL_HIGH_CURVATURE;
    }
    if (use_rgb_) {
      flags |= oed.EDGELABEL_RGB_CANNY;
    }
    oed.setEdgeType (flags);
    oed.setInputNormals(normal);
    oed.setInputCloud(input);
    oed.compute(*output, label_indices);
  }

  void OrganizedEdgeDetector::publishIndices(
    ros::Publisher& pub,
    ros::Publisher& pub_indices,
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& indices,
    const std_msgs::Header& header)
  {
    ////////////////////////////////////////////////////////
    // publish indices
    ////////////////////////////////////////////////////////
    PCLIndicesMsg msg;
    msg.header = header;
    msg.indices = indices;
    pub_indices.publish(msg);

    ////////////////////////////////////////////////////////
    // publish cloud
    ////////////////////////////////////////////////////////
    pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    ROS_INFO("%lu points", indices.size());
    pcl::copyPointCloud(*cloud, indices, *output);
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(*output, ros_output);
    ros_output.header = header;
    pub.publish(ros_output);
  }
  
  void OrganizedEdgeDetector::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (msg->height == 1) {
      NODELET_ERROR("[OrganizedEdgeDetector] organized pointcloud is required");
      return;
    }
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Label>::Ptr label(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    pcl::fromROSMsg(*msg, *cloud);
    
    estimateNormal(cloud, normal, msg->header);
    estimateEdge(cloud, normal, label, label_indices);
    ////////////////////////////////////////////////////////
    // build indices includes all the indices
    ////////////////////////////////////////////////////////
    std::vector<int> tmp1 = addIndices(label_indices[0].indices,
                                       label_indices[1].indices);
    std::vector<int> tmp2 = addIndices(tmp1,
                                       label_indices[2].indices);
    std::vector<int> tmp3 = addIndices(tmp2,
                                       label_indices[3].indices);
    std::vector<int> all = addIndices(tmp3,
                                       label_indices[4].indices);
    
    ////////////////////////////////////////////////////////
    // publish result
    ////////////////////////////////////////////////////////
    publishIndices(pub_nan_boundary_edges_, pub_nan_boundary_edges_indices_,
                   cloud,
                   label_indices[0].indices, msg->header);
    publishIndices(pub_occluding_edges_, pub_occluding_edges_indices_,
                   cloud,
                   label_indices[1].indices, msg->header);
    publishIndices(pub_occluded_edges_, pub_occluded_edges_indices_,
                   cloud,
                   label_indices[2].indices, msg->header);
    publishIndices(pub_curvature_edges_, pub_curvature_edges_indices_,
                   cloud,
                   label_indices[3].indices, msg->header);
    publishIndices(pub_rgb_edges_, pub_rgb_edges_indices_,
                   cloud,
                   label_indices[4].indices, msg->header);
    publishIndices(pub_all_edges_, pub_all_edges_indices_,
                   cloud,
                   all, msg->header);
  }
}

#include <pluginlib/class_list_macros.h>
typedef jsk_pcl_ros::OrganizedEdgeDetector OrganizedEdgeDetector;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, OrganizedEdgeDetector, OrganizedEdgeDetector, nodelet::Nodelet);
