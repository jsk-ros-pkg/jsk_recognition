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

#include "jsk_pcl_ros/multi_plane_sac_segmentation.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace jsk_pcl_ros
{
  void MultiPlaneSACSegmentation::onInit()
  {
    PCLNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&MultiPlaneSACSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("use_normal", use_normal_, false);
    ////////////////////////////////////////////////////////
    // publishers
    ////////////////////////////////////////////////////////
    pub_inliers_ = pnh_->advertise<ClusterPointIndices>("output_indices", 1);
    pub_coefficients_
      = pnh_->advertise<ModelCoefficientsArray>("output_coefficients", 1);
    pub_polygons_ = pnh_->advertise<PolygonArray>("output_polygons", 1);
  }

  void MultiPlaneSACSegmentation::subscribe()
  {
    ////////////////////////////////////////////////////////
    // subscriber
    ////////////////////////////////////////////////////////
    if (!use_normal_) {
      sub_ = pnh_->subscribe("input", 1, &MultiPlaneSACSegmentation::segment, this);
    }
    else {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_normal_.subscribe(*pnh_, "input_normal", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_normal_);
      sync_->registerCallback(boost::bind(&MultiPlaneSACSegmentation::segment,
                                          this, _1, _2));
    }
  }

  void MultiPlaneSACSegmentation::unsubscribe()
  {
    ////////////////////////////////////////////////////////
    // subscriber
    ////////////////////////////////////////////////////////
    if (!use_normal_) {
      sub_.shutdown();
    }
    else {
      sub_input_.unsubscribe();
      sub_normal_.unsubscribe();
    }
  }

  void MultiPlaneSACSegmentation::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    outlier_threshold_ = config.outlier_threshold;
    min_inliers_ = config.min_inliers;
    min_points_ = config.min_points;
    max_iterations_ = config.max_iterations;
  }
  
  void MultiPlaneSACSegmentation::applyRecursiveRANSAC(
      const pcl::PointCloud<PointT>::Ptr& input,
      const pcl::PointCloud<pcl::Normal>::Ptr& normal,
      std::vector<pcl::PointIndices::Ptr>& output_inliers,
      std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients,
      std::vector<ConvexPolygon::Ptr>& output_polygons)
  {
    pcl::PointCloud<PointT>::Ptr rest_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr rest_normal (new pcl::PointCloud<pcl::Normal>);
    *rest_cloud = *input;
    *rest_normal = *normal;
    int counter = 0;
    while (true) {
      NODELET_INFO("apply RANSAC: %d", counter);
      ++counter;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      if (!use_normal_) {
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (outlier_threshold_);
        seg.setInputCloud(rest_cloud);
        seg.setMaxIterations (max_iterations_);
        seg.segment (*inliers, *coefficients);
      }
      else {
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (outlier_threshold_);
        seg.setInputCloud(rest_cloud);
        seg.setInputNormals(rest_normal);
        seg.setMaxIterations (max_iterations_);
        seg.segment (*inliers, *coefficients);
      }
      NODELET_INFO("inliers: %lu", inliers->indices.size());
      if (inliers->indices.size() < min_inliers_) {
        return;
      }
      output_inliers.push_back(inliers);
      output_coefficients.push_back(coefficients);
      ConvexPolygon::Ptr convex = convexFromCoefficientsAndInliers<PointT>(
        input, inliers, coefficients);
      output_polygons.push_back(convex);
      // prepare for next loop
      pcl::PointCloud<PointT>::Ptr next_rest_cloud
        (new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr next_rest_normal
        (new pcl::PointCloud<pcl::Normal>);
      pcl::ExtractIndices<PointT> ex;
      ex.setInputCloud(rest_cloud);
      ex.setIndices(inliers);
      ex.setNegative(true);
      ex.setKeepOrganized(true);
      ex.filter(*next_rest_cloud);
      if (use_normal_) {
        pcl::ExtractIndices<pcl::Normal> ex_normal;
        ex_normal.setInputCloud(rest_normal);
        ex_normal.setIndices(inliers);
        ex_normal.setNegative(true);
        ex_normal.setKeepOrganized(true);
        ex_normal.filter(*next_rest_normal);
      }
      if (next_rest_cloud->points.size() < min_points_) {
        return;
      }
      rest_cloud = next_rest_cloud;
      rest_normal = next_rest_normal;
    }
  }
  void MultiPlaneSACSegmentation::segment(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::PointCloud2::ConstPtr& msg_normal)
  {
    ROS_INFO("segment");
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*msg, *input);
    pcl::fromROSMsg(*msg_normal, *normal);
    std::vector<pcl::PointIndices::Ptr> inliers;
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    std::vector<ConvexPolygon::Ptr> convexes;
    applyRecursiveRANSAC(input, normal, inliers, coefficients, convexes);

    jsk_pcl_ros::ClusterPointIndices ros_indices_output;
    jsk_pcl_ros::ModelCoefficientsArray ros_coefficients_output;
    jsk_pcl_ros::PolygonArray ros_polygon_output;
    ros_indices_output.header = msg->header;
    ros_coefficients_output.header = msg->header;
    ros_polygon_output.header = msg->header;
    ros_indices_output.cluster_indices
      = pcl_conversions::convertToROSPointIndices(inliers, msg->header);
    ros_coefficients_output.coefficients
      = pcl_conversions::convertToROSModelCoefficients(coefficients, msg->header);
    pub_inliers_.publish(ros_indices_output);
    pub_coefficients_.publish(ros_coefficients_output);
    for (size_t i = 0; i < convexes.size(); i++) {
      geometry_msgs::PolygonStamped polygon;
      polygon.header = msg->header;
      polygon.polygon = convexes[i]->toROSMsg();
      ros_polygon_output.polygons.push_back(polygon);
    }
    pub_polygons_.publish(ros_polygon_output);
  }
  
  void MultiPlaneSACSegmentation::segment(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    segment(msg, sensor_msgs::PointCloud2::ConstPtr());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::MultiPlaneSACSegmentation, nodelet::Nodelet);

