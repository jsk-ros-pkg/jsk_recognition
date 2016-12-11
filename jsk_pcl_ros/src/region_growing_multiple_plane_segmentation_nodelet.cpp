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

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_pcl_ros/region_growing_multiple_plane_segmentation.h"
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace jsk_pcl_ros
{

  void RegionGrowingMultiplePlaneSegmentation::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &RegionGrowingMultiplePlaneSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_polygons_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output/polygons", 1);
    pub_inliers_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output/inliers", 1);
    pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output/coefficients", 1);
    pub_clustering_result_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output/clustering_result", 1);
    pub_latest_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/latest_time", 1);
    pub_average_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/average_time", 1);
    done_initialization_ = true;
    onInitPostProcess();
  }

  void RegionGrowingMultiplePlaneSegmentation::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_normal_.subscribe(*pnh_, "input_normal", 1);
    sync_
      = boost::make_shared<
      message_filters::Synchronizer<NormalSyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_normal_);
    sync_->registerCallback(
      boost::bind(&RegionGrowingMultiplePlaneSegmentation::segment, this,
                  _1, _2));
  }

  void RegionGrowingMultiplePlaneSegmentation::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_normal_.unsubscribe();
  }

  void RegionGrowingMultiplePlaneSegmentation::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    angular_threshold_ = config.angular_threshold;
    distance_threshold_ = config.distance_threshold;
    max_curvature_ = config.max_curvature;
    min_size_ = config.min_size;
    max_size_ = config.max_size;
    min_area_ = config.min_area;
    max_area_ = config.max_area;
    cluster_tolerance_ = config.cluster_tolerance;
    ransac_refine_outlier_distance_threshold_
      = config.ransac_refine_outlier_distance_threshold;
    ransac_refine_max_iterations_
      = config.ransac_refine_max_iterations;
  }

  void RegionGrowingMultiplePlaneSegmentation::ransacEstimation(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::PointIndices::Ptr& indices,
    pcl::PointIndices& inliers,
    pcl::ModelCoefficients& coefficient)
  {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (ransac_refine_outlier_distance_threshold_);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.setMaxIterations (ransac_refine_max_iterations_);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.segment(inliers, coefficient);
  }
  
  void RegionGrowingMultiplePlaneSegmentation::segment(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::PointCloud2::ConstPtr& normal_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_initialization_) {
      return;
    }
    vital_checker_->poke();
    {
      jsk_recognition_utils::ScopedWallDurationReporter r
        = timer_.reporter(pub_latest_time_, pub_average_time_);
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
      pcl::fromROSMsg(*msg, *cloud);
      pcl::fromROSMsg(*normal_msg, *normal);
      // concatenate fields
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr
        all_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::concatenateFields(*cloud, *normal, *all_cloud);
      pcl::PointIndices::Ptr indices (new pcl::PointIndices);
      for (size_t i = 0; i < all_cloud->points.size(); i++) {
        if (!std::isnan(all_cloud->points[i].x) &&
            !std::isnan(all_cloud->points[i].y) &&
            !std::isnan(all_cloud->points[i].z) &&
            !std::isnan(all_cloud->points[i].normal_x) &&
            !std::isnan(all_cloud->points[i].normal_y) &&
            !std::isnan(all_cloud->points[i].normal_z) &&
            !std::isnan(all_cloud->points[i].curvature)) {
          if (all_cloud->points[i].curvature < max_curvature_) {
            indices->indices.push_back(i);
          }
        }
      }
      pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    
      // vector of pcl::PointIndices
      pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
      cec.setInputCloud (all_cloud);
      cec.setIndices(indices);
      cec.setClusterTolerance(cluster_tolerance_);
      cec.setMinClusterSize(min_size_);
      //cec.setMaxClusterSize(max_size_);
      {
        boost::mutex::scoped_lock lock2(global_custom_condigion_function_mutex);
        setCondifionFunctionParameter(angular_threshold_, distance_threshold_);
        cec.setConditionFunction(
          &RegionGrowingMultiplePlaneSegmentation::regionGrowingFunction);
        //ros::Time before = ros::Time::now();
        cec.segment (*clusters);
        // ros::Time end = ros::Time::now();
        // ROS_INFO("segment took %f sec", (before - end).toSec());
      }
      // Publish raw cluster information 
      // pcl::IndicesCluster is typdefed as std::vector<pcl::PointIndices>
      jsk_recognition_msgs::ClusterPointIndices ros_clustering_result;
      ros_clustering_result.cluster_indices
        = pcl_conversions::convertToROSPointIndices(*clusters, msg->header);
      ros_clustering_result.header = msg->header;
      pub_clustering_result_.publish(ros_clustering_result);

      // estimate planes
      std::vector<pcl::PointIndices::Ptr> all_inliers;
      std::vector<pcl::ModelCoefficients::Ptr> all_coefficients;
      jsk_recognition_msgs::PolygonArray ros_polygon;
      ros_polygon.header = msg->header;
      for (size_t i = 0; i < clusters->size(); i++) {
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr cluster = boost::make_shared<pcl::PointIndices>((*clusters)[i]);
        ransacEstimation(cloud, cluster,
                         *plane_inliers, *plane_coefficients);
        if (plane_inliers->indices.size() > 0) {
          jsk_recognition_utils::ConvexPolygon::Ptr convex
            = jsk_recognition_utils::convexFromCoefficientsAndInliers<pcl::PointXYZRGB>(
              cloud, plane_inliers, plane_coefficients);
          if (convex) {
            if (min_area_ > convex->area() || convex->area() > max_area_) {
              continue;
            }
            {
              // check direction consistency of coefficients and convex
              Eigen::Vector3f coefficient_normal(plane_coefficients->values[0],
                                                 plane_coefficients->values[1],
                                                 plane_coefficients->values[2]);
              if (convex->getNormalFromVertices().dot(coefficient_normal) < 0) {
                convex = boost::make_shared<jsk_recognition_utils::ConvexPolygon>(convex->flipConvex());
              }
            }
            // Normal should direct to origin
            {
              Eigen::Vector3f p = convex->getPointOnPlane();
              Eigen::Vector3f n = convex->getNormal();
              if (p.dot(n) > 0) {
                convex = boost::make_shared<jsk_recognition_utils::ConvexPolygon>(convex->flipConvex());
                Eigen::Vector3f new_normal = convex->getNormal();
                plane_coefficients->values[0] = new_normal[0];
                plane_coefficients->values[1] = new_normal[1];
                plane_coefficients->values[2] = new_normal[2];
                plane_coefficients->values[3] = convex->getD();
              }
            }
          
            geometry_msgs::PolygonStamped polygon;
            polygon.polygon = convex->toROSMsg();
            polygon.header = msg->header;
            ros_polygon.polygons.push_back(polygon);
            all_inliers.push_back(plane_inliers);
            all_coefficients.push_back(plane_coefficients);
          }
        }
      }
    
      jsk_recognition_msgs::ClusterPointIndices ros_indices;
      ros_indices.cluster_indices =
        pcl_conversions::convertToROSPointIndices(all_inliers, msg->header);
      ros_indices.header = msg->header;
      pub_inliers_.publish(ros_indices);
      jsk_recognition_msgs::ModelCoefficientsArray ros_coefficients;
      ros_coefficients.header = msg->header;
      ros_coefficients.coefficients =
        pcl_conversions::convertToROSModelCoefficients(
          all_coefficients, msg->header);
      pub_coefficients_.publish(ros_coefficients);
      pub_polygons_.publish(ros_polygon);
    }
  }

  double RegionGrowingMultiplePlaneSegmentation::global_angular_threshold = 0.0;
  double RegionGrowingMultiplePlaneSegmentation::global_distance_threshold = 0.0;
  boost::mutex RegionGrowingMultiplePlaneSegmentation::global_custom_condigion_function_mutex;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::RegionGrowingMultiplePlaneSegmentation, nodelet::Nodelet);
