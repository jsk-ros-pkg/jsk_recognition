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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros_utils/plane_concatenator.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/pcl_util.h"

namespace jsk_pcl_ros_utils
{
  void PlaneConcatenator::onInit()
  {
    DiagnosticNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &PlaneConcatenator::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output/indices", 1);
    pub_polygon_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output/polygons", 1);
    pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output/coefficients", 1);

    onInitPostProcess();
  }

  void PlaneConcatenator::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_polygon_.subscribe(*pnh_, "input/polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> > (100);
    sync_->connectInput(sub_cloud_, sub_indices_,
                        sub_polygon_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&PlaneConcatenator::concatenate, this,
                                        _1, _2, _3, _4));
  }

  void PlaneConcatenator::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_indices_.unsubscribe();
    sub_polygon_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }

  void PlaneConcatenator::concatenate(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_array_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_array_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    
    size_t nr_cluster = polygon_array_msg->polygons.size();
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // first convert to polygon instance
    std::vector<pcl::ModelCoefficients::Ptr> all_coefficients
      = pcl_conversions::convertToPCLModelCoefficients(
        coefficients_array_msg->coefficients);
    std::vector<pcl::PointIndices::Ptr> all_indices
      = pcl_conversions::convertToPCLPointIndices(indices_msg->cluster_indices);
    std::vector<pcl::PointCloud<PointT>::Ptr> all_clouds
      = jsk_recognition_utils::convertToPointCloudArray<PointT>(cloud, all_indices);
    std::vector<jsk_recognition_utils::Plane::Ptr> planes
      = jsk_recognition_utils::convertToPlanes(all_coefficients);
    // build connection map
    jsk_recognition_utils::IntegerGraphMap connection_map;
    for (size_t i = 0; i < nr_cluster; i++) {
      connection_map[i] = std::vector<int>();
      connection_map[i].push_back(i);
      // build kdtree
      pcl::KdTreeFLANN<PointT> kdtree;
      pcl::PointCloud<PointT>::Ptr focused_cloud = all_clouds[i];
      kdtree.setInputCloud(focused_cloud);
      // look up near polygon
      jsk_recognition_utils::Plane::Ptr focused_plane = planes[i];
      for (size_t j = i + 1; j < nr_cluster; j++) {
        jsk_recognition_utils::Plane::Ptr target_plane = planes[j];
        if (focused_plane->angle(*target_plane) < connect_angular_threshold_) {
          // second, check distance
          bool is_near_enough = isNearPointCloud(kdtree, all_clouds[j], target_plane);
          if (is_near_enough) {
            connection_map[i].push_back(j);
          }
        }
      }
    }
    std::vector<std::set<int> > cloud_sets;
    jsk_recognition_utils::buildAllGroupsSetFromGraphMap(connection_map, cloud_sets);
    // build new indices
    std::vector<pcl::PointIndices::Ptr> new_indices;
    std::vector<pcl::ModelCoefficients::Ptr> new_coefficients;
    for (size_t i = 0; i < cloud_sets.size(); i++) {
      pcl::PointIndices::Ptr new_one_indices (new pcl::PointIndices);
      new_coefficients.push_back(all_coefficients[*cloud_sets[i].begin()]);
      for (std::set<int>::iterator it = cloud_sets[i].begin();
           it != cloud_sets[i].end();
           ++it) {
        new_one_indices = jsk_recognition_utils::addIndices(*new_one_indices, *all_indices[*it]);
      }
      if (new_one_indices->indices.size() > min_size_) {
        new_indices.push_back(new_one_indices);
      }
    }
    
    // refinement
    std::vector<pcl::ModelCoefficients::Ptr> new_refined_coefficients;
    for (size_t i = 0; i < new_indices.size(); i++) {
      pcl::ModelCoefficients::Ptr refined_coefficients
        = refinement(cloud, new_indices[i], new_coefficients[i]);
      new_refined_coefficients.push_back(refined_coefficients);
    }

    // publish
    jsk_recognition_msgs::ClusterPointIndices new_ros_indices;
    jsk_recognition_msgs::ModelCoefficientsArray new_ros_coefficients;
    jsk_recognition_msgs::PolygonArray new_ros_polygons;
    new_ros_indices.header = cloud_msg->header;
    new_ros_coefficients.header = cloud_msg->header;
    new_ros_polygons.header = cloud_msg->header;
    for (size_t i = 0; i < new_indices.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr convex
        = jsk_recognition_utils::convexFromCoefficientsAndInliers<PointT>(
          cloud, new_indices[i], new_refined_coefficients[i]);
      if (convex->area() > min_area_ && convex->area() < max_area_) {
        geometry_msgs::PolygonStamped polygon;
        polygon.polygon = convex->toROSMsg();
        polygon.header = cloud_msg->header;
        new_ros_polygons.polygons.push_back(polygon);
        pcl_msgs::PointIndices ros_indices;
        ros_indices.header = cloud_msg->header;
        ros_indices.indices = new_indices[i]->indices;
        new_ros_indices.cluster_indices.push_back(ros_indices);
        pcl_msgs::ModelCoefficients ros_coefficients;
        ros_coefficients.header = cloud_msg->header;
        ros_coefficients.values = new_refined_coefficients[i]->values;
        new_ros_coefficients.coefficients.push_back(ros_coefficients);
      }
    }
    
    // new_ros_indices.cluster_indices
    //   = pcl_conversions::convertToROSPointIndices(new_indices, cloud_msg->header);
    // new_ros_coefficients.coefficients
    //   = pcl_conversions::convertToROSModelCoefficients(
    //     new_refined_coefficients, cloud_msg->header);

    pub_indices_.publish(new_ros_indices);
    pub_polygon_.publish(new_ros_polygons);
    pub_coefficients_.publish(new_ros_coefficients);
  }

  pcl::ModelCoefficients::Ptr PlaneConcatenator::refinement(
    pcl::PointCloud<PointT>::Ptr cloud,
    pcl::PointIndices::Ptr indices,
    pcl::ModelCoefficients::Ptr original_coefficients)
  {
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (ransac_refinement_outlier_threshold_);
    
    seg.setInputCloud(cloud);
    
    seg.setIndices(indices);
    seg.setMaxIterations (ransac_refinement_max_iteration_);
    Eigen::Vector3f normal (original_coefficients->values[0],
                            original_coefficients->values[1],
                            original_coefficients->values[2]);
    seg.setAxis(normal);
    // seg.setInputNormals(cloud);
    // seg.setDistanceFromOrigin(original_coefficients->values[3]);
    // seg.setEpsDist(ransac_refinement_eps_distance_);
    seg.setEpsAngle(ransac_refinement_eps_angle_);
    pcl::PointIndices::Ptr refined_inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr refined_coefficients(new pcl::ModelCoefficients);
    seg.segment(*refined_inliers, *refined_coefficients);
    if (refined_inliers->indices.size() > 0) {
      return refined_coefficients;
    }
    else {
      return original_coefficients;
    }
  }
  
  bool PlaneConcatenator::isNearPointCloud(
    pcl::KdTreeFLANN<PointT>& kdtree,
    pcl::PointCloud<PointT>::Ptr cloud,
    jsk_recognition_utils::Plane::Ptr target_plane)
  {
    pcl::PointCloud<PointT>::ConstPtr input_cloud = kdtree.getInputCloud();
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      if (kdtree.radiusSearch(p, connect_distance_threshold_,
                              k_indices, k_sqr_distances, 1) > 0) {
        // Decompose distance into pependicular distance and parallel distance
        const PointT near_p = input_cloud->points[k_indices[0]];
        Eigen::Affine3f plane_coordinates = target_plane->coordinates();
        Eigen::Vector3f plane_local_p = plane_coordinates.inverse() * p.getVector3fMap();
        Eigen::Vector3f plane_local_near_p = plane_coordinates.inverse() * near_p.getVector3fMap();
        Eigen::Vector3f plane_local_diff = plane_local_near_p - plane_local_p;
        double perpendicular_distance = std::abs(plane_local_diff[2]);
        if (perpendicular_distance < connect_perpendicular_distance_threshold_) {
          return true;
        }
      }
    }
    return false;
  }
    
  void PlaneConcatenator::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    connect_angular_threshold_ = config.connect_angular_threshold;
    connect_distance_threshold_ = config.connect_distance_threshold;
    connect_perpendicular_distance_threshold_ = config.connect_perpendicular_distance_threshold;
    ransac_refinement_max_iteration_ = config.ransac_refinement_max_iteration;
    ransac_refinement_outlier_threshold_
      = config.ransac_refinement_outlier_threshold;
    ransac_refinement_eps_angle_ = config.ransac_refinement_eps_angle;
    min_size_ = config.min_size;
    min_area_ = config.min_area;
    max_area_ = config.max_area;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PlaneConcatenator, nodelet::Nodelet);
