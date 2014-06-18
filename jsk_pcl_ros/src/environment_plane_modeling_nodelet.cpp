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

#include "jsk_pcl_ros/environment_plane_modeling.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>

#include <pluginlib/class_list_macros.h>


namespace jsk_pcl_ros
{
  void EnvironmentPlaneModeling::onInit()
  {
    PCLNodelet::onInit();
    environment_id_ = 0;
    distance_thr_ = 0.01;       // 1cm
    sampling_d_ = 0.01;         // 1cm

    debug_polygon_pub_
      = pnh_->advertise<geometry_msgs::PolygonStamped>("debug_polygon", 1);
    debug_env_polygon_pub_
      = pnh_->advertise<geometry_msgs::PolygonStamped>("debug_env_polygon", 1);
    debug_pointcloud_pub_
      = pnh_->advertise<sensor_msgs::PointCloud2>("debug_sampled_pointcloud", 1);
    debug_env_pointcloud_pub_
      = pnh_->advertise<sensor_msgs::PointCloud2>("debug_pointcloud", 1);
    
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_input_, sub_indices_, sub_polygons_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&EnvironmentPlaneModeling::inputCallback,
                                        this, _1, _2, _3, _4));
    
    lock_service_
      = pnh_->advertiseService("lock", &EnvironmentPlaneModeling::lockCallback,
                               this);
    polygon_on_environment_service_
      = pnh_->advertiseService("polygon_on_environment",
                               &EnvironmentPlaneModeling::polygonOnEnvironmentCallback, this);
  }

  void EnvironmentPlaneModeling::inputCallback(
    const sensor_msgs::PointCloud2::ConstPtr& input,
    const jsk_pcl_ros::ClusterPointIndices::ConstPtr& input_indices,
    const jsk_pcl_ros::PolygonArray::ConstPtr& input_polygons,
    const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& input_coefficients)
  {
    boost::mutex::scoped_lock(mutex_);
    latest_input_ = input;
    latest_input_indices_ = input_indices;
    latest_input_polygons_ = input_polygons;
    latest_input_coefficients_ = input_coefficients;
  }

  bool EnvironmentPlaneModeling::lockCallback(
    jsk_pcl_ros::EnvironmentLock::Request& req,
    jsk_pcl_ros::EnvironmentLock::Response& res)
  {
    if (!latest_input_) {
      NODELET_ERROR("[EnvironmentPlaneModeling] no valid input yet");
      return false;
    }
    //{
      boost::mutex::scoped_lock(mutex_);
      processing_input_ = latest_input_;
      processing_input_indices_ = latest_input_indices_;
      processing_input_polygons_ = latest_input_polygons_;
      processing_input_coefficients_ = latest_input_coefficients_;
      //}
    NODELET_INFO("lock %lu pointclouds",
                 processing_input_indices_->cluster_indices.size());
    // build kdtrees
    kdtrees_.clear();
    separated_point_cloud_.clear();
    pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*processing_input_, *pcl_cloud);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(pcl_cloud);
    for (size_t i = 0;
         i < processing_input_indices_->cluster_indices.size();
         i++) {
      pcl::PointCloud<PointT>::Ptr nonprojected_input (new pcl::PointCloud<PointT>);
      pcl::PointIndices::Ptr indices (new pcl::PointIndices);
      pcl_conversions::toPCL(processing_input_indices_->cluster_indices[i],
                             *indices);
      extract.setIndices(indices);
      extract.filter(*nonprojected_input);
      // project `nonprojected_input' to the plane
      pcl::PointCloud<PointT>::Ptr kdtree_input (new pcl::PointCloud<PointT>);
      pcl::ProjectInliers<PointT> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
      plane_coefficients->values = processing_input_coefficients_->coefficients[i].values;
      proj.setModelCoefficients(plane_coefficients);
      proj.setInputCloud(nonprojected_input);
      proj.filter(*kdtree_input);
      pcl::KdTreeFLANN<PointT>::Ptr kdtree (new pcl::KdTreeFLANN<PointT>);
      kdtree->setInputCloud(kdtree_input);
      kdtrees_.push_back(kdtree);
      separated_point_cloud_.push_back(kdtree_input);
    }
    res.environment_id = ++environment_id_;
    return true;
  }

  bool EnvironmentPlaneModeling::polygonNearEnoughToPointCloud(
    const size_t plane_i,
    const pcl::PointCloud<PointT>::Ptr sampled_point_cloud)
  {
    geometry_msgs::PolygonStamped target_polygon
      = processing_input_polygons_->polygons[plane_i];
    // debug information
    debug_env_polygon_pub_.publish(target_polygon);
    sensor_msgs::PointCloud2 debug_env_pointcloud;
    toROSMsg(*separated_point_cloud_[plane_i], debug_env_pointcloud);
    debug_env_pointcloud.header = processing_input_->header;
    debug_env_pointcloud_pub_.publish(debug_env_pointcloud);
      
    // check collision
    // all the sampled points should near enough from target_polygon
    pcl::KdTreeFLANN<PointT>::Ptr target_kdtree = kdtrees_[plane_i];
    // NODELET_INFO("checking %lu points", target_kdtree->getInputCloud()->points.size());
    for (size_t i = 0; i < sampled_point_cloud->points.size(); i++) {
      PointT p = sampled_point_cloud->points[i];
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      if (target_kdtree->radiusSearch(p,
                                      distance_thr_,
                                      k_indices,
                                      k_sqr_distances, 1) == 0) {
        // NODELET_INFO("%lu plane is too far (%lu sampled point [%f, %f, %f])",
        //              plane_i, i, 
        //              p.x, p.y, p.z);
        return false;
      }
      // target_kdtree->nearestKSearch(p, 1, k_indices, k_sqr_distances);
      // NODELET_INFO("%lu plane 's nearest distance is: %f (%f, %f, %f)",
      //              plane_i, k_sqr_distances[0], p.x, p.y, p.z);
      // if (k_sqr_distances[0] > distance_thr_) {
      //   return false;
      // }
      else {
         // NODELET_INFO("%lu plane is near enough (%lu sampled point [%f, %f, %f])",
         //              plane_i, i,
         //              p.x, p.y, p.z);
      }
    }
    return true;
  }
  
  bool EnvironmentPlaneModeling::polygonOnEnvironmentCallback(
    jsk_pcl_ros::PolygonOnEnvironment::Request& req,
    jsk_pcl_ros::PolygonOnEnvironment::Response& res)
  {
    if (req.environment_id != environment_id_) {
      NODELET_FATAL("environment id does not match. %u is provided but the environment stored is %u",
                    req.environment_id,
                    environment_id_);
      return false;
    }
    ros::Time before = ros::Time::now();
    
    pcl::PointCloud<PointT>::Ptr sampled_point_cloud (new pcl::PointCloud<PointT>());
    samplePolygonToPointCloud(req.polygon, sampled_point_cloud, sampling_d_);

    sensor_msgs::PointCloud2 debug_sampled_pointcloud;
    toROSMsg(*sampled_point_cloud, debug_sampled_pointcloud);
    debug_sampled_pointcloud.header = processing_input_->header;
    debug_pointcloud_pub_.publish(debug_sampled_pointcloud);

    bool found_contact_plane = false;
    for (size_t plane_i = 0;
         plane_i < processing_input_polygons_->polygons.size();
         plane_i++) {
      debug_polygon_pub_.publish(req.polygon);
      if (polygonNearEnoughToPointCloud(plane_i, sampled_point_cloud)) {
        found_contact_plane = true;
        break;
      }
    }
    ros::Time after = ros::Time::now();
    NODELET_INFO("kdtree took %f sec", (after - before).toSec());
    if (found_contact_plane) {
      res.result = true;
      return true;
    }
    else {
      res.result = false;
      res.reason = "the polygon is not on any plane";
      return true;
    }
    
  }

  void EnvironmentPlaneModeling::msgToPCL(
    const geometry_msgs::Point32 msg_point,
    PointT& pcl_point)
  {
    pcl_point.x = msg_point.x;
    pcl_point.y = msg_point.y;
    pcl_point.z = msg_point.z;
  }

  void EnvironmentPlaneModeling::internalPointDivide(const PointT& A, const PointT& B,
                                                     const double ratio,
                                                     PointT& output)
  {
    output.x = (1 - ratio) * A.x + ratio * B.x;
    output.y = (1 - ratio) * A.y + ratio * B.y;
    output.z = (1 - ratio) * A.z + ratio * B.z;
  }

  // TODO: should cache the pointcloud
  // take tf into account
  void EnvironmentPlaneModeling::samplePolygonToPointCloud(
    const geometry_msgs::PolygonStamped sample_polygon,
    pcl::PointCloud<PointT>::Ptr output,
    double sampling_param)
  {
    for (size_t i = 0; i < sample_polygon.polygon.points.size(); i++) {
      // geometry_msgs::Point32 from_point, to_point;
      // from_point = sample_polygon.polygon.points[i];
      // if (i == sample_polygon.polygon.points.size() - 1) {
      //   to_point = sample_polygon.polygon.points[0];
      // }
      // PointT from_pcl_point, to_pcl_point;
      // msgToPCL(from_point, from_pcl_point);
      // msgToPCL(to_point, to_pcl_point);
      //double d = pcl::euclideanDistance(from_pcl_point, to_pcl_point);
      // int sampling_num = (int)(d / sampling_param); // +1 or not
      // for (int j = 0; j < sampling_num; j++) {
      //   PointT dividing_point;
      //   internalPointDivide(from_pcl_point, to_pcl_point,
      //                       j / (double)sampling_num, dividing_point);
      //   output->points.push_back(dividing_point);
      // }
      //NODELET_INFO("sampled %d points", sampling_num);
      geometry_msgs::Point32 point = sample_polygon.polygon.points[i];
      PointT pcl_point;
      msgToPCL(point, pcl_point);
      output->points.push_back(pcl_point);
    }
  }
  
}

typedef jsk_pcl_ros::EnvironmentPlaneModeling EnvironmentPlaneModeling;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, EnvironmentPlaneModeling, EnvironmentPlaneModeling, nodelet::Nodelet);
