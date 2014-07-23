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
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <jsk_pcl_ros/SparseOccupancyGridArray.h>

#include <pluginlib/class_list_macros.h>

#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/grid_map.h"

namespace jsk_pcl_ros
{
  void EnvironmentPlaneModeling::onInit()
  {
    PCLNodelet::onInit();
    environment_id_ = 0;
    generation_ = 0;
    diagnostic_updater_.reset(new diagnostic_updater::Updater);
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add("Modeling Stats", boost::bind(&EnvironmentPlaneModeling::updateDiagnostic,
                                                           this,
                                                           _1));
    pnh_->param("history_accumulation", history_accumulation_, false);
    pnh_->param("history_statical_rejection", history_statical_rejection_, false);
    // setup publisher
    debug_polygon_pub_
      = pnh_->advertise<geometry_msgs::PolygonStamped>("debug_polygon", 1);
    debug_env_polygon_pub_
      = pnh_->advertise<geometry_msgs::PolygonStamped>("debug_env_polygon", 1);
    debug_pointcloud_pub_
      = pnh_->advertise<sensor_msgs::PointCloud2>("debug_sampled_pointcloud", 1);
    debug_env_pointcloud_pub_
      = pnh_->advertise<sensor_msgs::PointCloud2>("debug_pointcloud", 1);
    occlusion_result_polygons_pub_
      = pnh_->advertise<PolygonArray>(
        "occlusion_result_polygons", 1);
    occlusion_result_coefficients_pub_
      = pnh_->advertise<ModelCoefficientsArray>(
        "occlusion_result_coefficients", 1);
    occlusion_result_pointcloud_pub_
      = pnh_->advertise<sensor_msgs::PointCloud2>("occlusion_result_cloud", 1);
    occlusion_result_indices_pub_
      = pnh_->advertise<ClusterPointIndices>("occlusion_result_indices", 1);
    grid_map_array_pub_ = pnh_->advertise<SparseOccupancyGridArray>("output_grid_map", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EnvironmentPlaneModeling::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pnh_->param("continuous_estimation", continuous_estimation_, false);
    
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sub_static_polygons_.subscribe(*pnh_, "input_static_polygons", 1);
    sub_static_coefficients_.subscribe(*pnh_, "input_static_coefficients", 1);
    sync_->connectInput(sub_input_, sub_indices_,
                        sub_polygons_, sub_coefficients_,
                        sub_static_polygons_, sub_static_coefficients_);
    sync_->registerCallback(boost::bind(
                              &EnvironmentPlaneModeling::inputCallback,
                              this, _1, _2, _3, _4, _5, _6));
    
    lock_service_
      = pnh_->advertiseService("lock", &EnvironmentPlaneModeling::dummyLockCallback,
                               this);
    primitive_lock_service_
      = pnh_->advertiseService("primitive_lock", &EnvironmentPlaneModeling::primitiveLockCallback,
                               this);
    primitive_unlock_service_
      = pnh_->advertiseService("primitive_unlock", &EnvironmentPlaneModeling::primitiveUnlockCallback,
                               this);
    polygon_on_environment_service_
      = pnh_->advertiseService("polygon_on_environment",
                               &EnvironmentPlaneModeling::polygonOnEnvironmentCallback, this);
  }


  bool EnvironmentPlaneModeling::dummyLockCallback(EnvironmentLock::Request& req,
                                                   EnvironmentLock::Response& res)
  {
    return true;
  }

  bool EnvironmentPlaneModeling::primitiveLockCallback(std_srvs::Empty::Request& req,
                                                       std_srvs::Empty::Response& res)
  {
    mutex_.lock();
    NODELET_INFO("locked!!");
    return true;
  }

  bool EnvironmentPlaneModeling::primitiveUnlockCallback(std_srvs::Empty::Request& req,
                                                         std_srvs::Empty::Response& res)
  {
    mutex_.unlock();
    NODELET_INFO("unlocked!!");
    return true;
  }
  

  void EnvironmentPlaneModeling::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock(mutex_);
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EnvironmentPlaneModeling running");
    
    stat.add("Time to estimate occlusion (Avg.)",
             occlusion_estimate_time_acc_.mean());
    stat.add("Time to estimate occlusion (Max)",
             occlusion_estimate_time_acc_.max());
    stat.add("Time to estimate occlusion (Min)",
             occlusion_estimate_time_acc_.min());
    stat.add("Time to estimate occlusion (Var.)",
             occlusion_estimate_time_acc_.variance());

    stat.add("Time to build grid (Avg.)",
             grid_building_time_acc_.mean());
    stat.add("Time to build grid (Max)",
             grid_building_time_acc_.max());
    stat.add("Time to build grid (Min)",
             grid_building_time_acc_.min());
    stat.add("Time to build grid (Var.)",
             grid_building_time_acc_.variance());

    stat.add("Time to build kdtree (Avg.)",
             kdtree_building_time_acc_.mean());
    stat.add("Time to build kdtree (Max)",
             kdtree_building_time_acc_.max());
    stat.add("Time to build kdtree (Min)",
             kdtree_building_time_acc_.min());
    stat.add("Time to build kdtree (Var.)",
             kdtree_building_time_acc_.variance());
    
    stat.add("Time to check collision of polygons (Avg.)",
             polygon_collision_check_time_acc_.mean());
    stat.add("Time to check collision of polygons (Max)",
             polygon_collision_check_time_acc_.max());
    stat.add("Time to check collision of polygons (Min)",
             polygon_collision_check_time_acc_.min());
    stat.add("Time to check collision of polygons (Var.)",
             polygon_collision_check_time_acc_.variance());
    
  }
  
  void EnvironmentPlaneModeling::inputCallback(
    const sensor_msgs::PointCloud2::ConstPtr& input,
    const ClusterPointIndices::ConstPtr& input_indices,
    const PolygonArray::ConstPtr& polygons,
    const ModelCoefficientsArray::ConstPtr& coefficients,
    const PolygonArray::ConstPtr& static_polygons,
    const ModelCoefficientsArray::ConstPtr& static_coefficients)
  {
    NODELET_INFO_STREAM(getName() << "::inputCallback");
    boost::mutex::scoped_lock lock(mutex_);
    {
      if (input_indices->cluster_indices.size() == 0) {
        NODELET_WARN("no clusters is available");
        return;
      }
      latest_input_ = input;
      latest_input_indices_ = input_indices;
      latest_input_polygons_ = polygons;
      latest_input_coefficients_ = coefficients;
      latest_static_polygons_ = static_polygons;
      latest_static_coefficients_ = static_coefficients;
    }
    if (continuous_estimation_) {
      // EnvironmentLock::Request req;
      // EnvironmentLock::Response res;
      lockCallback();
    }
    diagnostic_updater_->update();
  }

  void EnvironmentPlaneModeling::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    plane_distance_threshold_ = config.plane_distance_threshold;
    plane_angle_threshold_ = config.plane_angle_threshold;
    distance_thr_ = config.distance_threshold;
    sampling_d_ = config.collision_check_sampling_d;
    if (resolution_size_ != config.resolution_size) {
      NODELET_INFO_STREAM(
        "clearing grid maps because of the change of resolution size: "
        << resolution_size_ << " -> " << config.resolution_size);
      resolution_size_ = config.resolution_size;
      //grid_maps_.clear();
      grid_maps_ = std::vector<GridMap::Ptr>();
    }
    grid_map_angle_threshold_ = config.grid_map_angle_threshold;
    grid_map_distance_threshold_ = config.grid_map_distance_threshold;
    required_vote_ = config.required_vote;
    static_generation_ = config.static_generation;
  }

  void EnvironmentPlaneModeling::updateAppendingInfo(
    const int env_plane_index,
    const size_t static_plane_index,
    std::map<int, std::set<size_t> >& result)
  {
    std::map<int, std::set<size_t> >::iterator it
      = result.find(env_plane_index);
    if (it == result.end()) {
      std::set<size_t> new_set;
      new_set.insert(static_plane_index);
      result[env_plane_index] = new_set;
    }
    else {
      result[env_plane_index].insert(static_plane_index);
    }
  }
    
  void EnvironmentPlaneModeling::extendConvexPolygon(
    const geometry_msgs::PolygonStamped& static_polygon,
    const PCLModelCoefficientMsg& coefficients,
    const geometry_msgs::PolygonStamped& nearest_polygon,
    geometry_msgs::PolygonStamped& output_polygon)
  {
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    pcl::ModelCoefficients plane_coefficients;
    plane_coefficients.values = coefficients.values;
    proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients>(plane_coefficients));
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (size_t j = 0; j < static_polygon.polygon.points.size(); j++) {
      pcl::PointXYZ p;
      pcl_conversions::toPCL(static_polygon.polygon.points[j], p);
      cloud.points.push_back(p);
    }
    for (size_t j = 0; j < nearest_polygon.polygon.points.size(); j++) {
      pcl::PointXYZ p;
      pcl_conversions::toPCL(nearest_polygon.polygon.points[j], p);
      cloud.points.push_back(p);
    }
    pcl::PointCloud<pcl::PointXYZ> projected_cloud;
    proj.setInputCloud(cloud.makeShared());
    proj.filter(projected_cloud);
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(projected_cloud.makeShared());
    chull.setDimension(2);
    pcl::PointCloud<pcl::PointXYZ> chull_output;
    chull.reconstruct(chull_output);
    output_polygon.header = nearest_polygon.header;
    for (size_t j = 0; j < chull_output.points.size(); j++) {
      geometry_msgs::Point32 p;
      pcl_conversions::fromPCL(chull_output.points[j], p);
      output_polygon.polygon.points.push_back(p);
    }
  }

  void EnvironmentPlaneModeling::copyClusterPointIndices(
    const jsk_pcl_ros::ClusterPointIndices::ConstPtr& indices,
    jsk_pcl_ros::ClusterPointIndices& output)
  {
    output.header = indices->header;
    for (size_t i = 0; i < indices->cluster_indices.size(); i++) {
      PCLIndicesMsg index = indices->cluster_indices[i];
      PCLIndicesMsg new_index;
      new_index.header = index.header;
      for (size_t j = 0; j < index.indices.size(); j++) {
        new_index.indices.push_back(index.indices[j]);
      }
      output.cluster_indices.push_back(new_index);
    }
  }

  void EnvironmentPlaneModeling::addIndices(
    const size_t start, const size_t end,
    PCLIndicesMsg& output)
  {
    for (size_t i = start; i < end; i++) {
      output.indices.push_back(i);
    }
  }

  
  void EnvironmentPlaneModeling::computePolygonCentroid(
    const geometry_msgs::PolygonStamped msg,
    pcl::PointXYZRGB& output)
  {
    output.x = output.y = output.z = 0;
    for (size_t i = 0; i < msg.polygon.points.size(); i++) {
      output.x += msg.polygon.points[i].x;
      output.y += msg.polygon.points[i].y;
      output.z += msg.polygon.points[i].z;
    }
    output.x /= msg.polygon.points.size();
    output.y /= msg.polygon.points.size();
    output.z /= msg.polygon.points.size();
  }

  void EnvironmentPlaneModeling::publishGridMap(
    const std_msgs::Header& header,
    const std::vector<GridMap::Ptr> grid_maps)
  {
    SparseOccupancyGridArray grid_array_msg;
    grid_array_msg.header = header;
    for (size_t i = 0; i < grid_maps.size(); i++) {
      GridMap::Ptr grid_map = grid_maps[i];
      SparseOccupancyGrid ros_grid_map;
      ros_grid_map.header = header;
      grid_map->toMsg(ros_grid_map);
      grid_array_msg.grids.push_back(ros_grid_map);
    }
    grid_map_array_pub_.publish(grid_array_msg);
  }
  
  void EnvironmentPlaneModeling::fillEstimatedRegionByPointCloud
  (const std_msgs::Header& header,
   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
   const ClusterPointIndices::ConstPtr& indices,
   const PolygonArray::ConstPtr& polygons,
   const ModelCoefficientsArray::ConstPtr& coefficients,
   const PolygonArray::ConstPtr& static_polygons,
   const ModelCoefficientsArray::ConstPtr& static_coefficients,
   const PolygonArray& result_polygons,
   const std::map<int, std::set<size_t> >& estimation_summary,
   pcl::PointCloud<PointT>::Ptr all_cloud,
   ClusterPointIndices& all_indices,
   std::vector<GridMap::Ptr> grid_maps)
  {
    NODELET_INFO("%lu convexhull will be fulfilled", estimation_summary.size());
    typedef std::map<int, std::set<size_t> >::const_iterator Iterator;
    *all_cloud = *input;
    copyClusterPointIndices(indices, all_indices);
    for (Iterator it = estimation_summary.begin();
         it != estimation_summary.end();
         it++)
    {
      int env_plane_index = it->first;
      std::set<size_t> static_polygon_indices = it->second;
      NODELET_INFO("%d plane is appended by %lu planes", env_plane_index,
                   static_polygon_indices.size());
      // 2cm
      GridMap::Ptr grid = grid_maps[env_plane_index];
      geometry_msgs::PolygonStamped convex_polygon
        = result_polygons.polygons[env_plane_index];
      if (convex_polygon.polygon.points.size() > 0) {
        NODELET_INFO("registering %lu lines", convex_polygon.polygon.points.size() - 1);
      }
      else {
        NODELET_INFO("registering 0 lines");
      }
      std::vector<GridIndex::Ptr> line_indices;
      for (size_t i = 0; i < convex_polygon.polygon.points.size() - 1; i++) {
        geometry_msgs::Point32 from = convex_polygon.polygon.points[i];
        geometry_msgs::Point32 to = convex_polygon.polygon.points[i + 1];
        pcl::PointXYZRGB from_pcl, to_pcl;
        pcl_conversions::toPCL(from, from_pcl);
        pcl_conversions::toPCL(to, to_pcl);
        std::vector<GridIndex::Ptr> aline_indices = grid->registerLine(from_pcl, to_pcl);
        for (size_t j = 0; j < aline_indices.size(); j++) {
          line_indices.push_back(aline_indices[j]);
        }
      }
      // the last one
      {
        geometry_msgs::Point32 from = convex_polygon.polygon.points[convex_polygon.polygon.points.size() - 1];
        geometry_msgs::Point32 to = convex_polygon.polygon.points[0];
        pcl::PointXYZRGB from_pcl, to_pcl;
        pcl_conversions::toPCL(from, from_pcl);
        pcl_conversions::toPCL(to, to_pcl);
        std::vector<GridIndex::Ptr> aline_indices = grid->registerLine(from_pcl, to_pcl);
        for (size_t j = 0; j < aline_indices.size(); j++) {
          line_indices.push_back(aline_indices[j]);
        }
      }
      std::vector<GridIndex::Ptr> filled_indices;
      
      ConvexPolygon convex_polygon_model = ConvexPolygon::fromROSMsg(convex_polygon.polygon);
      for (std::set<size_t>::iterator it = static_polygon_indices.begin();
           it != static_polygon_indices.end();
           it++) {
        size_t before_point_size = filled_indices.size();
        pcl::PointXYZRGB centroid;
        computePolygonCentroid(static_polygons->polygons[*it], centroid);
        // project the point onto the plane
        NODELET_INFO("hello2");
        // before running fillRegion, we should ensure the point will be inside of the
        // convex hull region
        Eigen::Vector3f centroid_eigen = centroid.getVector3fMap();
        Eigen::Vector3f centroid_projected;
        PCLModelCoefficientMsg the_coefficients = static_coefficients->coefficients[*it];
        Eigen::Vector3f normal;
        normal[0] = the_coefficients.values[0];
        normal[1] = the_coefficients.values[1];
        normal[2] = the_coefficients.values[2];
        double d = the_coefficients.values[3] / normal.norm();
        normal.normalize();
        //double alpha = normal.dot(centroid_eigen) - d;
        double alpha = normal.dot(centroid_eigen);
        centroid_projected = centroid_eigen - alpha * normal;
        Eigen::Vector3d centroid_projected_d (centroid_projected[0], centroid_projected[1], centroid_projected[2]);
        if (convex_polygon_model.isInside(centroid_projected_d)) {
          grid->fillRegion(centroid_projected, filled_indices);
          
          NODELET_INFO("%lu static polygon merged into %d env polygon and %lu points is required to fill",
                       *it, env_plane_index,
                       filled_indices.size() - before_point_size);
        }
        else {
          NODELET_ERROR("the centroid point is outside of convex region");
        }
      }
      // remove line region
      for (size_t j = 0; j < line_indices.size(); j++) {
        grid->removeIndex(line_indices[j]);
      }
      NODELET_INFO("add %lu points into %d cluster",
                   filled_indices.size(),
                   env_plane_index);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      grid->indicesToPointCloud(filled_indices, new_cloud);
      size_t before_point_num = all_cloud->points.size();
      *all_cloud = *all_cloud + *new_cloud;
      // update
      size_t after_point_num = all_cloud->points.size();
      addIndices(before_point_num, after_point_num, all_indices.cluster_indices[env_plane_index]);
    }
    // publish the result of concatenation
    sensor_msgs::PointCloud2 ros_output;
    toROSMsg(*all_cloud, ros_output);
    ros_output.header = header;
    occlusion_result_pointcloud_pub_.publish(ros_output);
    occlusion_result_indices_pub_.publish(all_indices);
    occlusion_result_polygons_pub_.publish(result_polygons);
    occlusion_result_coefficients_pub_.publish(coefficients);
  }

  void EnvironmentPlaneModeling::registerGridMap(
    const GridMap::Ptr new_grid_map)
  {
    grid_maps_.push_back(new_grid_map);
  }
  
  // build the grid map for each planes
  void EnvironmentPlaneModeling::buildGridMap(
    const std::vector<pcl::PointCloud<PointT>::Ptr>& segmented_clouds,
    const PolygonArray::ConstPtr& polygons,
    const ModelCoefficientsArray::ConstPtr& coefficients,
    std::vector<GridMap::Ptr>& ordered_grid_maps)
  {
    ScopedTimer timer = grid_building_time_acc_.scopedTimer();
    for (size_t i = 0; i < segmented_clouds.size(); i++) {
      pcl::PointCloud<PointT>::Ptr cloud = segmented_clouds[i];
      // we need to project the point clouds on to the plane
      // because of the noise of the pointclouds.
      int grid_map_index = findCorrespondGridMap(
        coefficients->coefficients[i].values,
        polygons->polygons[i].polygon);
      pcl::ProjectInliers<PointT> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      pcl::ModelCoefficients plane_coefficients;
      plane_coefficients.values = coefficients->coefficients[i].values;
      proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients>(plane_coefficients));
      pcl::PointCloud<PointT>::Ptr projected_cloud (new pcl::PointCloud<PointT>);
      proj.setInputCloud(cloud);
      proj.filter(*projected_cloud);
      if (grid_map_index == -1) {
        GridMap::Ptr grid(new GridMap(resolution_size_,
                                      coefficients->coefficients[i].values));
        grid->setGeneration(generation_);
        grid->registerPointCloud(projected_cloud);
        registerGridMap(grid);
        ordered_grid_maps.push_back(grid);
      }
      else {
        GridMap::Ptr grid = grid_maps_[grid_map_index];
        grid->registerPointCloud(projected_cloud);
        grid->vote();
        ordered_grid_maps.push_back(grid);
      }
      
    }
  }
  
  void EnvironmentPlaneModeling::estimateOcclusion(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
    const ClusterPointIndices::ConstPtr& input_indices,
    const std::vector<pcl::PointCloud<PointT>::Ptr>& segmented_cloud,
    std::vector<GridMap::Ptr>& grid_maps,
    const PolygonArray::ConstPtr& polygons,
    const ModelCoefficientsArray::ConstPtr& coefficients,
    const PolygonArray::ConstPtr& static_polygons,
    const ModelCoefficientsArray::ConstPtr& static_coefficients,
    PolygonArray::Ptr result_polygons,
    ModelCoefficientsArray::Ptr result_coefficients,
    pcl::PointCloud<PointT>::Ptr result_pointcloud,
    ClusterPointIndices::Ptr result_indices)
  {
    ScopedTimer timer = occlusion_estimate_time_acc_.scopedTimer();
    *result_polygons = *polygons; // copied
    *result_coefficients = *coefficients;
    
    std::map<int, std::set<size_t> > appending_map;
    for (size_t i = 0; i < static_polygons->polygons.size(); i++) {
      // looking for the nearest polygon from the static polygon
      geometry_msgs::PolygonStamped static_polygon
        = static_polygons->polygons[i];
      PCLModelCoefficientMsg static_coefficient
        = static_coefficients->coefficients[i];
      int nearest_index = findNearestPolygon(polygons,
                                             coefficients,
                                             static_polygon,
                                             static_coefficient);
      if (nearest_index != -1) {
        // merged into nearest_index
        NODELET_INFO("merging %lu into %d", i, nearest_index);
        geometry_msgs::PolygonStamped nearest_polygon
          = result_polygons->polygons[nearest_index];
        geometry_msgs::PolygonStamped new_polygon;
        
        extendConvexPolygon(static_polygon,
                            coefficients->coefficients[nearest_index],
                            nearest_polygon,
                            new_polygon);
        result_polygons->polygons[nearest_index] = new_polygon;
        
        // add the result into appended_map
        updateAppendingInfo(nearest_index, i, appending_map);
      }
    }
    // publish result_polygons and result_coefficients
    occlusion_result_polygons_pub_.publish(result_polygons);
    occlusion_result_coefficients_pub_.publish(result_coefficients);
    // pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
    // pcl::fromROSMsg(*processing_input_, *pcl_cloud);
    fillEstimatedRegionByPointCloud(processing_input_->header,
                                    input,
                                    input_indices,
                                    polygons,
                                    coefficients,
                                    static_polygons,
                                    static_coefficients,
                                    *result_polygons,
                                    appending_map,
                                    result_pointcloud,
                                    *result_indices,
                                    grid_maps);
  }

  void EnvironmentPlaneModeling::decomposePointCloud(
    const pcl::PointCloud<PointT>::Ptr& input,
    const ClusterPointIndices::ConstPtr& input_indices,
    std::vector<pcl::PointCloud<PointT>::Ptr>& output)
  {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    for (size_t i = 0;
         i < input_indices->cluster_indices.size();
         i++) {
      pcl::PointCloud<PointT>::Ptr segmented_cloud (new pcl::PointCloud<PointT>);
      pcl::PointIndices::Ptr indices (new pcl::PointIndices);
      pcl_conversions::toPCL(input_indices->cluster_indices[i],
                             *indices);
      extract.setIndices(indices);
      extract.filter(*segmented_cloud);
      output.push_back(segmented_cloud);
    }
  }

  int EnvironmentPlaneModeling::findCorrespondGridMap(
    const std::vector<float>& coefficients,
    const geometry_msgs::Polygon& polygon)
  {
    ConvexPolygon::Vertices vertices;
    for (size_t i = 0; i < polygon.points.size(); i++) {
      Eigen::Vector3d v(polygon.points[i].x,
                        polygon.points[i].y,
                        polygon.points[i].z);
      vertices.push_back(v);
    }
    //Plane new_grid_map(coefficients);
    ConvexPolygon new_grid_map(vertices, coefficients);
    Eigen::Vector3d c = new_grid_map.getCentroid();
    int min_index = -1;
    double min_distance = DBL_MAX;
    for (size_t i = 0; i < grid_maps_.size(); i++) {
      Plane grid_plane = grid_maps_[i]->toPlane();
      double angle = grid_plane.angle(new_grid_map);
      std::vector<float> grid_coefficients = grid_maps_[i]->getCoefficients();
      NODELET_INFO("comparing [%f, %f, %f, %f] and [%f, %f, %f, %f]",
                   coefficients[0], coefficients[1], coefficients[2], coefficients[3],
                   grid_coefficients[0], grid_coefficients[1], grid_coefficients[2], grid_coefficients[3]);
      NODELET_INFO("centroid: [%f, %f, %f]",
                   c[0], c[1], c[2]);
      NODELET_INFO_STREAM("  angle: " << angle);
      if (angle < grid_map_angle_threshold_) {
        double distance = grid_plane.distanceToPoint(c);
        NODELET_INFO_STREAM("  distance: " << distance);
        if (distance < grid_map_distance_threshold_) {
          if (min_distance > distance) {
            min_index = i;
            min_distance = distance;
          }
        }
      }
    }
    ROS_INFO_STREAM("min distance is " << min_distance);
    return min_index;
  }
  
  bool EnvironmentPlaneModeling::lockCallback()
  {
    //NODELET_INFO_STREAM(getName() << "::lockCallback");
    //boost::mutex::scoped_lock(mutex_);
    ++generation_;
    
    if (!latest_input_) {
      NODELET_ERROR("[EnvironmentPlaneModeling] no valid input yet");
      return false;
    }
    
    processing_input_ = latest_input_;
    processing_input_indices_ = latest_input_indices_;
    processing_input_polygons_ = latest_input_polygons_;
    processing_input_coefficients_ = latest_input_coefficients_;
    processing_static_polygons_ = latest_static_polygons_;
    processing_static_coefficients_ = latest_static_coefficients_;
    
    NODELET_INFO("lock %lu pointclouds",
                 processing_input_indices_->cluster_indices.size());
    if (processing_input_polygons_->polygons.size()
        != processing_input_coefficients_->coefficients.size()) {
      NODELET_ERROR_STREAM(
        "the size of the input polygon array and model coefficients "
        << "array is not same");
      return false;
    }
    if (processing_static_polygons_->polygons.size()
        != processing_static_coefficients_->coefficients.size()) {
      NODELET_ERROR_STREAM(
        "the size of the input static polygon array and "
        << "static model coefficients array is not same");
      return false;
    }

    pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*processing_input_, *pcl_cloud);
    
    NODELET_INFO("estimating occlusion first");
    // decompose pointcloud into segmented pointcloud
    std::vector<pcl::PointCloud<PointT>::Ptr> segmented_clouds;
    decomposePointCloud(
      pcl_cloud,
      processing_input_indices_,
      segmented_clouds);

    // if you don't need to have history of grid maps,
    // clear them.
    if (!history_accumulation_) { 
      //grid_maps_.clear();
      grid_maps_ = std::vector<GridMap::Ptr>();
    }
    
    selectionGridMaps();
    
    std::vector<GridMap::Ptr> ordered_grid_maps;  
    // first, build grid map
    buildGridMap(segmented_clouds,
                 processing_input_polygons_,
                 processing_input_coefficients_,
                 ordered_grid_maps);
    
    
    PolygonArray::Ptr result_polygons (new PolygonArray);
    ModelCoefficientsArray::Ptr result_coefficients(new ModelCoefficientsArray);
    pcl::PointCloud<PointT>::Ptr result_pointcloud (new pcl::PointCloud<PointT>);
    ClusterPointIndices::Ptr result_indices(new ClusterPointIndices);
    estimateOcclusion(pcl_cloud,
                      processing_input_indices_,
                      segmented_clouds,
                      ordered_grid_maps,
                      processing_input_polygons_,
                      processing_input_coefficients_,
                      processing_static_polygons_,
                      processing_static_coefficients_,
                      result_polygons,
                      result_coefficients,
                      result_pointcloud,
                      result_indices);
    publishGridMap(processing_input_->header, grid_maps_);
    if (result_indices->cluster_indices.size() == 0) {
      NODELET_WARN("failed to build gridmap?, result_indices equals to 0");
      return false;
    }
    {
      ScopedTimer timer = kdtree_building_time_acc_.scopedTimer();
      // build kdtrees
      kdtrees_.clear();
      separated_point_cloud_.clear();
    
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(result_pointcloud);
      for (size_t i = 0;
           i < result_indices->cluster_indices.size();
           i++) {
        pcl::PointCloud<PointT>::Ptr nonprojected_input (new pcl::PointCloud<PointT>);
        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        pcl_conversions::toPCL(result_indices->cluster_indices[i],
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
      //res.environment_id = ++environment_id_;
    }
    
    NODELET_INFO_STREAM("grid maps: " << grid_maps_.size());
    for (size_t i = 0; i < grid_maps_.size(); i++) {
      GridMap::Ptr grid = grid_maps_[i];
      std::vector<float> coefficients = grid->getCoefficients();
      NODELET_INFO_STREAM("  " << i << ": " <<
                          coefficients[0] << ", " <<
                          coefficients[1] << ", " <<
                          coefficients[2] << ", " <<
                          coefficients[3] <<
                          " ( " << grid->getVoteNum() << ")");
    }
    return true;
  }

  bool EnvironmentPlaneModeling::polygonNearEnoughToPointCloud(
    const size_t plane_i,
    const pcl::PointCloud<PointT>::Ptr sampled_point_cloud)
  {
    // geometry_msgs::PolygonStamped target_polygon
    //   = processing_input_polygons_->polygons[plane_i];
    // sensor_msgs::PointCloud2 debug_env_pointcloud;
    GridMap::Ptr grid = grid_maps_[plane_i];
    Plane::Ptr grid_plane = grid->toPlanePtr();
    for (size_t i = 0; i < sampled_point_cloud->points.size(); i++) {
      PointT p = sampled_point_cloud->points[i];
      Eigen::Vector3d p_eigen;
      pcl_conversions::fromPCLToEigen(p, p_eigen);
      double d = grid_plane->distanceToPoint(p_eigen);
      if (d < resolution_size_) {
        //NODELET_INFO_STREAM("distance: " << d);
        Eigen::Vector3d p_eigen_projected;
        grid_plane->project(p_eigen, p_eigen_projected);
        Eigen::Vector3f p_eigen_projected_f(p_eigen_projected[0], p_eigen_projected[1], p_eigen_projected[2]);
        if (!grid->isBinsOccupied(p_eigen_projected_f)) {
          return false;            // near enough!! hopefully...
        }
      }
      else {
        return false;
      }
    }
    return true;
  }

  // bool EnvironmentPlaneModeling::polygonNearEnoughToPointCloud(
  //   const size_t plane_i,
  //   const pcl::PointCloud<PointT>::Ptr sampled_point_cloud)
  // {
  //   if (plane_i >= kdtrees_.size()) {
  //     NODELET_WARN("the number of kdtrees is smaller than the index of the plane");
  //     return false;
  //   }
  //   geometry_msgs::PolygonStamped target_polygon
  //     = processing_input_polygons_->polygons[plane_i];
  //   // debug debugrmation
  //   debug_env_polygon_pub_.publish(target_polygon);
  //   sensor_msgs::PointCloud2 debug_env_pointcloud;
  //   toROSMsg(*separated_point_cloud_[plane_i], debug_env_pointcloud);
  //   debug_env_pointcloud.header = processing_input_->header;
  //   debug_env_pointcloud_pub_.publish(debug_env_pointcloud);
  //   NODELET_INFO_STREAM("plane_i: " << plane_i);
  //   NODELET_INFO_STREAM("kdtrees_.size: " << kdtrees_.size());
  //   if (plane_i >= kdtrees_.size()) {
  //     NODELET_WARN("the number of kdtrees is smaller than the index of the plane2");
  //     return false;
  //   }
  //   // check collision
  //   // all the sampled points should near enough from target_polygon
  //   pcl::KdTreeFLANN<PointT>::Ptr target_kdtree = kdtrees_[plane_i];
  //   // NODELET_INFO("checking %lu points", target_kdtree->getInputCloud()->points.size());
  //   for (size_t i = 0; i < sampled_point_cloud->points.size(); i++) {
  //     PointT p = sampled_point_cloud->points[i];
  //     std::vector<int> k_indices;
  //     std::vector<float> k_sqr_distances;
  //     if (target_kdtree->radiusSearch(p,
  //                                     distance_thr_,
  //                                     k_indices,
  //                                     k_sqr_distances, 1) == 0) {
  //       return false;
  //     }
  //   }
  //   return true;
  // }

  void EnvironmentPlaneModeling::selectionGridMaps()
  {
        // clean up too minor grids
    if (history_statical_rejection_) {
      for (std::vector<GridMap::Ptr>::iterator it = grid_maps_.begin();
           it != grid_maps_.end();) {
        GridMap::Ptr map = *it;
        if ((generation_ - map->getGeneration()) > static_generation_) {
          if (map->getVoteNum() < required_vote_) { // minimum 5
            it = grid_maps_.erase(it);
          }
          else {
            it++;
          }
        }
        else {
          it++;
        }
      }
    }
  }
  
  int EnvironmentPlaneModeling::findNearestPolygon(
    const PolygonArray::ConstPtr& polygons,
    const ModelCoefficientsArray::ConstPtr& coefficients,
    const geometry_msgs::PolygonStamped& static_polygon,
    const PCLModelCoefficientMsg& static_coefficient)
  {

    int nearest_index = -1;
    double min_angle_distance = DBL_MAX;
    for (size_t j = 0; j < polygons->polygons.size(); j++) {
      geometry_msgs::PolygonStamped candidate_polygon = polygons->polygons[j];
      Plane a(coefficients->coefficients[j].values);
      Plane b(static_coefficient.values);
      if (!a.isSameDirection(b)) {
        b = b.flip();
      }

      if (a.distance(b) > plane_distance_threshold_) {
        continue;
      }
      double theta = a.angle(b);
      if (theta > M_PI / 2.0) {
        theta = M_PI  - theta;
      }
      if (theta > plane_angle_threshold_) {
        continue;
      }
      if (min_angle_distance > theta) {
        min_angle_distance = theta;
        nearest_index = j;
      }
    }
    return nearest_index;
  }
  
  bool EnvironmentPlaneModeling::polygonOnEnvironmentCallback(
    PolygonOnEnvironment::Request& req,
    PolygonOnEnvironment::Response& res)
  {
    //boost::mutex::scoped_lock(mutex_);
    if (req.environment_id != environment_id_ && req.environment_id != 0) { // 0 is always OK
      NODELET_FATAL("environment id does not match. %u is provided but the environment stored is %u",
                    req.environment_id,
                    environment_id_);
      return false;
    }
    ScopedTimer timer = polygon_collision_check_time_acc_.scopedTimer();
    pcl::PointCloud<PointT>::Ptr sampled_point_cloud (new pcl::PointCloud<PointT>());
    samplePolygonToPointCloud(req.polygon, sampled_point_cloud, sampling_d_);

    // sensor_msgs::PointCloud2 debug_sampled_pointcloud;
    // toROSMsg(*sampled_point_cloud, debug_sampled_pointcloud);
    // debug_sampled_pointcloud.header = processing_input_->header;
    // debug_pointcloud_pub_.publish(debug_sampled_pointcloud);

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
    if (found_contact_plane) {
      //NODELET_INFO("on plane");
      res.result = true;
      return true;
    }
    else {
      //NODELET_INFO("not on plane");
      res.result = false;
      res.reason = "the polygon is not on any plane";
      return true;
    }
    
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
      pcl_conversions::toPCL(point, pcl_point);
      output->points.push_back(pcl_point);
    }
  }
  
}

typedef jsk_pcl_ros::EnvironmentPlaneModeling EnvironmentPlaneModeling;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, EnvironmentPlaneModeling, EnvironmentPlaneModeling, nodelet::Nodelet);
