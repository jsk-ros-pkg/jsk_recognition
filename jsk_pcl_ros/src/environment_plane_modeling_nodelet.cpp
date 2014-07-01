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
    diagnostic_updater_.reset(new diagnostic_updater::Updater);
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add("Modeling Stats", boost::bind(&EnvironmentPlaneModeling::updateDiagnostic,
                                                           this,
                                                           _1));
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
      = pnh_->advertiseService("lock", &EnvironmentPlaneModeling::lockCallback,
                               this);
    polygon_on_environment_service_
      = pnh_->advertiseService("polygon_on_environment",
                               &EnvironmentPlaneModeling::polygonOnEnvironmentCallback, this);
  }

  void EnvironmentPlaneModeling::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock(mutex_);
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EnvironmentPlaneModeling running");
    
    stat.add("Time to estimate occlusion (Avg.)",
             boost::accumulators::mean(occlusion_estimate_time_acc_));
    stat.add("Time to estimate occlusion (Max)",
             boost::accumulators::max(occlusion_estimate_time_acc_));
    stat.add("Time to estimate occlusion (Min)",
             boost::accumulators::min(occlusion_estimate_time_acc_));
    stat.add("Time to estimate occlusion (Var.)",
             boost::accumulators::variance(occlusion_estimate_time_acc_));

    stat.add("Time to build grid (Avg.)",
             boost::accumulators::mean(grid_building_time_acc_));
    stat.add("Time to build grid (Max)",
             boost::accumulators::max(grid_building_time_acc_));
    stat.add("Time to build grid (Min)",
             boost::accumulators::min(grid_building_time_acc_));
    stat.add("Time to build grid (Var.)",
             boost::accumulators::variance(grid_building_time_acc_));
  }
  
  void EnvironmentPlaneModeling::inputCallback(
    const sensor_msgs::PointCloud2::ConstPtr& input,
    const ClusterPointIndices::ConstPtr& input_indices,
    const PolygonArray::ConstPtr& polygons,
    const ModelCoefficientsArray::ConstPtr& coefficients,
    const PolygonArray::ConstPtr& static_polygons,
    const ModelCoefficientsArray::ConstPtr& static_coefficients)
  {
    NODELET_INFO("hello world");
    {
      boost::mutex::scoped_lock(mutex_);
      latest_input_ = input;
      latest_input_indices_ = input_indices;
      latest_input_polygons_ = polygons;
      latest_input_coefficients_ = coefficients;
      latest_static_polygons_ = static_polygons;
      latest_static_coefficients_ = static_coefficients;
    }
    if (continuous_estimation_) {
      EnvironmentLock::Request req;
      EnvironmentLock::Response res;
      lockCallback(req, res);
    }
    diagnostic_updater_->update();
  }

  void EnvironmentPlaneModeling::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    plane_distance_threshold_ = config.plane_distance_threshold;
    plane_angle_threshold_ = config.plane_angle_threshold;
    distance_thr_ = config.distance_threshold;
    sampling_d_ = config.collision_check_sampling_d;
    resolution_size_ = config.resolution_size;
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
   ClusterPointIndices& all_indices)
  {
    NODELET_DEBUG("%lu convexhull will be fulfilled", estimation_summary.size());
    typedef std::map<int, std::set<size_t> >::const_iterator Iterator;
    *all_cloud = *input;
    copyClusterPointIndices(indices, all_indices);
    ros::Time bot = ros::Time::now();
    SparseOccupancyGridArray grid_array_msg;
    grid_array_msg.header = header;
    for (Iterator it = estimation_summary.begin();
         it != estimation_summary.end();
         it++)
    {
      int env_plane_index = it->first;
      std::set<size_t> static_polygon_indices = it->second;
      NODELET_DEBUG("%d plane is appended by %lu planes", env_plane_index,
                   static_polygon_indices.size());
      // 2cm
      GridMap grid(resolution_size_, coefficients->coefficients[env_plane_index].values);
      grid.registerPointCloud(input);
      geometry_msgs::PolygonStamped convex_polygon
        = result_polygons.polygons[env_plane_index];
      for (size_t i = 0; i < convex_polygon.polygon.points.size() - 1; i++) {
        geometry_msgs::Point32 from = convex_polygon.polygon.points[i];
        geometry_msgs::Point32 to = convex_polygon.polygon.points[i + 1];
        pcl::PointXYZRGB from_pcl, to_pcl;
        pcl_conversions::toPCL(from, from_pcl);
        pcl_conversions::toPCL(to, to_pcl);
        grid.registerLine(from_pcl, to_pcl);
      }

      std::vector<GridIndex::Ptr> filled_indices;
      
      for (std::set<size_t>::iterator it = static_polygon_indices.begin();
           it != static_polygon_indices.end();
           it++) {
        size_t before_point_size = filled_indices.size();
        pcl::PointXYZRGB centroid;
        computePolygonCentroid(static_polygons->polygons[*it], centroid);
        grid.fillRegion(centroid.getVector3fMap(), filled_indices);
        NODELET_DEBUG("%lu static polygon merged into %d env polygon and %lu points is required to fill",
                     *it, env_plane_index,
                     filled_indices.size() - before_point_size);
      }
      
      NODELET_DEBUG("add %lu points into %d cluster",
                   filled_indices.size(),
                   env_plane_index);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      grid.indicesToPointCloud(filled_indices, new_cloud);
      size_t before_point_num = all_cloud->points.size();
      *all_cloud = *all_cloud + *new_cloud;
      // update
      size_t after_point_num = all_cloud->points.size();
      addIndices(before_point_num, after_point_num, all_indices.cluster_indices[env_plane_index]);
      SparseOccupancyGrid grid_msg;
      NODELET_INFO("hogeeeee");
      grid_msg.header = header;
      grid.toMsg(grid_msg);
      grid_array_msg.grids.push_back(grid_msg);
    }
    grid_map_array_pub_.publish(grid_array_msg);
    ros::Time eot = ros::Time::now();
    grid_building_time_acc_((eot - bot).toSec());
    // publish the result of concatenation
    sensor_msgs::PointCloud2 ros_output;
    toROSMsg(*all_cloud, ros_output);
    ros_output.header = header;
    occlusion_result_pointcloud_pub_.publish(ros_output);
    occlusion_result_indices_pub_.publish(all_indices);
    occlusion_result_polygons_pub_.publish(result_polygons);
    occlusion_result_coefficients_pub_.publish(coefficients);
  }
  
  void EnvironmentPlaneModeling::estimateOcclusion(
      const sensor_msgs::PointCloud2::ConstPtr& input,
      const ClusterPointIndices::ConstPtr& input_indices,
      const PolygonArray::ConstPtr& polygons,
      const ModelCoefficientsArray::ConstPtr& coefficients,
      const PolygonArray::ConstPtr& static_polygons,
      const ModelCoefficientsArray::ConstPtr& static_coefficients,
      PolygonArray::Ptr result_polygons,
      ModelCoefficientsArray::Ptr result_coefficients,
      pcl::PointCloud<PointT>::Ptr result_pointcloud,
      ClusterPointIndices::Ptr result_indices)
  {
    *result_polygons = *polygons;
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
        NODELET_DEBUG("merging %lu into %d", i, nearest_index);
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
    pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*processing_input_, *pcl_cloud);
    fillEstimatedRegionByPointCloud(processing_input_->header,
                                    pcl_cloud,
                                    input_indices,
                                    polygons,
                                    coefficients,
                                    static_polygons,
                                    static_coefficients,
                                    *result_polygons,
                                    appending_map,
                                    result_pointcloud,
                                    *result_indices);
  }

  
  bool EnvironmentPlaneModeling::lockCallback(
    EnvironmentLock::Request& req,
    EnvironmentLock::Response& res)
  {
    boost::mutex::scoped_lock(mutex_);
    
    if (!latest_input_) {
      NODELET_ERROR("[EnvironmentPlaneModeling] no valid input yet");
      return false;
    }
    ros::Time bot = ros::Time::now();
    processing_input_ = latest_input_;
    processing_input_indices_ = latest_input_indices_;
    processing_input_polygons_ = latest_input_polygons_;
    processing_input_coefficients_ = latest_input_coefficients_;
    processing_static_polygons_ = latest_static_polygons_;
    processing_static_coefficients_ = latest_static_coefficients_;
    
    NODELET_DEBUG("lock %lu pointclouds",
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
    
    NODELET_DEBUG("estimating occlusion first");
    PolygonArray::Ptr result_polygons (new PolygonArray);
    ModelCoefficientsArray::Ptr result_coefficients(new ModelCoefficientsArray);
    pcl::PointCloud<PointT>::Ptr result_pointcloud (new pcl::PointCloud<PointT>);
    ClusterPointIndices::Ptr result_indices(new ClusterPointIndices);
    estimateOcclusion(processing_input_,
                      processing_input_indices_,
                      processing_input_polygons_,
                      processing_input_coefficients_,
                      processing_static_polygons_,
                      processing_static_coefficients_,
                      result_polygons,
                      result_coefficients,
                      result_pointcloud,
                      result_indices);
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
    res.environment_id = ++environment_id_;

    ros::Time eot = ros::Time::now();

    occlusion_estimate_time_acc_((bot - eot).toSec());
    
    return true;
  }

  bool EnvironmentPlaneModeling::polygonNearEnoughToPointCloud(
    const size_t plane_i,
    const pcl::PointCloud<PointT>::Ptr sampled_point_cloud)
  {
    geometry_msgs::PolygonStamped target_polygon
      = processing_input_polygons_->polygons[plane_i];
    // debug debugrmation
    debug_env_polygon_pub_.publish(target_polygon);
    sensor_msgs::PointCloud2 debug_env_pointcloud;
    toROSMsg(*separated_point_cloud_[plane_i], debug_env_pointcloud);
    debug_env_pointcloud.header = processing_input_->header;
    debug_env_pointcloud_pub_.publish(debug_env_pointcloud);
      
    // check collision
    // all the sampled points should near enough from target_polygon
    pcl::KdTreeFLANN<PointT>::Ptr target_kdtree = kdtrees_[plane_i];
    // NODELET_DEBUG("checking %lu points", target_kdtree->getInputCloud()->points.size());
    for (size_t i = 0; i < sampled_point_cloud->points.size(); i++) {
      PointT p = sampled_point_cloud->points[i];
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      if (target_kdtree->radiusSearch(p,
                                      distance_thr_,
                                      k_indices,
                                      k_sqr_distances, 1) == 0) {
        return false;
      }
    }
    return true;
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
    if (req.environment_id != environment_id_ && req.environment_id != 0) { // 0 is always OK
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
    NODELET_DEBUG("kdtree took %f sec", (after - before).toSec());
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
      //NODELET_DEBUG("sampled %d points", sampling_num);
      geometry_msgs::Point32 point = sample_polygon.polygon.points[i];
      PointT pcl_point;
      pcl_conversions::toPCL(point, pcl_point);
      output->points.push_back(pcl_point);
    }
  }
  
}

typedef jsk_pcl_ros::EnvironmentPlaneModeling EnvironmentPlaneModeling;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, EnvironmentPlaneModeling, EnvironmentPlaneModeling, nodelet::Nodelet);
