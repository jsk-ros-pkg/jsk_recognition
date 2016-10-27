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

#include "jsk_pcl_ros/environment_plane_modeling.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <jsk_recognition_msgs/SparseOccupancyGridArray.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseArray.h>
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_pcl_ros/grid_map.h"
#include <jsk_topic_tools/rosparam_utils.h>
namespace jsk_pcl_ros
{
  void EnvironmentPlaneModeling::onInit()
  {
    DiagnosticNodelet::onInit();
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EnvironmentPlaneModeling::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pnh_->param("complete_footprint_region", complete_footprint_region_, false);
    pub_debug_magnified_polygons_
      = pnh_->advertise<jsk_recognition_msgs::PolygonArray>(
        "debug/magnified_polygons", 1);
    pub_debug_convex_point_cloud_
      = pnh_->advertise<sensor_msgs::PointCloud2>(
        "debug/convex_cloud", 1);
    pub_debug_raw_grid_map_
      = pnh_->advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>(
        "debug/raw_grid_map", 1);
    pub_debug_noeroded_grid_map_
      = pnh_->advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>(
        "debug/noeroded_grid_map", 1);
    pub_debug_plane_coords_
      = pnh_->advertise<geometry_msgs::PoseArray>(
        "debug/plane_poses", 1);
    pub_debug_magnified_plane_coords_
      = pnh_->advertise<geometry_msgs::PoseArray>(
        "debug/magnified_plane_poses", 1);
    pub_grid_map_
      = pnh_->advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>(
        "output", 1, true);
    pub_snapped_move_base_simple_goal_ = pnh_->advertise<geometry_msgs::PoseStamped>(
      "/footstep_simple/goal", 1);
    pub_non_plane_indices_ = pnh_->advertise<PCLIndicesMsg>(
      "output/non_plane_indices", 1);
    if (complete_footprint_region_) {
      tf_listener_ = TfListenerSingleton::getInstance();
          
      sub_leg_bbox_ = pnh_->subscribe(
        "input/leg_bounding_box", 1,
        &EnvironmentPlaneModeling::boundingBoxCallback, this);
      
      jsk_topic_tools::readVectorParameter(
        *pnh_, "footprint_frames", footprint_frames_);
      
    }
    sub_move_base_simple_goal_ = pnh_->subscribe(
      "/move_base_simple/goal", 1,
      &EnvironmentPlaneModeling::moveBaseSimpleGoalCallback, this);
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_full_cloud_.subscribe(*pnh_, "input/full_cloud", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_polygons_.subscribe(*pnh_, "input/polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_full_cloud_, sub_polygons_,
                        sub_coefficients_, sub_indices_);
    sync_->registerCallback(
      boost::bind(&EnvironmentPlaneModeling::inputCallback,
                  this, _1, _2, _3, _4, _5));

    onInitPostProcess();
  }

  void EnvironmentPlaneModeling::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    magnify_distance_ = config.magnify_distance;
    distance_threshold_ = config.distance_threshold;
    normal_threshold_ = config.normal_threshold;
    resolution_ = config.resolution;
    morphological_filter_size_ = config.morphological_filter_size;
    erode_filter_size_ = config.erode_filter_size;
    footprint_plane_angular_threshold_ = config.footprint_plane_angular_threshold;
    footprint_plane_distance_threshold_ = config.footprint_plane_distance_threshold;
  }

  void EnvironmentPlaneModeling::printInputData(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& full_cloud_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    NODELET_INFO("Input data --");
    NODELET_INFO("  Number of points -- %d", cloud_msg->width * cloud_msg->height);
    NODELET_INFO("  Number of full points -- %d", full_cloud_msg->width * full_cloud_msg->height);
    NODELET_INFO("  Number of clusters: -- %lu", indices_msg->cluster_indices.size());
    NODELET_INFO("  Frame Id: %s", cloud_msg->header.frame_id.c_str());
    NODELET_INFO("  Complete Footprint: %s", complete_footprint_region_? "true": "false");
  } 

  bool EnvironmentPlaneModeling::isValidFrameIds(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& full_cloud_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    std::string frame_id = cloud_msg->header.frame_id;
    if (full_cloud_msg->header.frame_id != frame_id) {
      return false;
    }
    if (polygon_msg->header.frame_id != frame_id) {
      return false;
    }
    for (size_t i = 0; i < polygon_msg->polygons.size(); i++) {
      if (polygon_msg->polygons[i].header.frame_id != frame_id) {
        return false;
      }
    }
    if (coefficients_msg->header.frame_id != frame_id) {
      return false;
    }
    for (size_t i = 0; i < coefficients_msg->coefficients.size(); i++) {
      if (coefficients_msg->coefficients[i].header.frame_id != frame_id) {
        return false;
      }
    }
    if (indices_msg->header.frame_id != frame_id) {
      return false;
    }
    for (size_t i = 0; i < indices_msg->cluster_indices.size(); i++) {
      if (indices_msg->cluster_indices[i].header.frame_id != frame_id) {
        return false;
      }
    }
    return true;
  }

  void EnvironmentPlaneModeling::moveBaseSimpleGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (latest_grid_maps_.size() == 0) {
      NODELET_WARN("not yet grid maps are available");
      return;
    }

    tf::StampedTransform tf_transform = lookupTransformWithDuration(
      tf_listener_,
      latest_global_header_.frame_id,
      msg->header.frame_id,
      latest_global_header_.stamp,
      ros::Duration(1.0));
    
    Eigen::Affine3f local_coords, transform;
    tf::poseMsgToEigen(msg->pose, local_coords);
    tf::transformTFToEigen(tf_transform, transform);
    Eigen::Affine3f global_coords = transform * local_coords;

    // lookup suitable grid
    double max_height = - DBL_MAX;
    GridPlane::Ptr max_grid;
    Eigen::Affine3f max_projected_coords = Eigen::Affine3f::Identity();
    for (size_t i = 0; i < latest_grid_maps_.size(); i++) {
      GridPlane::Ptr target_grid = latest_grid_maps_[i];
      Eigen::Affine3f projected_coords;
      target_grid->getPolygon()->projectOnPlane(global_coords, projected_coords);
      Eigen::Vector3f projected_point(projected_coords.translation());
      if (target_grid->isOccupiedGlobal(projected_point)) {
        double height = projected_point[2];
        if (max_height < height) {
          max_height = height;
          max_grid = target_grid;
          max_projected_coords = projected_coords;
        }
      }
    }
    if (max_grid) {
      // publish it
      geometry_msgs::PoseStamped ros_coords;
      tf::poseEigenToMsg(max_projected_coords, ros_coords.pose);
      ros_coords.header.stamp = msg->header.stamp;
      ros_coords.header.frame_id = latest_global_header_.frame_id;
      pub_snapped_move_base_simple_goal_.publish(ros_coords);
    }
    else {
      NODELET_ERROR("Failed to find corresponding grid");
    }
  }
  
  void EnvironmentPlaneModeling::boundingBoxCallback(
    const jsk_recognition_msgs::BoundingBox::ConstPtr& box)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_leg_bounding_box_ = box;
  }
  
  void EnvironmentPlaneModeling::inputCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& full_cloud_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    try {
      // check frame_id
      if (!isValidFrameIds(cloud_msg, full_cloud_msg, polygon_msg, coefficients_msg, indices_msg)) {
        NODELET_FATAL("frame_id is not correct");
        return;
      }
      if (complete_footprint_region_ && !latest_leg_bounding_box_) {
        NODELET_ERROR("Bounding Box for Footprint is not yet ready");
        return;
      }
      // first, print all the information about ~inputs
      printInputData(cloud_msg, full_cloud_msg, polygon_msg, coefficients_msg, indices_msg);

    
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
      pcl::fromROSMsg(*cloud_msg, *cloud);

      pcl::PointCloud<pcl::PointNormal>::Ptr full_cloud (new pcl::PointCloud<pcl::PointNormal>);
      pcl::fromROSMsg(*full_cloud_msg, *full_cloud);
    
      // convert to jsk_recognition_utils::ConvexPolygon
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes = convertToConvexPolygons(cloud, indices_msg, coefficients_msg);
      // magnify convexes
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> magnified_convexes = magnifyConvexes(convexes);
      publishConvexPolygonsBoundaries(pub_debug_convex_point_cloud_, cloud_msg->header, magnified_convexes);
      // Publish magnified convexes for debug
      publishConvexPolygons(pub_debug_magnified_polygons_, cloud_msg->header, magnified_convexes);
      // publish pose_array for debug
      {
        geometry_msgs::PoseArray pose_array;
        pose_array.header = cloud_msg->header;
        for (size_t i = 0; i < convexes.size(); i++) {
          Eigen::Affine3f pose = convexes[i]->coordinates();
          geometry_msgs::Pose ros_pose;
          tf::poseEigenToMsg(pose, ros_pose);
          pose_array.poses.push_back(ros_pose);
        }
        pub_debug_plane_coords_.publish(pose_array);
      }
      {
        geometry_msgs::PoseArray pose_array;
        pose_array.header = cloud_msg->header;
        for (size_t i = 0; i < magnified_convexes.size(); i++) {
          Eigen::Affine3f pose = magnified_convexes[i]->coordinates();
          geometry_msgs::Pose ros_pose;
          tf::poseEigenToMsg(pose, ros_pose);
          pose_array.poses.push_back(ros_pose);
        }
        pub_debug_magnified_plane_coords_.publish(pose_array);
      }
      
      // build GridMaps
      std::set<int> non_plane_indices;
      std::vector<GridPlane::Ptr> raw_grid_planes = buildGridPlanes(full_cloud, magnified_convexes, non_plane_indices);
      
      publishGridMaps(pub_debug_raw_grid_map_, cloud_msg->header, raw_grid_planes);
      PCLIndicesMsg ros_non_plane_indices;
      ros_non_plane_indices.indices = std::vector<int>(non_plane_indices.begin(),
                                                       non_plane_indices.end());
      ros_non_plane_indices.header = cloud_msg->header;
      pub_non_plane_indices_.publish(ros_non_plane_indices);
      std::vector<GridPlane::Ptr> morphological_filtered_grid_planes
        = morphologicalFiltering(raw_grid_planes);
      
      publishGridMaps(pub_debug_noeroded_grid_map_, cloud_msg->header,
                      morphological_filtered_grid_planes);
      
      std::vector<GridPlane::Ptr> eroded_grid_planes
        = erodeFiltering(morphological_filtered_grid_planes);
      std::vector<GridPlane::Ptr> result_grid_planes;
      
      if (complete_footprint_region_) { // complete footprint region if needed
        result_grid_planes
          = completeFootprintRegion(cloud_msg->header,
                                    eroded_grid_planes);
      }
      else {
        result_grid_planes = eroded_grid_planes;
     }

      publishGridMaps(pub_grid_map_, cloud_msg->header,
                      result_grid_planes);
      
      latest_global_header_ = cloud_msg->header;
      latest_grid_maps_ = result_grid_planes;
    }
    catch (tf2::TransformException& e) {
      NODELET_ERROR("Failed to lookup transformation: %s", e.what());
    }
  }

  std::vector<GridPlane::Ptr> EnvironmentPlaneModeling::erodeFiltering(
      std::vector<GridPlane::Ptr>& grid_maps)
  {
    std::vector<GridPlane::Ptr> ret;
    for (size_t i = 0; i < grid_maps.size(); i++) {
      GridPlane::Ptr eroded_grid_map = grid_maps[i]->erode(erode_filter_size_);
      ret.push_back(eroded_grid_map);
    }
    return ret;
  }
  
  int EnvironmentPlaneModeling::lookupGroundPlaneForFootprint(
    const Eigen::Affine3f& pose, const std::vector<GridPlane::Ptr>& grid_maps)
  {
    Eigen::Vector3f foot_z = (pose.rotation() * Eigen::Vector3f::UnitZ()).normalized();
    Eigen::Vector3f foot_p(pose.translation());
    double min_distance = DBL_MAX;
    int min_index = -1;
    for (size_t i = 0; i < grid_maps.size(); i++) {
      GridPlane::Ptr grid = grid_maps[i];
      Eigen::Vector3f normal = grid->getPolygon()->getNormal();
      if (std::abs(normal.dot(foot_z)) > cos(footprint_plane_angular_threshold_)) {
        // compare distance
        if (grid->getPolygon()->distanceToPoint(foot_p) < footprint_plane_distance_threshold_) {
          Eigen::Vector3f foot_center(pose.translation());
          if (!grid->isOccupiedGlobal(foot_center)) {
            // check distance to point
            double d = grid->getPolygon()->distanceFromVertices(foot_center);
            if (d < min_distance) {
              min_distance = d;
              min_index = i;
            }
          }
          else {
            NODELET_INFO("Foot print is already occupied");
            return -1;
          }
          // NB: else break?
        }
      }
    }
    return min_index;
  }

  int EnvironmentPlaneModeling::lookupGroundPlaneForFootprint(
    const std::string& footprint_frame, const std_msgs::Header& header,
    const std::vector<GridPlane::Ptr>& grid_maps)
  {
    // first, lookup location of frames
    tf::StampedTransform transform
      = lookupTransformWithDuration(tf_listener_,
                                    header.frame_id, footprint_frame,
                                    header.stamp,
                                    ros::Duration(1.0));
    Eigen::Affine3f eigen_transform;
    tf::transformTFToEigen(transform, eigen_transform);
    // lookup ground plane for the foot
    return lookupGroundPlaneForFootprint(eigen_transform, grid_maps);
  }

  GridPlane::Ptr EnvironmentPlaneModeling::completeGridMapByBoundingBox(
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box,
      const std_msgs::Header& header,
      GridPlane::Ptr grid_map)
  {
    // resolve tf
    tf::StampedTransform tf_transform = lookupTransformWithDuration(
      tf_listener_,
      header.frame_id,
      box->header.frame_id,
      header.stamp,
      ros::Duration(1.0));
    Eigen::Affine3f transform;
    tf::transformTFToEigen(tf_transform, transform);
    Eigen::Affine3f local_pose;
    tf::poseMsgToEigen(box->pose, local_pose);
    Eigen::Affine3f global_pose = transform * local_pose;
    std::vector<double> dimensions;
    dimensions.push_back(box->dimensions.x);
    dimensions.push_back(box->dimensions.y);
    dimensions.push_back(box->dimensions.z);
    jsk_recognition_utils::Cube::Ptr cube (new Cube(Eigen::Vector3f(global_pose.translation()),
                             Eigen::Quaternionf(global_pose.rotation()),
                             dimensions));
    GridPlane::Ptr completed_grid_map = grid_map->clone();
    completed_grid_map->fillCellsFromCube(*cube);
    return completed_grid_map;
  }
  
  std::vector<GridPlane::Ptr> EnvironmentPlaneModeling::completeFootprintRegion(
    const std_msgs::Header& header, std::vector<GridPlane::Ptr>& grid_maps)
  {
    try {
      std::vector<GridPlane::Ptr> completed_grid_maps(grid_maps.size());
      std::set<int> ground_plane_indices;
      for (size_t i = 0; i < footprint_frames_.size(); i++) {
        std::string footprint_frame = footprint_frames_[i];
        int grid_index = lookupGroundPlaneForFootprint(
          footprint_frame, header, grid_maps);
        if (grid_index != -1) {
          NODELET_INFO("Found ground plane for %s: %d", footprint_frame.c_str(), grid_index);
          ground_plane_indices.insert(grid_index);
        }
        else {
          NODELET_WARN("Cannnot find ground plane for %s: %d", footprint_frame.c_str(), grid_index);
        }
      }
      for (size_t i = 0; i < grid_maps.size(); i++) {
        if (ground_plane_indices.find(i) == ground_plane_indices.end()) {
          // It's not a ground plane, just copy the original
          completed_grid_maps[i] = grid_maps[i];
        }
        else {
          completed_grid_maps[i] = completeGridMapByBoundingBox(
            latest_leg_bounding_box_, header, grid_maps[i]);
        }
      }
      return completed_grid_maps;
    }
    catch (tf2::TransformException& e) {
      NODELET_FATAL("Failed to lookup transformation: %s", e.what());
      return std::vector<GridPlane::Ptr>();
    }
  }
  
  std::vector<GridPlane::Ptr> EnvironmentPlaneModeling::morphologicalFiltering(
    std::vector<GridPlane::Ptr>& raw_grid_maps)
  {
    std::vector<GridPlane::Ptr> ret;
    for (size_t i = 0; i < raw_grid_maps.size(); i++) {
      GridPlane::Ptr dilated_grid_map = raw_grid_maps[i]->dilate(morphological_filter_size_);
      GridPlane::Ptr eroded_grid_map = dilated_grid_map->erode(morphological_filter_size_);
      ret.push_back(eroded_grid_map);
    }
    return ret;
  }

  
  void EnvironmentPlaneModeling::publishConvexPolygonsBoundaries(
    ros::Publisher& pub,
    const std_msgs::Header& header,
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      boundary_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < convexes.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr
        one_boundary_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      convexes[i]->boundariesToPointCloud<pcl::PointXYZ>(
        *one_boundary_cloud);
      *boundary_cloud = *boundary_cloud + *one_boundary_cloud;
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*boundary_cloud, ros_cloud);
    ros_cloud.header = header;
    pub.publish(ros_cloud);
  }
  
  void EnvironmentPlaneModeling::publishGridMaps(
      ros::Publisher& pub,
      const std_msgs::Header& header,
      std::vector<GridPlane::Ptr>& grids)
  {
    jsk_recognition_msgs::SimpleOccupancyGridArray grid_array;
    grid_array.header = header;
    for (size_t i = 0; i < grids.size(); i++) {
      jsk_recognition_msgs::SimpleOccupancyGrid grid = grids[i]->toROSMsg();
      grid.header = header;
      grid_array.grids.push_back(grid);
    }
    pub.publish(grid_array);
  }
  
  
  std::vector<GridPlane::Ptr> EnvironmentPlaneModeling::buildGridPlanes(
    pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes,
    std::set<int>& non_plane_indices)
  {
    std::vector<GridPlane::Ptr> ret(convexes.size());
//#pragma omp parallel for
    for (size_t i = 0; i < convexes.size(); i++) {
      GridPlane::Ptr grid(new GridPlane(convexes[i], resolution_));
      size_t num = grid->fillCellsFromPointCloud(cloud, distance_threshold_,
                                                 normal_threshold_,
                                                 non_plane_indices);
      NODELET_INFO("%lu plane contains %lu points",
                       i, num);
      ret[i] = grid;
    }
    return ret;
  }

  

  void EnvironmentPlaneModeling::publishConvexPolygons(
    ros::Publisher& pub,
    const std_msgs::Header& header,
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
  {
    jsk_recognition_msgs::PolygonArray polygon_array;
    polygon_array.header = header;
    for (size_t i = 0; i < convexes.size(); i++) {
      geometry_msgs::PolygonStamped polygon;
      polygon.polygon = convexes[i]->toROSMsg();
      polygon.header = header;
      polygon_array.polygons.push_back(polygon);
    }
    pub.publish(polygon_array);
  }

  std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> EnvironmentPlaneModeling::magnifyConvexes(
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
  {
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> ret(0);
    for (size_t i = 0; i < convexes.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr vertices_convex(new jsk_recognition_utils::ConvexPolygon(convexes[i]->getVertices()));
      jsk_recognition_utils::ConvexPolygon::Ptr new_convex = vertices_convex->magnifyByDistance(magnify_distance_);
      // check orientation
      if (new_convex->getNormalFromVertices().dot(Eigen::Vector3f::UnitZ()) < 0) {
        new_convex = boost::make_shared<jsk_recognition_utils::ConvexPolygon>(new_convex->flipConvex());
      }
      ret.push_back(new_convex);
    }
    return ret;
  }
  
  std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> EnvironmentPlaneModeling::convertToConvexPolygons(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes(0);
    for (size_t i = 0; i < indices_msg->cluster_indices.size(); i++) {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      inliers->indices = indices_msg->cluster_indices[i].indices;
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      coefficients->values = coefficients_msg->coefficients[i].values;
      jsk_recognition_utils::ConvexPolygon::Ptr convex
        = jsk_recognition_utils::convexFromCoefficientsAndInliers<pcl::PointNormal>(
          cloud, inliers, coefficients);
      convexes.push_back(convex);
    }
    
    return convexes;
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EnvironmentPlaneModeling,
                        nodelet::Nodelet);
