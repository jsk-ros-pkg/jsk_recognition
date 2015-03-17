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

#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/grid_map.h"

namespace jsk_pcl_ros
{
  void EnvironmentPlaneModeling::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EnvironmentPlaneModeling::configCallback, this, _1, _2);
    srv_->setCallback (f);

    debug_magnified_polygons_
      = pnh_->advertise<jsk_recognition_msgs::PolygonArray>(
        "debug/magnified_polygons", 1);
    pub_grid_map_
      = pnh_->advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>(
        "output", 1);
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_polygons_.subscribe(*pnh_, "input/polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_polygons_,
                        sub_coefficients_, sub_indices_);
    sync_->registerCallback(
      boost::bind(&EnvironmentPlaneModeling::inputCallback,
                  this, _1, _2, _3, _4));
  }

  void EnvironmentPlaneModeling::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    magnify_distance_ = config.magnify_distance;
    distance_threshold_ = config.distance_threshold;
    resolution_ = config.resolution;
  }

  void EnvironmentPlaneModeling::printInputData(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    NODELET_INFO("Input data --");
    NODELET_INFO("  Number of points -- %d", cloud_msg->width * cloud_msg->height);
    NODELET_INFO("  Number of Clusters: -- %lu", indices_msg->cluster_indices.size());
    NODELET_INFO("  Frame Id: %s", cloud_msg->header.frame_id.c_str());
  } 

  bool EnvironmentPlaneModeling::isValidFrameIds(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    std::string frame_id = cloud_msg->header.frame_id;
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
  
  void EnvironmentPlaneModeling::inputCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    // check frame_id
    if (!isValidFrameIds(cloud_msg, polygon_msg, coefficients_msg, indices_msg)) {
      NODELET_FATAL("frame_id is not correct");
      return;
    }
    // first, print all the information about ~inputs
    printInputData(cloud_msg, polygon_msg, coefficients_msg, indices_msg);

    
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // convert to jsk_pcl_ros::ConvexPolygon
    std::vector<ConvexPolygon::Ptr> convexes = convertToConvexPolygons(cloud, indices_msg, coefficients_msg);
    // magnify convexes
    std::vector<ConvexPolygon::Ptr> magnified_convexes = magnifyConvexes(convexes);

    // Publish magnified convexes for debug
    publishConvexPolygons(debug_magnified_polygons_, cloud_msg->header, magnified_convexes);

    // build GridMaps
    std::vector<GridPlane::Ptr> grid_planes = buildGridPlanes(cloud, magnified_convexes);

    publishGridMaps(pub_grid_map_, cloud_msg->header, grid_planes);
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
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    std::vector<ConvexPolygon::Ptr> convexes)
  {
    std::vector<GridPlane::Ptr> ret(convexes.size());
    
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t i = 0; i < convexes.size(); i++) {
      GridPlane::Ptr grid(new GridPlane(convexes[i], resolution_));
      grid->fillCellsFromPointCloud(cloud, distance_threshold_);
      ret[i] = grid;
    }
    return ret;
  }

  

  void EnvironmentPlaneModeling::publishConvexPolygons(
    ros::Publisher& pub,
    const std_msgs::Header& header,
    std::vector<ConvexPolygon::Ptr>& convexes)
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

  std::vector<ConvexPolygon::Ptr> EnvironmentPlaneModeling::magnifyConvexes(
    std::vector<ConvexPolygon::Ptr>& convexes)
  {
    std::vector<ConvexPolygon::Ptr> ret(0);
    for (size_t i = 0; i < convexes.size(); i++) {
      ret.push_back(convexes[i]->magnifyByDistance(magnify_distance_));
    }
    return ret;
  }
  
  std::vector<ConvexPolygon::Ptr> EnvironmentPlaneModeling::convertToConvexPolygons(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    std::vector<ConvexPolygon::Ptr> convexes(0);

    for (size_t i = 0; i < indices_msg->cluster_indices.size(); i++) {
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      inliers->indices = indices_msg->cluster_indices[i].indices;
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      coefficients->values = coefficients_msg->coefficients[i].values;
      ConvexPolygon::Ptr convex
        = convexFromCoefficientsAndInliers<pcl::PointNormal>(
          cloud, inliers, coefficients);
      convexes.push_back(convex);
    }
    
    return convexes;
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EnvironmentPlaneModeling,
                        nodelet::Nodelet);
