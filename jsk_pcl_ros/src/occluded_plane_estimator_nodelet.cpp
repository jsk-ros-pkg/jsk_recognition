// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/occluded_plane_estimator.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

#include "jsk_pcl_ros/grid_map.h"

namespace jsk_pcl_ros
{
  void OccludedPlaneEstimator::onInit()
  {
    PCLNodelet::onInit();
    polygon_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_polygons", 1);
    coefficient_pub_ = pnh_->advertise<jsk_pcl_ros::ModelCoefficientsArray>("output_coefficients", 1);
    indices_pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>("output_indices", 1);
    cloud_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OccludedPlaneEstimator::configCallback, this, _1, _2);
    srv_->setCallback (f);

    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sub_static_polygons_.subscribe(*pnh_, "input_static_polygons", 1);
    sub_static_coefficients_.subscribe(*pnh_, "input_static_coefficients", 1);
    sub_pointcloud_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sync_->connectInput(sub_pointcloud_, sub_indices_,
                        sub_polygons_, sub_coefficients_,
                        sub_static_polygons_, sub_static_coefficients_);
    sync_->registerCallback(boost::bind(&OccludedPlaneEstimator::estimate,
                                        this, _1, _2, _3, _4, _5, _6));
    require_estimation_ = false;
    require_estimation_service_ = pnh_->advertiseService(
      "require_estimation",
      &OccludedPlaneEstimator::requireEstimationCallback,
      this);
  }

  void OccludedPlaneEstimator::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    plane_distance_threshold_ = config.plane_distance_threshold;
    plane_angle_threshold_ = config.plane_angle_threshold;
  }

  bool OccludedPlaneEstimator::requireEstimationCallback(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock(estimation_mutex_);
    require_estimation_ = true;
    return true;
  }

  void OccludedPlaneEstimator::normalizePlaneParameters(
    const PCLModelCoefficientMsg& coefficient,
    Eigen::Vector3f& normal,
    double& d)
  {
    normal[0] = coefficient.values[0];
    normal[1] = coefficient.values[1];
    normal[2] = coefficient.values[2];
    d = coefficient.values[3];
    if (normal.norm() != 1.0) {
      d = d / normal.norm();
      normal = normal / normal.norm();
    }
  }
  
  // linear search, it may not fast enough if we need to tuckle against
  // larger scene
  int OccludedPlaneEstimator::findNearestPolygon(
    const jsk_pcl_ros::PolygonArray::ConstPtr& polygons,
    const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
    const geometry_msgs::PolygonStamped& static_polygon,
    const PCLModelCoefficientMsg& static_coefficient)
  {
    int nearest_index = -1;
    double min_angle_distance = DBL_MAX;
    for (size_t j = 0; j < polygons->polygons.size(); j++) {
      geometry_msgs::PolygonStamped candidate_polygon = polygons->polygons[j];
      PCLModelCoefficientMsg candidate_coefficient = coefficients->coefficients[j];
      if (candidate_polygon.header.frame_id != static_polygon.header.frame_id) {
        NODELET_ERROR("frame_id of static polygon and candidate polygon are not the same one: %s and %s",
                      static_polygon.header.frame_id.c_str(),
                      candidate_polygon.header.frame_id.c_str());
        continue;
      }
      Eigen::Vector3f a_normal, b_normal;
      double a_distance, b_distance;
      normalizePlaneParameters(candidate_coefficient, a_normal, a_distance);
      normalizePlaneParameters(static_coefficient, b_normal, b_distance);
      if (a_normal.dot(b_normal) < 0) {
        b_distance = - b_distance;
        b_normal = - b_normal;
      }
      // NODELET_INFO("[%f, %f, %f] - %f  --- [%f, %f, %f] - %f",
      //              a_normal[0], a_normal[1], a_normal[2], a_distance,
      //              b_normal[0], b_normal[1], b_normal[2], b_distance);
      // NODELET_INFO("%lu - %lu distance: %f", i, j, fabs(fabs(a_distance) - fabs(b_distance)));
      if (fabs(fabs(a_distance) - fabs(b_distance)) > plane_distance_threshold_) {
        continue;
      }
      double theta = fabs(acos(a_normal.dot(b_normal)));
      // NODELET_INFO("%lu - %lu angle: %f", i, j, theta);
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


  void OccludedPlaneEstimator::extendConvexPolygon(
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
      p.x = static_polygon.polygon.points[j].x;
      p.y = static_polygon.polygon.points[j].y;
      p.z = static_polygon.polygon.points[j].z;
      cloud.points.push_back(p);
    }
    for (size_t j = 0; j < nearest_polygon.polygon.points.size(); j++) {
      pcl::PointXYZ p;
      p.x = nearest_polygon.polygon.points[j].x;
      p.y = nearest_polygon.polygon.points[j].y;
      p.z = nearest_polygon.polygon.points[j].z;
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
      p.x = chull_output.points[j].x;
      p.y = chull_output.points[j].y;
      p.z = chull_output.points[j].z;
      output_polygon.polygon.points.push_back(p);
    }
  }
  
  void OccludedPlaneEstimator::updateAppendingInfo(
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

  void OccludedPlaneEstimator::msgToPCL(const geometry_msgs::Point32& msg,
                                        pcl::PointXYZRGB& p)
  {
    p.x = msg.x;
    p.y = msg.y;
    p.z = msg.z;
  }

  void OccludedPlaneEstimator::computePolygonCentroid(const geometry_msgs::PolygonStamped msg,
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

  // method to deep copy ClusterPointIndices
  void OccludedPlaneEstimator::copyClusterPointIndices
  (const jsk_pcl_ros::ClusterPointIndices::ConstPtr& indices,
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

  void OccludedPlaneEstimator::addIndices(const size_t start, const size_t end,
                                          PCLIndicesMsg& output)
  {
    for (size_t i = start; i < end; i++) {
      output.indices.push_back(i);
    }
  }
  
  void OccludedPlaneEstimator::fullfillEstimatedRegionByPointCloud
  (const std_msgs::Header& header,
   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
   const jsk_pcl_ros::ClusterPointIndices::ConstPtr& indices,
   const jsk_pcl_ros::PolygonArray::ConstPtr& polygons,
   const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
   const jsk_pcl_ros::PolygonArray::ConstPtr& static_polygons,
   const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& static_coefficients,
   const jsk_pcl_ros::PolygonArray& result_polygons,
   const std::map<int, std::set<size_t> >& estimation_summary)
  {
    NODELET_INFO("%lu convexhull will be fulfilled", estimation_summary.size());
    typedef std::map<int, std::set<size_t> >::const_iterator Iterator;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *all_cloud = *input;
    // copy indices
    jsk_pcl_ros::ClusterPointIndices all_indices;
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
      GridMap grid(0.01, coefficients->coefficients[env_plane_index].values);
      grid.registerPointCloud(input);
      geometry_msgs::PolygonStamped convex_polygon
        = result_polygons.polygons[env_plane_index];
      for (size_t i = 0; i < convex_polygon.polygon.points.size() - 1; i++) {
        geometry_msgs::Point32 from = convex_polygon.polygon.points[i];
        geometry_msgs::Point32 to = convex_polygon.polygon.points[i + 1];
        pcl::PointXYZRGB from_pcl, to_pcl;
        msgToPCL(from, from_pcl);
        msgToPCL(to, to_pcl);
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
        NODELET_INFO("%lu static polygon merged into %d env polygon and %lu points is required to fill",
                     *it, env_plane_index,
                     filled_indices.size() - before_point_size);
      }
      
      NODELET_INFO("add %lu points into %d cluster",
                   filled_indices.size(),
                   env_plane_index);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      grid.indicesToPointCloud(filled_indices, new_cloud);
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
    cloud_pub_.publish(ros_output);
    indices_pub_.publish(all_indices);
    
  }
  
  
  void OccludedPlaneEstimator::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& input,
    const jsk_pcl_ros::ClusterPointIndices::ConstPtr& input_indices,
    const jsk_pcl_ros::PolygonArray::ConstPtr& polygons,
    const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
    const jsk_pcl_ros::PolygonArray::ConstPtr& static_polygons,
    const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& static_coefficients)
  {
    boost::mutex::scoped_lock(mutex_);
    boost::mutex::scoped_lock(estimation_mutex_);
    if (!require_estimation_) {
      NODELET_DEBUG("no estimation is required");
      return;
    }
    // error check
    if (polygons->polygons.size() != coefficients->coefficients.size()) {
      NODELET_ERROR("the size of the input polygon array and model coefficients array is not same");
      return;
    }
    if (static_polygons->polygons.size() != static_coefficients->coefficients.size()) {
      NODELET_ERROR("the size of the input static polygon array and static model coefficients array is not same");
      return;
    }
    jsk_pcl_ros::PolygonArray result_polygons;
    jsk_pcl_ros::ModelCoefficientsArray result_coefficients;
    result_polygons = *polygons;
    result_coefficients = *coefficients;
    // a map from static_polygons index to the list of
    // the indices of the polygons which are appended
    std::map<int, std::set<size_t> > appending_map;
    for (size_t i = 0; i < static_polygons->polygons.size(); i++) {
      // looking for the nearest polygon from the static polygon
      geometry_msgs::PolygonStamped static_polygon = static_polygons->polygons[i];
      PCLModelCoefficientMsg static_coefficient = static_coefficients->coefficients[i];
      int nearest_index = findNearestPolygon(polygons,
                                             coefficients,
                                             static_polygon,
                                             static_coefficient);
      if (nearest_index != -1) {
        // merged into nearest_index
        NODELET_INFO("merging %lu into %d", i, nearest_index);
        geometry_msgs::PolygonStamped nearest_polygon = result_polygons.polygons[nearest_index];
        geometry_msgs::PolygonStamped new_polygon;
        extendConvexPolygon(static_polygon,
                            coefficients->coefficients[nearest_index],
                            nearest_polygon,
                            new_polygon);
        result_polygons.polygons[nearest_index] = new_polygon;
        
        // add the result into appended_map
        updateAppendingInfo(nearest_index, i, appending_map);
      }
    }

    polygon_pub_.publish(result_polygons);
    coefficient_pub_.publish(result_coefficients);
    
    // fulfill estimated region by pointcloud and publish thems
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    fromROSMsg(*input, *pcl_cloud);
    fullfillEstimatedRegionByPointCloud(input->header,
                                        pcl_cloud,
                                        input_indices,
                                        polygons,
                                        coefficients,
                                        static_polygons,
                                        static_coefficients,
                                        result_polygons,
                                        appending_map);
    
    
    
    require_estimation_ = false;
  }
  
}

typedef jsk_pcl_ros::OccludedPlaneEstimator OccludedPlaneEstimator;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, OccludedPlaneEstimator, OccludedPlaneEstimator, nodelet::Nodelet);
