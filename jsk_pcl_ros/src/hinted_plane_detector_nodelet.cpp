/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, JSK Lab
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
#include "jsk_pcl_ros/hinted_plane_detector.h"
#include "pcl_ros/transforms.h"
#include <visualization_msgs/Marker.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros {
  
  void HintedPlaneDetector::onInit() {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&HintedPlaneDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    pub_hint_polygon_ = advertise<geometry_msgs::PolygonStamped>(
      *pnh_, "output/hint/polygon", 1);
    pub_hint_polygon_array_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output/hint/polygon_array", 1);
    pub_hint_inliers_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/hint/inliers", 1);
    pub_hint_coefficients_ = advertise<PCLModelCoefficientMsg>(
      *pnh_, "output/hint/coefficients", 1);
    pub_polygon_ = advertise<geometry_msgs::PolygonStamped>(
      *pnh_, "output/polygon", 1);
    pub_polygon_array_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output/polygon_array", 1);
    pub_hint_filtered_indices_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/hint_filtered_indices", 1);
    pub_plane_filtered_indices_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/plane_filtered_indices", 1);
    pub_density_filtered_indices_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/density_filtered_indices", 1);
    pub_euclidean_filtered_indices_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/euclidean_filtered_indices", 1);
    pub_inliers_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/inliers", 1);
    pub_coefficients_ = advertise<PCLModelCoefficientMsg>(
      *pnh_, "output/coefficients", 1);
    onInitPostProcess();
  }

  void HintedPlaneDetector::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_hint_cloud_.subscribe(*pnh_, "input/hint/cloud", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_hint_cloud_);
    sync_->registerCallback(boost::bind(&HintedPlaneDetector::detect,
                                        this, _1, _2));
  }

  void HintedPlaneDetector::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_hint_cloud_.unsubscribe();
  }

  void HintedPlaneDetector::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    hint_outlier_threashold_ = config.hint_outlier_threashold;
    hint_max_iteration_ = config.hint_max_iteration;
    hint_min_size_ = config.hint_min_size;
    outlier_threashold_ = config.outlier_threashold;
    max_iteration_ = config.max_iteration;
    min_size_ = config.min_size;
    eps_angle_ = config.eps_angle;
    normal_filter_eps_angle_ = config.normal_filter_eps_angle;
    euclidean_clustering_filter_tolerance_ = config.euclidean_clustering_filter_tolerance;
    euclidean_clustering_filter_min_size_ = config.euclidean_clustering_filter_min_size;
    density_radius_ = config.density_radius;
    density_num_ = config.density_num;
    enable_euclidean_filtering_ = config.enable_euclidean_filtering;
    enable_normal_filtering_ = config.enable_normal_filtering;
    enable_distance_filtering_ = config.enable_distance_filtering;
    enable_density_filtering_ = config.enable_density_filtering;
  }
  
  void HintedPlaneDetector::detect(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& hint_cloud_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      input_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      hint_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *input_cloud);
    pcl::fromROSMsg(*hint_cloud_msg, *hint_cloud);
    
    // estimate plane out of hint_cloud
    
    jsk_recognition_utils::ConvexPolygon::Ptr convex;
    
    if (detectHintPlane(hint_cloud, convex) && convex) {
      if (detectLargerPlane(input_cloud, convex)) {
        NODELET_INFO("success to detect!");
      }
      else {
        NODELET_ERROR("failed to detect larger plane");
      }
    }
  }

  pcl::PointIndices::Ptr HintedPlaneDetector::getBestCluster(
    pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const jsk_recognition_utils::ConvexPolygon::Ptr hint_convex)
  {
    Eigen::Vector3f center = hint_convex->centroid();
    double min_dist = DBL_MAX;
    size_t min_index = 0;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      Eigen::Vector4f center_cluster4;
      pcl::compute3DCentroid<pcl::PointNormal>(*input_cloud,
                                               cluster_indices[i].indices,
                                               center_cluster4);
      Eigen::Vector3f center_cluster3(center_cluster4[0], center_cluster4[1], center_cluster4[2]);
      double dist = (center - center_cluster3).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_index = i;
      }
    }
    pcl::PointIndices::Ptr ret (new pcl::PointIndices);
    ret->indices = cluster_indices[min_index].indices;
    return ret;
  }

  void HintedPlaneDetector::densityFilter(
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    const pcl::PointIndices::Ptr indices,
    pcl::PointIndices& output)
  {
    if (enable_density_filtering_) {
      pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
      pcl::KdTreeFLANN<pcl::PointNormal>::IndicesPtr indices_ptr
        (new std::vector<int>);
      *indices_ptr = indices->indices;
      kdtree.setInputCloud(cloud, indices_ptr);
      for (size_t i = 0; i < indices->indices.size(); i++) {
        int point_index = indices->indices[i];
        std::vector<int> result_indices;
        std::vector<float> result_distances;
        kdtree.radiusSearch(i, density_radius_,
                            result_indices, result_distances);
        if (result_distances.size() >= density_num_) {
          output.indices.push_back(point_index);
        }
      }
    }
    else {
      output = *indices;
    }
    output.header = cloud->header;
    PCLIndicesMsg ros_indices;
    pcl_conversions::fromPCL(output, ros_indices);
    pub_density_filtered_indices_.publish(ros_indices);
  }

  void HintedPlaneDetector::euclideanFilter(
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    const pcl::PointIndices::Ptr indices,
    const jsk_recognition_utils::ConvexPolygon::Ptr hint_convex,
    pcl::PointIndices& output)
  {
    if (enable_euclidean_filtering_) {
      pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
      ec.setClusterTolerance(euclidean_clustering_filter_tolerance_);
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree
        (new pcl::search::KdTree<pcl::PointNormal>);
      tree->setInputCloud(cloud);
      ec.setSearchMethod(tree);
      ec.setIndices(indices);
      ec.setInputCloud(cloud);
      //ec.setMinClusterSize ();
      std::vector<pcl::PointIndices> cluster_indices;
      ec.extract(cluster_indices);
      if (cluster_indices.size() == 0) {
        return;
      }
      NODELET_INFO("%lu clusters", cluster_indices.size());
      pcl::PointIndices::Ptr filtered_indices
        = getBestCluster(cloud, cluster_indices, hint_convex);
      output = *filtered_indices;
    }
    else {
      output = *indices;
    }
    output.header = cloud->header;
    PCLIndicesMsg ros_indices;
    pcl_conversions::fromPCL(output, ros_indices);
    pub_euclidean_filtered_indices_.publish(ros_indices);
  }

  void HintedPlaneDetector::hintFilter(
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    const jsk_recognition_utils::ConvexPolygon::Ptr hint_convex,
    pcl::PointIndices& output)
  {
    for (size_t i = 0; i < cloud->points.size(); i++) {
      pcl::PointNormal p = cloud->points[i];
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.y)) {
        Eigen::Vector4f v = p.getVector4fMap();
        if (!enable_distance_filtering_ || hint_convex->distanceToPoint(v) < outlier_threashold_) {
          Eigen::Vector3f n(p.normal_x, p.normal_y, p.normal_z);
          if (!enable_normal_filtering_ || hint_convex->angle(n) < normal_filter_eps_angle_) {
            output.indices.push_back(i);
          }
        }
      }
    }
    output.header = cloud->header;
    PCLIndicesMsg ros_candidate_inliers;
    pcl_conversions::fromPCL(output, ros_candidate_inliers);
    pub_hint_filtered_indices_.publish(ros_candidate_inliers);
  }

  void HintedPlaneDetector::planeFilter(
    const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
    const pcl::PointIndices::Ptr indices,
    const Eigen::Vector3f& normal,
    pcl::PointIndices& output,
    pcl::ModelCoefficients& coefficients)
  {
    pcl::SACSegmentation<pcl::PointNormal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (outlier_threashold_);
    seg.setMaxIterations (max_iteration_);
    seg.setEpsAngle(eps_angle_);
    seg.setAxis(normal);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.segment(output, coefficients);
    coefficients.header = cloud->header;
    output.header = cloud->header;
    PCLIndicesMsg ros_indices;
    pcl_conversions::fromPCL(output, ros_indices);
    pub_plane_filtered_indices_.publish(ros_indices);
  }
  
  bool HintedPlaneDetector::detectLargerPlane(
    pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,
    jsk_recognition_utils::ConvexPolygon::Ptr hint_convex)
  {
    pcl::PointIndices::Ptr candidate_inliers (new pcl::PointIndices);
    hintFilter(input_cloud, hint_convex, *candidate_inliers);
    // filter points based on hint_convex to get better result...
    
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    planeFilter(input_cloud, candidate_inliers,
                hint_convex->getNormal(),
                *plane_inliers, *plane_coefficients);
    if (plane_inliers->indices.size() < min_size_) { // good!
      NODELET_ERROR("failed to detect by plane fitting filtering");
      return false;
    }
    // Check direction of plane_coefficients
    Eigen::Vector3f plane_normal(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);
    if (plane_normal.dot(Eigen::Vector3f::UnitZ()) > 0) {
      // flip
      plane_coefficients->values[0] = -plane_coefficients->values[0];
      plane_coefficients->values[1] = -plane_coefficients->values[1];
      plane_coefficients->values[2] = -plane_coefficients->values[2];
      plane_coefficients->values[3] = -plane_coefficients->values[3];
    }
    // filtering by euclidean clustering
    pcl::PointIndices::Ptr euclidean_filtered_indices(new pcl::PointIndices);
    euclideanFilter(input_cloud, plane_inliers, hint_convex,
                    *euclidean_filtered_indices);
    if (euclidean_filtered_indices->indices.size() < min_size_) {
      NODELET_ERROR("failed to detect by euclidean filtering");
      return false;
    }
    pcl::PointIndices::Ptr density_filtered_indices (new pcl::PointIndices);
    densityFilter(
      input_cloud, euclidean_filtered_indices, *density_filtered_indices);
      
    if (density_filtered_indices->indices.size() < min_size_) {
      NODELET_ERROR("failed to detect by density filtering");
      return false;
    }
    jsk_recognition_utils::ConvexPolygon::Ptr convex
      = jsk_recognition_utils::convexFromCoefficientsAndInliers<pcl::PointNormal>(
        input_cloud, density_filtered_indices, plane_coefficients);
    // publish to ROS
    publishPolygon(convex, pub_polygon_, pub_polygon_array_,
                   input_cloud->header);
    PCLIndicesMsg ros_inliers;
    pcl_conversions::fromPCL(*density_filtered_indices, ros_inliers);
    pub_inliers_.publish(ros_inliers);
    PCLModelCoefficientMsg ros_coefficients;
    pcl_conversions::fromPCL(*plane_coefficients, ros_coefficients);
    pub_coefficients_.publish(ros_coefficients);
    return true;
  }

  void HintedPlaneDetector::publishPolygon(
    const jsk_recognition_utils::ConvexPolygon::Ptr convex,
    ros::Publisher& pub_polygon,
    ros::Publisher& pub_polygon_array,
    const pcl::PCLHeader& header)
  {
    geometry_msgs::PolygonStamped ros_polygon;
    ros_polygon.polygon = convex->toROSMsg();
    pcl_conversions::fromPCL(header, ros_polygon.header);
    jsk_recognition_msgs::PolygonArray ros_polygon_array;
    pcl_conversions::fromPCL(header, ros_polygon_array.header);
    ros_polygon_array.polygons.push_back(
      ros_polygon);
    pub_polygon_array.publish(ros_polygon_array);
    pub_polygon.publish(ros_polygon);
  }
  
  bool HintedPlaneDetector::detectHintPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr hint_cloud,
    jsk_recognition_utils::ConvexPolygon::Ptr& convex)
  {
    pcl::PointIndices::Ptr hint_inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr hint_coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (hint_outlier_threashold_);
    seg.setMaxIterations (hint_max_iteration_);
    seg.setInputCloud(hint_cloud);
    seg.segment(*hint_inliers, *hint_coefficients);
    if (hint_inliers->indices.size() > hint_min_size_) { // good!
      convex = jsk_recognition_utils::convexFromCoefficientsAndInliers<pcl::PointXYZ>(
        hint_cloud, hint_inliers, hint_coefficients);
      // publish hint results for debug/visualization
      publishPolygon(convex,
                     pub_hint_polygon_, pub_hint_polygon_array_,
                     hint_cloud->header);
      PCLIndicesMsg ros_indices;
      PCLModelCoefficientMsg ros_coefficients;
      pcl_conversions::fromPCL(*hint_inliers, ros_indices);
      pub_hint_inliers_.publish(ros_indices);
      pcl_conversions::fromPCL(*hint_coefficients, ros_coefficients);
      pub_hint_coefficients_.publish(ros_coefficients);
      return true;
    }
    else {
      NODELET_ERROR("Failed to find hint plane");
      return false;
    }
  }

}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HintedPlaneDetector, nodelet::Nodelet);
