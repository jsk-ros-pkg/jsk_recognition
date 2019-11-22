// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "jsk_pcl_ros/organized_multi_plane_segmentation.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <set>
#include <Eigen/StdVector>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/integral_image_normal.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pluginlib/class_list_macros.h>

#include <boost/format.hpp>
#include <pcl/common/centroid.h>
#include <visualization_msgs/Marker.h>
#include "jsk_recognition_utils/geo_util.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <jsk_topic_tools/color_utils.h>

namespace jsk_pcl_ros
{

  void OrganizedMultiPlaneSegmentation::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    //////////////////////////////////////////////////////////
    // prepare diagnostics
    //////////////////////////////////////////////////////////
    diagnostic_updater_.reset(new diagnostic_updater::Updater);
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(
      getName() + "::NormalEstimation",
      boost::bind(
        &OrganizedMultiPlaneSegmentation::updateDiagnosticNormalEstimation,
        this,
        _1));
    diagnostic_updater_->add(
      getName() + "::PlaneSegmentation",
      boost::bind(
        &OrganizedMultiPlaneSegmentation::updateDiagnosticPlaneSegmentation,
        this,
        _1));
    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    normal_estimation_vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    plane_segmentation_vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    estimate_normal_ = true;
    pnh_->getParam("estimate_normal", estimate_normal_);
    //////////////////////////////////////////////////////////
    // prepare publishers
    //////////////////////////////////////////////////////////
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    polygon_pub_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output_polygon", 1);
    coefficients_pub_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_, "output_coefficients", 1);
    org_pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output_nonconnected", 1);
    org_polygon_pub_
      = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output_nonconnected_polygon", 1);
    org_coefficients_pub_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_, 
        "output_nonconnected_coefficients", 1);
    
    refined_pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, 
      "output_refined", 1);
    refined_polygon_pub_
      = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output_refined_polygon", 1);
    refined_coefficients_pub_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_, 
        "output_refined_coefficients", 1);
    
    pub_connection_marker_
      = advertise<visualization_msgs::Marker>(*pnh_, 
        "debug_connection_map", 1);

    if (estimate_normal_) {
      normal_pub_
        = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_normal", 1);
    }
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &OrganizedMultiPlaneSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);

    diagnostics_timer_ = pnh_->createTimer(
      ros::Duration(1.0),
      boost::bind(&OrganizedMultiPlaneSegmentation::updateDiagnostics,
                  this,
                  _1));
    onInitPostProcess();
  }

  void OrganizedMultiPlaneSegmentation::forceToDirectOrigin(
    const std::vector<pcl::ModelCoefficients>& coefficients,
    std::vector<pcl::ModelCoefficients>& output_coefficients)
  {
    output_coefficients.resize(coefficients.size());
    for (size_t i = 0; i < coefficients.size(); i++) {
      pcl::ModelCoefficients plane_coefficient = coefficients[i];
      jsk_recognition_utils::Plane plane(coefficients[i].values);
      
      Eigen::Vector3f p = plane.getPointOnPlane();
      Eigen::Vector3f n = plane.getNormal();
      if (p.dot(n) < 0) {
        output_coefficients[i] = plane_coefficient;
      }
      else {
        jsk_recognition_utils::Plane flip = plane.flip();
        pcl::ModelCoefficients new_coefficient;
        flip.toCoefficients(new_coefficient.values);
        output_coefficients[i] = new_coefficient;
      }
    }
  }

  void OrganizedMultiPlaneSegmentation::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1,
                           &OrganizedMultiPlaneSegmentation::segment, this);
  }

  void OrganizedMultiPlaneSegmentation::unsubscribe()
  {
    sub_.shutdown();
  }

  void OrganizedMultiPlaneSegmentation::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_size_ = config.min_size;
    angular_threshold_ = config.angular_threshold;
    distance_threshold_ = config.distance_threshold;
    max_curvature_ = config.max_curvature;
    connect_plane_angle_threshold_ = config.connect_plane_angle_threshold;
    connect_distance_threshold_ = config.connect_distance_threshold;
    max_depth_change_factor_ = config.max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    depth_dependent_smoothing_ = config.depth_dependent_smoothing;
    estimation_method_ = config.estimation_method;
    border_policy_ignore_ = config.border_policy_ignore;
    publish_normal_ = config.publish_normal;
    ransac_refine_coefficients_ = config.ransac_refine_coefficients;
    ransac_refine_outlier_distance_threshold_
      = config.ransac_refine_outlier_distance_threshold;
    min_refined_area_threshold_ = config.min_refined_area_threshold;
    max_refined_area_threshold_ = config.max_refined_area_threshold;
    //concave_alpha_ = config.concave_alpha;
  }
  
  void OrganizedMultiPlaneSegmentation::connectPlanesMap(
    const pcl::PointCloud<PointT>::Ptr& input,
    const std::vector<pcl::ModelCoefficients>& model_coefficients,
    const std::vector<pcl::PointIndices>& boundary_indices,
    jsk_recognition_utils::IntegerGraphMap& connection_map)
  {
    NODELET_DEBUG("size of model_coefficients: %lu", model_coefficients.size());
    if (model_coefficients.size() == 0) {
      return;                   // do nothing
    }

    if (model_coefficients.size() == 1) {
      connection_map[0]= std::vector<int>();
      return;
    }
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    for (size_t i = 0; i < model_coefficients.size(); i++) {
      // initialize connection_map[i]
      connection_map[i] = std::vector<int>();
      connection_map[i].push_back(i);
      for (size_t j = i + 1; j < model_coefficients.size(); j++) {
        // check if i and j can be connected
        pcl::ModelCoefficients a_coefficient = model_coefficients[i];
        pcl::ModelCoefficients b_coefficient = model_coefficients[j];
        Eigen::Vector3f a_normal(a_coefficient.values[0], a_coefficient.values[1], a_coefficient.values[2]);
        Eigen::Vector3f b_normal(b_coefficient.values[0], b_coefficient.values[1], b_coefficient.values[2]);
        double a_distance = a_coefficient.values[3];
        double b_distance = b_coefficient.values[3];
        // force to check the coefficients is normalized
        if (a_normal.norm() != 1.0) {
          a_distance = a_distance / a_normal.norm();
          a_normal = a_normal / a_normal.norm();
        }
        if (b_normal.norm() != 1.0) {
          b_distance = b_distance / b_normal.norm();
          b_normal = b_normal / b_normal.norm();
        }
        double theta = fabs(acos(a_normal.dot(b_normal)));
        NODELET_DEBUG("%lu - %lu angle: %f", i, j, theta);
        if (theta > M_PI / 2.0) {
          theta = M_PI  - theta;
        }
        if (theta > connect_plane_angle_threshold_) {
          continue;
        }
        // the planes are near enough as a plane formula.

        // compute the distance between two boundaries.
        // if they are near enough, we can regard these two map should connect
        pcl::PointIndices::Ptr a_indices
          = boost::make_shared<pcl::PointIndices>(boundary_indices[i]);
        pcl::PointIndices::Ptr b_indices
          = boost::make_shared<pcl::PointIndices>(boundary_indices[j]);
        pcl::PointCloud<PointT> a_cloud, b_cloud;
        extract.setIndices(a_indices);
        extract.filter(a_cloud);
        extract.setIndices(b_indices);
        extract.filter(b_cloud);
        if (a_cloud.points.size() > 0) {
          pcl::KdTreeFLANN<PointT> kdtree;
          kdtree.setInputCloud(a_cloud.makeShared());
          bool foundp = false;
          for (size_t pi = 0; pi < b_cloud.points.size(); pi++) {
            PointT p = b_cloud.points[pi];
            std::vector<int> k_indices;
            std::vector<float> k_sqr_distances;
            if (kdtree.radiusSearch(
                  p, connect_distance_threshold_, k_indices, k_sqr_distances, 1) > 0) {
              NODELET_DEBUG("%lu - %lu connected", i, j);
              foundp = true;
              break;
            }
          }
          if (foundp) {
            connection_map[i].push_back(j);
          }
        }
      }
    }
  }

  void OrganizedMultiPlaneSegmentation::pclIndicesArrayToClusterPointIndices(
    const std::vector<pcl::PointIndices>& inlier_indices,
    const std_msgs::Header& header,
    jsk_recognition_msgs::ClusterPointIndices& output_indices)
  {
    for (size_t i = 0; i < inlier_indices.size(); i++) {
      pcl::PointIndices inlier = inlier_indices[i];
      PCLIndicesMsg one_indices;
      one_indices.header = header;
      one_indices.indices = inlier.indices;
      output_indices.cluster_indices.push_back(one_indices);
    }
  }
  
  void OrganizedMultiPlaneSegmentation::pointCloudToPolygon(
    const pcl::PointCloud<PointT>& input,
    geometry_msgs::Polygon& polygon)
  {
    for (size_t i = 0; i < input.points.size(); i++) {
      geometry_msgs::Point32 point;
      point.x = input.points[i].x;
      point.y = input.points[i].y;
      point.z = input.points[i].z;
      polygon.points.push_back(point);
    }
  }
  
  void OrganizedMultiPlaneSegmentation::buildConnectedPlanes(
    const pcl::PointCloud<PointT>::Ptr& input,
    const std_msgs::Header& header,
    const std::vector<pcl::PointIndices>& inlier_indices,
    const std::vector<pcl::PointIndices>& boundary_indices,
    const std::vector<pcl::ModelCoefficients>& model_coefficients,
    const jsk_recognition_utils::IntegerGraphMap& connection_map,
    std::vector<pcl::PointIndices>& output_indices,
    std::vector<pcl::ModelCoefficients>& output_coefficients,
    std::vector<pcl::PointCloud<PointT> >& output_boundary_clouds)
  { 
    std::vector<std::set<int> > cloud_sets;
    NODELET_DEBUG("connection_map:");
    for (jsk_recognition_utils::IntegerGraphMap::const_iterator it = connection_map.begin();
         it != connection_map.end();
         ++it) {
      int from_index = it->first;
      std::stringstream ss;
      ss << "connection map: " << from_index << " [";
      for (size_t i = 0; i < it->second.size(); i++) {
        ss << i << ", ";
      }
      NODELET_DEBUG("%s", ss.str().c_str());
    }

    jsk_recognition_utils::buildAllGroupsSetFromGraphMap(connection_map, cloud_sets);
    connected_plane_num_counter_.add(cloud_sets.size());
    for (size_t i = 0; i < cloud_sets.size(); i++) {
      pcl::PointIndices one_indices;
      pcl::PointIndices one_boundaries;
      std::vector<float> new_coefficients;
      new_coefficients.resize(4, 0);
      for (std::set<int>::iterator it = cloud_sets[i].begin();
           it != cloud_sets[i].end();
           ++it) {
        NODELET_DEBUG("%lu includes %d", i, *it);
        new_coefficients = model_coefficients[*it].values;
        pcl::PointIndices inlier = inlier_indices[*it];
        pcl::PointIndices boundary_inlier = boundary_indices[*it];
        // append indices...
        one_indices = *jsk_recognition_utils::addIndices(one_indices, inlier);
        one_boundaries = *jsk_recognition_utils::addIndices(one_boundaries, boundary_inlier);
      }
      if (one_indices.indices.size() == 0) {
        continue;
      }
      // normalize coefficients
      double norm = sqrt(new_coefficients[0] * new_coefficients[0] 
                         + new_coefficients[1] * new_coefficients[1]
                         + new_coefficients[2] * new_coefficients[2]);
      new_coefficients[0] /= norm;
      new_coefficients[1] /= norm;
      new_coefficients[2] /= norm;
      new_coefficients[3] /= norm;
      
      // take the average of the coefficients
      pcl::ModelCoefficients pcl_new_coefficients;
      pcl_new_coefficients.values = new_coefficients;
      // estimate concave hull
      pcl::PointIndices::Ptr indices_ptr
        = boost::make_shared<pcl::PointIndices>(one_boundaries);
      pcl::ModelCoefficients::Ptr coefficients_ptr
        = boost::make_shared<pcl::ModelCoefficients>(pcl_new_coefficients);
      jsk_recognition_utils::ConvexPolygon::Ptr convex
        = jsk_recognition_utils::convexFromCoefficientsAndInliers<PointT>(
          input, indices_ptr, coefficients_ptr);
      if (convex) {
        pcl::PointCloud<PointT> chull_output;
        convex->boundariesToPointCloud<PointT>(chull_output);
        output_indices.push_back(one_indices);
        output_coefficients.push_back(pcl_new_coefficients);
        output_boundary_clouds.push_back(chull_output);
      }
      else {
        NODELET_ERROR("failed to build convex");
      }
    }

  }

  ////////////////////////////////////////////////////////
  // simple PCL wrapper
  ////////////////////////////////////////////////////////
  void OrganizedMultiPlaneSegmentation::segmentOrganizedMultiPlanes(
    pcl::PointCloud<PointT>::Ptr input,
    pcl::PointCloud<pcl::Normal>::Ptr normal,
    PlanarRegionVector& regions,
    std::vector<pcl::ModelCoefficients>& model_coefficients,
    std::vector<pcl::PointIndices>& inlier_indices,
    pcl::PointCloud<pcl::Label>::Ptr& labels,
    std::vector<pcl::PointIndices>& label_indices,
    std::vector<pcl::PointIndices>& boundary_indices)
  {
    plane_segmentation_vital_checker_->poke();
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(min_size_);
    mps.setAngularThreshold(angular_threshold_);
    mps.setDistanceThreshold(distance_threshold_);
    mps.setMaximumCurvature(max_curvature_);
    mps.setInputCloud(input);
    mps.setInputNormals(normal);
    {
      jsk_topic_tools::ScopedTimer timer = plane_segmentation_time_acc_.scopedTimer();
      mps.segmentAndRefine(
        regions, model_coefficients, inlier_indices,
        labels, label_indices, boundary_indices);
    }
  }
  
  void OrganizedMultiPlaneSegmentation::publishSegmentationInformation(
    const std_msgs::Header& header,
    const pcl::PointCloud<PointT>::Ptr input,
    ros::Publisher& indices_pub,
    ros::Publisher& polygon_pub,
    ros::Publisher& coefficients_pub,
    const std::vector<pcl::PointIndices>& inlier_indices,
    const std::vector<pcl::PointCloud<PointT> >& boundaries,
    const std::vector<pcl::ModelCoefficients>& model_coefficients)
  {
    jsk_recognition_msgs::ClusterPointIndices indices;
    jsk_recognition_msgs::ModelCoefficientsArray coefficients_array;
    jsk_recognition_msgs::PolygonArray polygon_array;
    indices.header = header;
    polygon_array.header = header;
    coefficients_array.header = header;
    
    ////////////////////////////////////////////////////////
    // publish inliers
    ////////////////////////////////////////////////////////
    pclIndicesArrayToClusterPointIndices(inlier_indices, header, indices);
    indices_pub.publish(indices);
    
    ////////////////////////////////////////////////////////
    // boundaries as polygon
    ////////////////////////////////////////////////////////
    for (size_t i = 0; i < boundaries.size(); i++) {
      geometry_msgs::PolygonStamped polygon;
      pcl::PointCloud<PointT> boundary_cloud = boundaries[i];
      pointCloudToPolygon(boundary_cloud, polygon.polygon);
      polygon.header = header;
      polygon_array.polygons.push_back(polygon);
    }
    polygon_pub.publish(polygon_array);

    ////////////////////////////////////////////////////////
    // publish coefficients
    ////////////////////////////////////////////////////////
    for (size_t i = 0; i < model_coefficients.size(); i++) {
      PCLModelCoefficientMsg coefficient;
      coefficient.values = model_coefficients[i].values;
      coefficient.header = header;
      coefficients_array.coefficients.push_back(coefficient);
    }
    coefficients_pub.publish(coefficients_array);    
  }
  
  void OrganizedMultiPlaneSegmentation::publishSegmentationInformation(
    const std_msgs::Header& header,
    const pcl::PointCloud<PointT>::Ptr input,
    ros::Publisher& indices_pub,
    ros::Publisher& polygon_pub,
    ros::Publisher& coefficients_pub,
    const std::vector<pcl::PointIndices>& inlier_indices,
    const std::vector<pcl::PointIndices>& boundary_indices,
    const std::vector<pcl::ModelCoefficients>& model_coefficients)
  {
    // convert boundary_indices into boundary pointcloud
    std::vector<pcl::PointCloud<PointT> > boundaries;
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    for (size_t i = 0; i < boundary_indices.size(); i++) {
      pcl::PointCloud<PointT> boundary_cloud;
      pcl::PointIndices boundary_one_indices = boundary_indices[i];
      pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(boundary_indices[i]);
      extract.setIndices(indices_ptr);
      extract.filter(boundary_cloud);
      boundaries.push_back(boundary_cloud);
    }
    
    publishSegmentationInformation(
      header, input, indices_pub, polygon_pub, coefficients_pub,
      inlier_indices, boundaries, model_coefficients);
  }

  void OrganizedMultiPlaneSegmentation::publishMarkerOfConnection(
    jsk_recognition_utils::IntegerGraphMap connection_map,
    const pcl::PointCloud<PointT>::Ptr cloud,
    const std::vector<pcl::PointIndices>& inliers,
    const std_msgs::Header& header)
  {
    ////////////////////////////////////////////////////////
    // visualize connection as lines
    ////////////////////////////////////////////////////////
    visualization_msgs::Marker connection_marker;
    connection_marker.type = visualization_msgs::Marker::LINE_LIST;
    connection_marker.scale.x = 0.01;
    connection_marker.header = header;
    connection_marker.pose.orientation.w = 1.0;
    connection_marker.color = jsk_topic_tools::colorCategory20(0);
    
    ////////////////////////////////////////////////////////
    // first, compute centroids for each clusters
    ////////////////////////////////////////////////////////
    jsk_recognition_utils::Vertices centroids;
    for (size_t i = 0; i < inliers.size(); i++) {
      pcl::PointIndices::Ptr target_inliers
        = boost::make_shared<pcl::PointIndices>(inliers[i]);
      pcl::PointCloud<PointT>::Ptr target_cloud (new pcl::PointCloud<PointT>);
      Eigen::Vector4f centroid;
      pcl::ExtractIndices<PointT> ex;
      ex.setInputCloud(cloud);
      ex.setIndices(target_inliers);
      ex.filter(*target_cloud);
      pcl::compute3DCentroid(*target_cloud, centroid);
      Eigen::Vector3f centroid_3f(centroid[0], centroid[1], centroid[2]);
      centroids.push_back(centroid_3f);
    }

    for (jsk_recognition_utils::IntegerGraphMap::iterator it = connection_map.begin();
         it != connection_map.end();
         ++it) {
      // from = i
      int from_index = it->first;
      std::vector<int> the_connection_map = connection_map[from_index];
      for (size_t j = 0; j < the_connection_map.size(); j++) {
        int to_index = the_connection_map[j];
        //std::cout << "connection: " << from_index << " --> " << to_index << std::endl;
        Eigen::Vector3f from_point = centroids[from_index];
        Eigen::Vector3f to_point = centroids[to_index];
        geometry_msgs::Point from_point_ros, to_point_ros;
        jsk_recognition_utils::pointFromVectorToXYZ<Eigen::Vector3f, geometry_msgs::Point>(
          from_point, from_point_ros);
        jsk_recognition_utils::pointFromVectorToXYZ<Eigen::Vector3f, geometry_msgs::Point>(
          to_point, to_point_ros);
        connection_marker.points.push_back(from_point_ros);
        connection_marker.points.push_back(to_point_ros);
        connection_marker.colors.push_back(jsk_topic_tools::colorCategory20(from_index));
        connection_marker.colors.push_back(jsk_topic_tools::colorCategory20(from_index));
      }
    }
    pub_connection_marker_.publish(connection_marker);
  }
  
  void OrganizedMultiPlaneSegmentation::segmentFromNormals(
    pcl::PointCloud<PointT>::Ptr input,
    pcl::PointCloud<pcl::Normal>::Ptr normal,
    const std_msgs::Header& header)
  {
    PlanarRegionVector regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    ////////////////////////////////////////////////////////
    // segment planes based on pcl's organized multi plane
    // segmentation.
    ////////////////////////////////////////////////////////
    segmentOrganizedMultiPlanes(input, normal, regions, model_coefficients,
                                inlier_indices, labels, label_indices,
                                boundary_indices);
    std::vector<pcl::ModelCoefficients> fixed_model_coefficients;
    forceToDirectOrigin(model_coefficients, fixed_model_coefficients);
    model_coefficients = fixed_model_coefficients;
    
    original_plane_num_counter_.add(regions.size());
    publishSegmentationInformation(
      header, input,
      org_pub_, org_polygon_pub_, org_coefficients_pub_,
      inlier_indices, boundary_indices, model_coefficients);
    
    ////////////////////////////////////////////////////////
    // segmentation by PCL organized multiplane segmentation
    // is not enough. we "connect" planes like graph problem.
    ////////////////////////////////////////////////////////
    jsk_recognition_utils::IntegerGraphMap connection_map;
    connectPlanesMap(input, model_coefficients, boundary_indices, connection_map);
    publishMarkerOfConnection(connection_map, input, inlier_indices, header);
    std::vector<pcl::PointIndices> output_nonrefined_indices;
    std::vector<pcl::ModelCoefficients> output_nonrefined_coefficients;
    std::vector<pcl::PointCloud<PointT> > output_nonrefined_boundary_clouds;
    buildConnectedPlanes(input, header,
                         inlier_indices,
                         boundary_indices,
                         model_coefficients,
                         connection_map,
                         output_nonrefined_indices,
                         output_nonrefined_coefficients,
                         output_nonrefined_boundary_clouds);
    std::vector<pcl::ModelCoefficients> fixed_output_nonrefined_coefficients;
    forceToDirectOrigin(output_nonrefined_coefficients, fixed_output_nonrefined_coefficients);
    output_nonrefined_coefficients = fixed_output_nonrefined_coefficients;
    publishSegmentationInformation(
      header, input,
      pub_, polygon_pub_, coefficients_pub_,
      output_nonrefined_indices,
      output_nonrefined_boundary_clouds,
      output_nonrefined_coefficients);
    ////////////////////////////////////////////////////////
    // refine coefficients based on RANSAC
    ////////////////////////////////////////////////////////
    if (ransac_refine_coefficients_) {
      std::vector<pcl::PointIndices> refined_inliers;
      std::vector<pcl::ModelCoefficients> refined_coefficients;
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> refined_convexes;
      refineBasedOnRANSAC(
        input, output_nonrefined_indices, output_nonrefined_coefficients,
        refined_inliers, refined_coefficients, refined_convexes);
      std::vector<pcl::PointCloud<PointT> > refined_boundary_clouds;
      for (size_t i = 0; i < refined_convexes.size(); i++) {
        pcl::PointCloud<PointT> refined_boundary;
        refined_convexes[i]->boundariesToPointCloud(refined_boundary);
        refined_boundary_clouds.push_back(refined_boundary);
      }
      std::vector<pcl::ModelCoefficients> fixed_refined_coefficients;
      forceToDirectOrigin(refined_coefficients, fixed_refined_coefficients);
      refined_coefficients = fixed_refined_coefficients;
      publishSegmentationInformation(
        header, input,
        refined_pub_, refined_polygon_pub_, refined_coefficients_pub_,
        refined_inliers, refined_boundary_clouds, refined_coefficients);
    }
  }

  void OrganizedMultiPlaneSegmentation::refineBasedOnRANSAC(
    const pcl::PointCloud<PointT>::Ptr input,
    const std::vector<pcl::PointIndices>& input_indices,
    const std::vector<pcl::ModelCoefficients>& input_coefficients,
    std::vector<pcl::PointIndices>& output_indices,
    std::vector<pcl::ModelCoefficients>& output_coefficients,
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& output_convexes)
  {
    jsk_topic_tools::ScopedTimer timer
      = ransac_refinement_time_acc_.scopedTimer();
    for (size_t i = 0; i < input_indices.size(); i++) {
      pcl::PointIndices::Ptr input_indices_ptr
        = boost::make_shared<pcl::PointIndices>(input_indices[i]);
      Eigen::Vector3f normal(input_coefficients[i].values[0],
                             input_coefficients[i].values[1],
                             input_coefficients[i].values[2]);
      normal.normalize();
      ////////////////////////////////////////////////////////
      // run RANSAC
      ////////////////////////////////////////////////////////
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (ransac_refine_outlier_distance_threshold_);
      seg.setInputCloud(input);
      seg.setIndices(input_indices_ptr);
      seg.setMaxIterations (10000);
      seg.setAxis(normal);
      seg.setEpsAngle(pcl::deg2rad(20.0));
      pcl::PointIndices::Ptr refined_inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr refined_coefficients(new pcl::ModelCoefficients);
      seg.segment(*refined_inliers, *refined_coefficients);
      if (refined_inliers->indices.size() > 0) {
        ////////////////////////////////////////////////////////
        // compute boundaries from convex hull of
        ////////////////////////////////////////////////////////
        jsk_recognition_utils::ConvexPolygon::Ptr convex = jsk_recognition_utils::convexFromCoefficientsAndInliers<PointT>(
          input, refined_inliers, refined_coefficients);
        if (convex) {
          // check area threshold
          double area = convex->area();
          if (area > min_refined_area_threshold_ &&
              area < max_refined_area_threshold_) {
            output_convexes.push_back(convex);
            output_indices.push_back(*refined_inliers);
            output_coefficients.push_back(*refined_coefficients);
          }
        }
      }
    }
  }

  void OrganizedMultiPlaneSegmentation::estimateNormal(pcl::PointCloud<PointT>::Ptr input,
                                                       pcl::PointCloud<pcl::Normal>::Ptr output)
  {
    jsk_topic_tools::ScopedTimer timer = normal_estimation_time_acc_.scopedTimer();
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    if (estimation_method_ == 0) {
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    }
    else if (estimation_method_ == 1) {
     ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    }
    else if (estimation_method_ == 2) {
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    }
    else {
      NODELET_FATAL("unknown estimation method, force to use COVARIANCE_MATRIX: %d",
                    estimation_method_);
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    }

    if (border_policy_ignore_) {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_IGNORE);
    }
    else {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_MIRROR);
    }
    
    ne.setMaxDepthChangeFactor(max_depth_change_factor_);
    ne.setNormalSmoothingSize(normal_smoothing_size_);
    ne.setDepthDependentSmoothing(depth_dependent_smoothing_);
    ne.setInputCloud(input);
    ne.compute(*output);
  }
  
  void OrganizedMultiPlaneSegmentation::segment
  (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    
    // if estimate_normal_ is true, we run integral image normal estimation
    // before segmenting planes
    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());
    pcl::fromROSMsg(*msg, *input);
    
    if (estimate_normal_) {
      normal_estimation_vital_checker_->poke();
      estimateNormal(input, normal);
      // publish normal to ros
      if (publish_normal_) {
        sensor_msgs::PointCloud2 normal_ros_cloud;
        pcl::toROSMsg(*normal, normal_ros_cloud);
        normal_ros_cloud.header = msg->header;
        normal_pub_.publish(normal_ros_cloud);
      }
    }
    else {
      pcl::fromROSMsg(*msg, *normal);
    }
    
    segmentFromNormals(input, normal, msg->header);
    diagnostic_updater_->update();
  }

  void OrganizedMultiPlaneSegmentation::updateDiagnosticNormalEstimation(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (estimate_normal_) {
      bool alivep = normal_estimation_vital_checker_->isAlive();
      if (alivep) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "NormalEstimation running");
        jsk_topic_tools::addDiagnosticInformation(
          "Time to estimate normal", normal_estimation_time_acc_, stat);
        // normal estimation parameters
        if (estimation_method_ == 0) {
          stat.add("Estimation Method", "AVERAGE_3D_GRADIENT");
        }
        else if (estimation_method_ == 1) {
          stat.add("Estimation Method", "COVARIANCE_MATRIX");
        }
        else if (estimation_method_ == 2) {
          stat.add("Estimation Method", "AVERAGE_DEPTH_CHANGE");
        }
        if (border_policy_ignore_) {
          stat.add("Border Policy", "ignore");
        }
        else {
          stat.add("Border Policy", "mirror");
        }
        stat.add("Max Depth Change Factor", max_depth_change_factor_);
        stat.add("Normal Smoothing Size", normal_smoothing_size_);
        if (depth_dependent_smoothing_) {
          stat.add("Depth Dependent Smooting", "Enabled");
        }
        else {
          stat.add("Depth Dependent Smooting", "Disabled");
        }
        if (publish_normal_) {
          stat.add("Publish Normal", "Enabled");
        }
        else {
          stat.add("Publish Normal", "Disabled");
        }
      }
      else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     (boost::format("NormalEstimation not running for %f sec")
                      % normal_estimation_vital_checker_->deadSec()).str());
      }
      
    }
    else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "NormalEstimation is not activated");
    }
  }

  void OrganizedMultiPlaneSegmentation::updateDiagnosticPlaneSegmentation(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    bool alivep = plane_segmentation_vital_checker_->isAlive();
    if (alivep) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "PlaneSegmentation running");
      jsk_topic_tools::addDiagnosticInformation(
        "Time to segment planes", plane_segmentation_time_acc_, stat);
      if (ransac_refine_coefficients_) {
        jsk_topic_tools::addDiagnosticInformation(
          "Time to refine by RANSAC", ransac_refinement_time_acc_, stat);
      }
      stat.add("Minimum Inliers", min_size_);
      stat.add("Angular Threshold (rad)", angular_threshold_);
      stat.add("Angular Threshold (deg)", angular_threshold_ / M_PI * 180.0);
      stat.add("Distance Threshold", distance_threshold_);
      stat.add("Max Curvature", max_curvature_);
      if (ransac_refine_coefficients_) {
        stat.add("Use RANSAC refinement", "True");
        stat.add("RANSAC refinement distance threshold",
                 ransac_refine_outlier_distance_threshold_);
      }
      else {
        stat.add("Use RANSAC refinement", "False");
      }
      
      stat.add("Number of original segmented planes (Avg.)", 
               original_plane_num_counter_.mean());
      stat.add("Number of connected segmented planes (Avg.)", 
               connected_plane_num_counter_.mean());
    }
    else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   (boost::format("PlaneSegmentation not running for %f sec")
                    % plane_segmentation_vital_checker_->deadSec()).str());
    }
    
  }

  void OrganizedMultiPlaneSegmentation::updateDiagnostics(
    const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(mutex_);
    diagnostic_updater_->update();
  }
  
}
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OrganizedMultiPlaneSegmentation,
                        nodelet::Nodelet);
