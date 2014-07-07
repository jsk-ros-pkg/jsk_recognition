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

#include "jsk_pcl_ros/organized_multi_plane_segmentation.h"
#include "jsk_pcl_ros/ModelCoefficientsArray.h"
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <set>
#include <Eigen/StdVector>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/integral_image_normal.h>

#include <pluginlib/class_list_macros.h>

#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
typedef pcl_msgs::ModelCoefficients PCLModelCoefficientMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
typedef pcl::ModelCoefficients PCLModelCoefficientMsg;
#endif


namespace jsk_pcl_ros
{

  void OrganizedMultiPlaneSegmentation::onInit()
  {
    PCLNodelet::onInit();
    estimate_normal_ = false;
    pnh_->getParam("estimate_normal", estimate_normal_);
    // publishers
    pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>("output", 1);
    polygon_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_polygon", 1);
    coefficients_pub_ = pnh_->advertise<jsk_pcl_ros::ModelCoefficientsArray>("output_coefficients", 1);
    org_pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices>("output_nonconnected", 1);
    org_polygon_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_nonconnected_polygon", 1);
    org_coefficients_pub_ = pnh_->advertise<jsk_pcl_ros::ModelCoefficientsArray>("output_nonconnected_coefficients", 1);
    if (estimate_normal_) {
      normal_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_normal", 1);
    }
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OrganizedMultiPlaneSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);

    sub_ = pnh_->subscribe("input", 1, &OrganizedMultiPlaneSegmentation::segment, this);
  }

  void OrganizedMultiPlaneSegmentation::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    min_size_ = config.min_size;
    angular_threshold_ = config.angular_threshold;
    distance_threshold_ = config.distance_threshold;
    max_curvature_ = config.max_curvature;
    connect_plane_angle_threshold_ = config.connect_plane_angle_threshold;
    connect_plane_distance_threshold_ = config.connect_plane_distance_threshold;
    connect_distance_threshold_ = config.connect_distance_threshold;
    max_depth_change_factor_ = config.max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    depth_dependent_smoothing_ = config.depth_dependent_smoothing;
    estimation_method_ = config.estimation_method;
    border_policy_ignore_ = config.border_policy_ignore;
    publish_normal_ = config.publish_normal;
    //concave_alpha_ = config.concave_alpha;
  }

  void OrganizedMultiPlaneSegmentation::connectPlanesMap(const pcl::PointCloud<PointT>::Ptr& input,
                                                         const std::vector<pcl::ModelCoefficients>& model_coefficients,
                                                         const std::vector<pcl::PointIndices>& boundary_indices,
                                                         std::vector<std::map<size_t, bool> >& connection_map)
  {
    if (model_coefficients.size() == 0) {
      return;                   // do nothing
    }
      
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    connection_map.resize(model_coefficients.size());
    for (size_t i = 0; i < model_coefficients.size() - 1; i++) {
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
        NODELET_DEBUG("%lu - %lu distance: %f", i, j, fabs(fabs(a_distance) - fabs(b_distance)));
        if (fabs(fabs(a_distance) - fabs(b_distance)) > connect_plane_distance_threshold_) {
          continue;
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
        
        pcl::PointIndices::Ptr a_indices
          = boost::make_shared<pcl::PointIndices>(boundary_indices[i]);
        pcl::PointIndices::Ptr b_indices
          = boost::make_shared<pcl::PointIndices>(boundary_indices[j]);
        pcl::PointCloud<PointT> a_cloud, b_cloud;
        extract.setIndices(a_indices);
        extract.filter(a_cloud);
        extract.setIndices(b_indices);
        extract.filter(b_cloud);

        // compute the nearest neighbor of a_cloud and b_cloud,
        // and check the distance between them
        if (a_cloud.points.size() > 0) {
          pcl::KdTreeFLANN<PointT> kdtree;
          kdtree.setInputCloud(a_cloud.makeShared());
          bool foundp = false;
          for (size_t pi = 0; pi < b_cloud.points.size(); pi++) {
            PointT p = b_cloud.points[pi];
            std::vector<int> k_indices;
            std::vector<float> k_sqr_distances;
            if (kdtree.radiusSearch(p, connect_distance_threshold_, k_indices, k_sqr_distances, 1) > 0) {
              NODELET_DEBUG("%lu - %lu connected", i, j);
              foundp = true;
              break;
            }
          }
          if (foundp) {
            // i and j can be connected
            connection_map[i].insert(std::map<size_t, bool>::value_type(j, true));
          }
        }
      }
    }
  }

  void OrganizedMultiPlaneSegmentation::pclIndicesArrayToClusterPointIndices(const std::vector<pcl::PointIndices>& inlier_indices,
                                                                             const std_msgs::Header& header,
                                                                             jsk_pcl_ros::ClusterPointIndices& output_indices)
  {
    for (size_t i = 0; i < inlier_indices.size(); i++) {
      pcl::PointIndices inlier = inlier_indices[i];
      PCLIndicesMsg one_indices;
      one_indices.header = header;
      one_indices.indices = inlier.indices;
      output_indices.cluster_indices.push_back(one_indices);
    }
  }
  
  void OrganizedMultiPlaneSegmentation::pointCloudToPolygon(const pcl::PointCloud<PointT>& input,
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
  
  void OrganizedMultiPlaneSegmentation::buildConnectedPlanes(const pcl::PointCloud<PointT>::Ptr& input,
                                                             const std_msgs::Header& header,
                                                             const std::vector<pcl::PointIndices>& inlier_indices,
                                                             const std::vector<pcl::PointIndices>& boundary_indices,
                                                             const std::vector<pcl::ModelCoefficients>& model_coefficients,
                                                             std::vector<std::map<size_t, bool> > connection_map,
                                                             std::vector<pcl::PointIndices>& output_indices,
                                                             std::vector<pcl::ModelCoefficients>& output_coefficients,
                                                             std::vector<pcl::PointCloud<PointT> >& output_boundary_clouds)
  { 
    std::vector<std::set<int> > cloud_sets;
    for (size_t i = 0; i < connection_map.size(); i++) {
      bool i_done = false;
      // check i is included in cloud_sets
      for (size_t j = 0; j < cloud_sets.size(); j++) {
        if (cloud_sets[j].find(i) != cloud_sets[j].end()) {
          // i is included in cloud_sets[j]
          NODELET_DEBUG("%lu is included in cloud_sets[%lu]", i, j);
          for (std::map<size_t, bool>::iterator it = connection_map[i].begin();
               it != connection_map[i].end(); it++) {
            NODELET_DEBUG("%lu -> %lu", (*it).first, j);
            cloud_sets[j].insert((*it).first);
          }
          i_done = true;
          break;
        }
      }
      // i is not yet included in cloud_sets
      if (!i_done) {
        std::set<int> new_set;
        new_set.insert(i);
        for (std::map<size_t, bool>::iterator it = connection_map[i].begin();
             it != connection_map[i].end(); it++) {
          new_set.insert((*it).first);
        }
        cloud_sets.push_back(new_set);
      }
    }
      
    for (size_t i = 0; i < cloud_sets.size(); i++) {
      NODELET_DEBUG("%lu cloud", i);
      
      pcl::PointIndices one_indices;
      pcl::PointIndices one_boundaries;
      for (std::set<int>::iterator it = cloud_sets[i].begin();
           it != cloud_sets[i].end();
           it++) {
        NODELET_DEBUG("%lu includes %d", i, *it);
        pcl::PointIndices inlier = inlier_indices[*it];
        pcl::PointIndices boundary_inlier = boundary_indices[*it];
        // append indices...
        for (size_t j = 0; j < inlier.indices.size(); j++) {
          one_indices.indices.push_back(inlier.indices[j]);
        }
        for (size_t j = 0; j < boundary_inlier.indices.size(); j++) {
          one_boundaries.indices.push_back(boundary_inlier.indices[j]);
        }
      }
      if (one_indices.indices.size() == 0) {
        continue;
      }
      output_indices.push_back(one_indices);
      output_coefficients.push_back(model_coefficients[(*cloud_sets[i].begin())]);
      // estimate concave hull

      pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>());
      pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(one_boundaries);
      pcl::ProjectInliers<PointT> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setIndices (indices_ptr);
      proj.setInputCloud (input);
      proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients>(output_coefficients[i]));
      proj.filter (*cloud_projected);
      
      pcl::ConvexHull<PointT> chull;
      chull.setInputCloud(input);
      chull.setDimension(2);
      //chull.setAlpha(concave_alpha_);      // should be parameterized
      chull.setInputCloud(cloud_projected);
      pcl::PointCloud<PointT> chull_output;
      chull.reconstruct(chull_output);
      output_boundary_clouds.push_back(chull_output);
    }

  }

  void OrganizedMultiPlaneSegmentation::segmentFromNormals(pcl::PointCloud<PointT>::Ptr input,
                                                           pcl::PointCloud<pcl::Normal>::Ptr normal,
                                                           const std_msgs::Header& header)
  {
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(min_size_);
    mps.setAngularThreshold(angular_threshold_);
    mps.setDistanceThreshold(distance_threshold_);
    mps.setMaximumCurvature(max_curvature_);
    mps.setInputCloud(input);
    mps.setInputNormals(normal);

    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>());
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    
    mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    if (regions.size() == 0) {
      NODELET_DEBUG("no region is segmented");
    }

    {
      jsk_pcl_ros::ClusterPointIndices indices;
      jsk_pcl_ros::ModelCoefficientsArray coefficients_array;
      jsk_pcl_ros::PolygonArray polygon_array;
      indices.header = header;
      polygon_array.header = header;
      coefficients_array.header = header;
      pclIndicesArrayToClusterPointIndices(inlier_indices, header,
                                           indices);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(input);
      for (size_t i = 0; i < regions.size(); i++) {
        pcl::PointCloud<PointT> boundary_cloud;
        pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(boundary_indices[i]);
        extract.setIndices(indices_ptr);
        extract.filter(boundary_cloud);
        geometry_msgs::PolygonStamped polygon;
        pointCloudToPolygon(boundary_cloud, polygon.polygon);
        polygon.header = header;
        polygon_array.polygons.push_back(polygon);
      }
      org_pub_.publish(indices);
      org_polygon_pub_.publish(polygon_array);

      // coefficients
      for (size_t i = 0; i < model_coefficients.size(); i++) {
        PCLModelCoefficientMsg coefficient;
        coefficient.values = model_coefficients[i].values;
        coefficient.header = header;
        coefficients_array.coefficients.push_back(coefficient);
      }
      org_coefficients_pub_.publish(coefficients_array);
    }
    // connection
    // this might be slow...
    ros::Time before_connect_time = ros::Time::now();
    NODELET_DEBUG("checking %lu connection", regions.size());
    
    std::vector<std::map<size_t, bool> > connection_map;
    connectPlanesMap(input, model_coefficients, boundary_indices, connection_map);
    {
      std::vector<pcl::PointIndices> output_indices;
      std::vector<pcl::ModelCoefficients> output_coefficients;
      std::vector<pcl::PointCloud<PointT> > output_boundary_clouds;
    
      buildConnectedPlanes(input, header,
                           inlier_indices,
                           boundary_indices,
                           model_coefficients,
                           connection_map,
                           output_indices, output_coefficients, output_boundary_clouds);

      jsk_pcl_ros::ClusterPointIndices indices;
      jsk_pcl_ros::PolygonArray polygon_array;
      indices.header = header;
      polygon_array.header = header;
      pclIndicesArrayToClusterPointIndices(output_indices, header,
                                           indices);
      for (size_t i = 0; i < output_boundary_clouds.size(); i++) {
        geometry_msgs::PolygonStamped polygon;
        polygon.header = header;
        pointCloudToPolygon(output_boundary_clouds[i], polygon.polygon);
        polygon_array.polygons.push_back(polygon);
      }
      pub_.publish(indices);
      polygon_pub_.publish(polygon_array);
      jsk_pcl_ros::ModelCoefficientsArray coefficients_array;
      coefficients_array.header = header;
      for (size_t i = 0; i < output_coefficients.size(); i++) {
        PCLModelCoefficientMsg coefficient;
        coefficient.values = output_coefficients[i].values;
        coefficient.header = header;
        coefficients_array.coefficients.push_back(coefficient);
      }
      coefficients_pub_.publish(coefficients_array);
    }
    ros::Time after_connect_time = ros::Time::now();
    NODELET_DEBUG("checking connection done, %f", (after_connect_time - before_connect_time).toSec());
  }

  void OrganizedMultiPlaneSegmentation::estimateNormal(pcl::PointCloud<PointT>::Ptr input,
                                                       pcl::PointCloud<pcl::Normal>::Ptr output)
  {
    
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
      NODELET_FATAL("unknown estimation method: %d", estimation_method_);
      return;
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

    ros::Time before = ros::Time::now();
    ne.setInputCloud(input);
    ne.compute(*output);
    ros::Time after = ros::Time::now();
    NODELET_DEBUG("normal estimation: %f (%f Hz)", (after - before).toSec(), 1 / (after - before).toSec());
  }
  
  void OrganizedMultiPlaneSegmentation::segment
  (const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    // if estimate_normal_ is true, we run integral image normal estimation
    // before segmenting planes
    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());
    pcl::fromROSMsg(*msg, *input);
    if (estimate_normal_) {
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
  }
  
}

typedef jsk_pcl_ros::OrganizedMultiPlaneSegmentation OrganizedMultiPlaneSegmentation;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, OrganizedMultiPlaneSegmentation, OrganizedMultiPlaneSegmentation, nodelet::Nodelet);
