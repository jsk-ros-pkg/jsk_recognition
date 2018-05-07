// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
/*
 * primitive_shape_classifier_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_pcl_ros/primitive_shape_classifier.h>
#include <algorithm>
#include <iterator>
#include <pcl/common/centroid.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace jsk_pcl_ros
{
  void PrimitiveShapeClassifier::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&PrimitiveShapeClassifier::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_class_ = advertise<jsk_recognition_msgs::ClassificationResult>(*pnh_, "output", 1);
    pub_boundary_indices_ =
      advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "debug/boundary_indices", 1);
    pub_projected_cloud_ =
      advertise<sensor_msgs::PointCloud2>(*pnh_, "debug/projected_cloud", 1);

    onInitPostProcess();
  }

  void PrimitiveShapeClassifier::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_normal_.subscribe(*pnh_, "input/normal", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sub_polygons_.subscribe(*pnh_, "input/polygons", 1);

    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
    sync_->connectInput(sub_cloud_, sub_normal_, sub_indices_, sub_polygons_);
    sync_->registerCallback(boost::bind(&PrimitiveShapeClassifier::process, this, _1, _2, _3, _4));
  }

  void PrimitiveShapeClassifier::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_normal_.unsubscribe();
    sub_indices_.unsubscribe();
    sub_polygons_.unsubscribe();
  }

  void PrimitiveShapeClassifier::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_points_num_ = config.min_points_num;

    sac_max_iterations_ = config.sac_max_iterations;
    sac_distance_threshold_ = config.sac_distance_threshold;
    if (config.sac_radius_limit_min < config.sac_radius_limit_max) {
      sac_radius_limit_min_ = config.sac_radius_limit_min;
      sac_radius_limit_max_ = config.sac_radius_limit_max;
    } else {
      config.sac_radius_limit_min = sac_radius_limit_min_;
      config.sac_radius_limit_max = sac_radius_limit_max_;
    }

    box_threshold_ = config.box_threshold;
    circle_threshold_ = config.circle_threshold;

    if (queue_size_ != config.queue_size) {
      queue_size_ = config.queue_size;
      if (isSubscribed()) {
        unsubscribe();
        subscribe();
      }
    }
  }

  bool
  PrimitiveShapeClassifier::checkFrameId(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud,
                                         const sensor_msgs::PointCloud2::ConstPtr& ros_normal,
                                         const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& ros_indices,
                                         const jsk_recognition_msgs::PolygonArray::ConstPtr& ros_polygons)
  {
    std::string cloud_topic = ros::names::resolve(sub_cloud_.getTopic());
    std::string normal_topic = ros::names::resolve(sub_normal_.getTopic());
    std::string indices_topic = ros::names::resolve(sub_indices_.getTopic());
    std::string polygons_topic = ros::names::resolve(sub_polygons_.getTopic());
    if (!jsk_recognition_utils::isSameFrameId(ros_cloud->header, ros_normal->header)) {
      NODELET_ERROR_STREAM(cloud_topic << " and " << normal_topic << " must have same frame_id");
      return false;
    }
    if (!jsk_recognition_utils::isSameFrameId(ros_cloud->header, ros_indices->header)) {
      NODELET_ERROR_STREAM(cloud_topic << " and " << indices_topic << " must have same frame_id");
      return false;
    }
    if (!jsk_recognition_utils::isSameFrameId(ros_cloud->header, ros_polygons->header)) {
      NODELET_ERROR_STREAM(cloud_topic << " and " << polygons_topic << " must have same frame_id");
      return false;
    }
    NODELET_DEBUG_STREAM("Frame id is ok: " << ros_cloud->header.frame_id);
    return true;
  }

  void
  PrimitiveShapeClassifier::process(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud,
                                    const sensor_msgs::PointCloud2::ConstPtr& ros_normal,
                                    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& ros_indices,
                                    const jsk_recognition_msgs::PolygonArray::ConstPtr& ros_polygons)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!checkFrameId(ros_cloud, ros_normal, ros_indices, ros_polygons)) return;

    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*ros_cloud, *input);

    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*ros_normal, *normal);

    pcl::ExtractIndices<PointT> ext_input;
    ext_input.setInputCloud(input);
    pcl::ExtractIndices<pcl::Normal> ext_normal;
    ext_normal.setInputCloud(normal);

    std::vector<jsk_recognition_utils::Polygon::Ptr> polygons
      = jsk_recognition_utils::Polygon::fromROSMsg(*ros_polygons);

    jsk_recognition_msgs::ClassificationResult result;
    result.header = ros_cloud->header;
    result.classifier = "primitive_shape_classifier";
    result.target_names.push_back("box");
    result.target_names.push_back("circle");
    result.target_names.push_back("other");

    pcl::PointCloud<PointT>::Ptr projected_cloud(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointIndices::Ptr> boundary_indices;

    NODELET_DEBUG_STREAM("Cluster num: " << ros_indices->cluster_indices.size());
    for (size_t i = 0; i < ros_indices->cluster_indices.size(); ++i)
    {
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      indices->indices = ros_indices->cluster_indices[i].indices;
      NODELET_DEBUG_STREAM("Estimating cluster #" << i << " (" << indices->indices.size() << " points)");

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
      ext_input.setIndices(indices);
      ext_input.filter(*cluster_cloud);

      pcl::PointCloud<pcl::Normal>::Ptr cluster_normal(new pcl::PointCloud<pcl::Normal>);
      ext_normal.setIndices(indices);
      ext_normal.filter(*cluster_normal);

      pcl::ModelCoefficients::Ptr support_plane(new pcl::ModelCoefficients);
      if (!getSupportPlane(cluster_cloud, polygons, support_plane))
      {
        NODELET_ERROR_STREAM("cloud " << i << " has no support plane. skipped");
        continue;
      }

      pcl::PointIndices::Ptr b(new pcl::PointIndices);
      pcl::PointCloud<PointT>::Ptr pc(new pcl::PointCloud<PointT>);
      float circle_likelihood, box_likelihood;
      estimate(cluster_cloud, cluster_normal, support_plane,
               b, pc,
               circle_likelihood, box_likelihood);
      boundary_indices.push_back(std::move(b));
      *projected_cloud += *pc;

      if (circle_likelihood > circle_threshold_) {
        // circle
        result.labels.push_back(1);
        result.label_names.push_back("circle");
        result.label_proba.push_back(circle_likelihood);
      } else if (box_likelihood > box_threshold_) {
        // box
        result.labels.push_back(0);
        result.label_names.push_back("box");
        result.label_proba.push_back(box_likelihood);
      } else {
        // other
        result.labels.push_back(3);
        result.label_names.push_back("other");
        result.label_proba.push_back(0.0);
      }
    }

    // publish results
    if (pub_projected_cloud_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 ros_projected_cloud;
      pcl::toROSMsg(*projected_cloud, ros_projected_cloud);
      ros_projected_cloud.header = ros_cloud->header;
      pub_projected_cloud_.publish(ros_projected_cloud);
    }

    if (pub_boundary_indices_.getNumSubscribers() > 0) {
      jsk_recognition_msgs::ClusterPointIndices ros_boundary_indices;
      ros_boundary_indices.header = ros_cloud->header;
      for (size_t i = 0; i < boundary_indices.size(); ++i)
      {
        pcl_msgs::PointIndices ri;
        pcl_conversions::moveFromPCL(*boundary_indices[i], ri);
        ros_boundary_indices.cluster_indices.push_back(ri);
      }
      pub_boundary_indices_.publish(ros_boundary_indices);
    }

    pub_class_.publish(result);
  }

  bool
  PrimitiveShapeClassifier::estimate(const pcl::PointCloud<PointT>::Ptr& cloud,
                                     const pcl::PointCloud<pcl::Normal>::Ptr& normal,
                                     const pcl::ModelCoefficients::Ptr& plane,
                                     pcl::PointIndices::Ptr& boundary_indices,
                                     pcl::PointCloud<PointT>::Ptr& projected_cloud,
                                     float& circle_likelihood,
                                     float& box_likelihood)
  {
    // estimate boundaries
    pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
    pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> b;
    b.setInputCloud(cloud);
    b.setInputNormals(normal);
    b.setSearchMethod(typename pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));
    b.setAngleThreshold(DEG2RAD(70));
    b.setRadiusSearch(0.03);
    b.compute(*boundaries);

    // set boundaries as indices
    for (size_t i = 0; i < boundaries->points.size(); ++i)
      if ((int)boundaries->points[i].boundary_point)
        boundary_indices->indices.push_back(i);

    // extract boundaries
    pcl::PointCloud<PointT>::Ptr boundary_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> ext;
    ext.setInputCloud(cloud);
    ext.setIndices(boundary_indices);
    ext.filter(*boundary_cloud);

    // thresholding with min points num
    if (boundary_indices->indices.size() < min_points_num_)
      return false;

    // project to supporting plane
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(boundary_cloud);
    proj.setModelCoefficients(plane);
    proj.filter(*projected_cloud);

    // estimate circles
    pcl::PointIndices::Ptr      circle_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr circle_coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      line_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr line_coeffs(new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> seg;
    seg.setInputCloud(projected_cloud);

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(sac_max_iterations_);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setDistanceThreshold(sac_distance_threshold_);
    seg.setRadiusLimits(sac_radius_limit_min_, sac_radius_limit_max_);
    seg.segment(*circle_inliers, *circle_coeffs);

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(sac_max_iterations_);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setDistanceThreshold(sac_distance_threshold_);
    seg.segment(*line_inliers, *line_coeffs);

    // compute likelihood
    circle_likelihood =
      1.0f * circle_inliers->indices.size() / projected_cloud->points.size();
    box_likelihood =
      1.0f * line_inliers->indices.size() / projected_cloud->points.size();

    NODELET_DEBUG_STREAM("Projected cloud has " << projected_cloud->points.size() << " points");
    NODELET_DEBUG_STREAM(circle_inliers->indices.size() << " circle inliers found");
    NODELET_DEBUG_STREAM("circle confidence: " << circle_likelihood);
    NODELET_DEBUG_STREAM(line_inliers->indices.size() << " line inliers found");
    NODELET_DEBUG_STREAM("box confidence: " << box_likelihood);

    return true;
  }

  bool
  PrimitiveShapeClassifier::getSupportPlane(const pcl::PointCloud<PointT>::Ptr& cloud,
                                            const std::vector<jsk_recognition_utils::Polygon::Ptr>& polygons,
                                            pcl::ModelCoefficients::Ptr& coeff)
  {
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*cloud, c);
    Eigen::Vector3f centroid(c[0], c[1], c[2]);
    Eigen::Vector3f projected;
    for (size_t i = 0; i < polygons.size(); ++i)
    {
      jsk_recognition_utils::Polygon::Ptr p = polygons[i];
      p->project(centroid, projected);
      if (p->isInside(projected)) {
        p->toCoefficients(coeff->values);
        return true;
      }
    }
    return false;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::PrimitiveShapeClassifier, nodelet::Nodelet);
