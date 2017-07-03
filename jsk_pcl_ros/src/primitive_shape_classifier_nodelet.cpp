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
    if (queue_size_ = config.queue_size) {
      queue_size_ = config.queue_size;
      if (isSubscribed()) {
        unsubscribe();
        subscribe();
      }
    }
  }
/*
  void PrimitiveShapeClassifier::process(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud,
                                         const sensor_msgs::PointCloud2::ConstPtr& ros_normal,
                                         const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices,
                                         const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons)
  {
    boost::mutex::scoped_lock lock(mutex_);

    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*ros_cloud, *input);

    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*ros_normal, *normal);

    pcl::ExtractIndices<PointT> ext_input;
    ext_input.setInputCloud(input);
    pcl::ExtractIndices<pcl::Normal> ext_normal;
    ext_normal.setInputCloud(normal);

    pcl::ProjectInliers<PointT> proj;

    for (size_t i = 0; i < indices->cluster_indices.size(); ++i)
    {
      pcl::IndicesPtr cluster_indices(new std::vector<int>(indices->cluster_indices[i].indices));

      pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
      ext_input.setIndices(cluster_indices);
      ext_input.filter(cluster_cloud);

      pcl::PointCloud<pcl::Normal>::Ptr cluster_normal(new pcl::PointCloud<pcl::Normal>);
      ext_normal.setIndices(cluster_indices);
      ext_normal.filter(cluster_normal);

      pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> b;
      b.setInputCloud(cluster_cloud);
      b.setInputNormals(cluster_normal);
      b.setSearchMethod(typename pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>));
      b.setAngleThreshold(DEG2RAD(70));
      b.setRadiusSearch(0.03);

      pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
      b.compute(boundaries);

      pcl::PointCloud<PointT>::Ptr boundary_cloud(new pcl::PointCloud<PointT>);
      for (size_t j = 0; j < boundaries->points.size(); ++k)
      {
        if ((int)boundaries->points[k].boundary_point)
          boundary_cloud->points.push_back(cluster_cloud->points[k]);
      }
      boundary_cloud->width  = boundary_cloud->points.size();
      boundary_cloud->height = 1;

      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(boundary_cloud);

    }
  }
*/
  void
  PrimitiveShapeClassifier::process(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud,
                                    const sensor_msgs::PointCloud2::ConstPtr& ros_normal,
                                    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& ros_indices,
                                    const jsk_recognition_msgs::PolygonArray::ConstPtr& ros_polygons)
  {
    boost::mutex::scoped_lock lock(mutex_);

    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*ros_cloud, *input);

    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*ros_normal, *normal);

    pcl::ExtractIndices<PointT> ext_input;
    ext_input.setInputCloud(input);
    pcl::ExtractIndices<pcl::Normal> ext_normal;
    ext_normal.setInputCloud(normal);

    std::vector<jsk_recognition_utils::Polygon::Ptr> polygons
      = jsk_recognition_utils::Polygon::fromROSMsg(*ros_polygons, Eigen::Affine3f());

    // for visualization
    pcl::PointCloud<PointT>::Ptr projected_cloud(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointIndices::Ptr> boundary_indices(ros_indices->cluster_indices.size());

    for (size_t i = 0; i < ros_indices->cluster_indices.size(); ++i)
    {
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      indices->indices = ros_indices->cluster_indices[i].indices;

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

      boundary_indices.push_back(b);
      *projected_cloud += *pc;
    }

    // publish results
    sensor_msgs::PointCloud2 ros_projected_cloud;
    pcl::toROSMsg(*projected_cloud, ros_projected_cloud);
    ros_projected_cloud.header = ros_cloud->header;
    pub_projected_cloud_.publish(ros_projected_cloud);

    jsk_recognition_msgs::ClusterPointIndices ros_boundary_indices;
    ros_boundary_indices.header = ros_cloud->header;
    for (size_t i = 0; i < boundary_indices.size(); ++i)
    {
      pcl_msgs::PointIndices ri;
      pcl_conversions::moveFromPCL(*boundary_indices[i], ri);
      ros_boundary_indices.cluster_indices.push_back(ri);
    }
    pub_boundary_indices_.publish(ros_boundary_indices);

    // TODO: publish class
  }

  void
  PrimitiveShapeClassifier::estimate(const pcl::PointCloud<PointT>::Ptr& cloud,
                                     const pcl::PointCloud<pcl::Normal>::Ptr& normal,
                                     const pcl::ModelCoefficients::Ptr& plane,
                                     pcl::PointIndices::Ptr& boundary_indices, /* for debug */
                                     pcl::PointCloud<PointT>::Ptr& projected_cloud, /* for debug */
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

    // project to plane
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
    seg.setMaxIterations(500);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setDistanceThreshold(0.005);
    seg.setRadiusLimits(0.025, 0.13);
    seg.segment(*circle_inliers, *circle_coeffs);

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setDistanceThreshold(0.005);
    seg.segment(*line_inliers, *line_coeffs);

    // compute likelihood
    circle_likelihood =
      1.0 * circle_inliers->indices.size() / projected_cloud->points.size();
    box_likelihood =
      1.0 * line_inliers->indices.size() / projected_cloud->points.size();

    NODELET_INFO_STREAM("Projected cloud has " << projected_cloud->points.size() << " points");
    NODELET_INFO_STREAM(circle_inliers->indices.size() << " circle inliers found");
    NODELET_INFO_STREAM("confidence: " << circle_likelihood);
    NODELET_INFO_STREAM(line_inliers->indices.size() << " line inliers found");
    NODELET_INFO_STREAM("confidence: " << box_likelihood);
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
/*
  void PrimitiveShapeClassifier::computeNormal(const pcl::PointCloud<PointT>::Ptr& input,
                                               pcl::PointCloud<pcl::Normal>::Ptr& output)
  {
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> n;
    switch (normal_estimation_method_) {
    case 0:
      n.setNormalEstimationMethod(n.AVERAGE_3D_GRADIENT);
      break;
    case 1:
      n.setNormalEstimationMethod(n.COVARIANCE_MATRIX);
      break;
    case 2:
      n.setNormalEstimationMethod(n.AVERAGE_DEPTH_CHANGE);
      break;
    default:
      n.setNormalEstimationMethod(n.COVARIANCE_MATRIX);
      break;
    }

    switch (normal_border_policy_) {
    case 0:
      n.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_IGNORE);
      break;
    default:
      n.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_MIRROR);
      break;
    }

    n.setMaxDepthChangeFactor(max_depth_change_factor_);
    n.setNormalSmoothingSize(normal_smoothing_size_);
    n.setDepthDependentSmoothing(depth_dependent_smoothing_);
    n.setInputCloud(input);
    n.compute(output);
  }
*/
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::PrimitiveShapeClassifier, nodelet::Nodelet);
