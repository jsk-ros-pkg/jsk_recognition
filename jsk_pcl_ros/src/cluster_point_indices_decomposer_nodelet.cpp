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
 *     disclaimer in the documentation and/or other materials provided
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

#include "jsk_pcl_ros/cluster_point_indices_decomposer.h"
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <boost/format.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/irange.hpp>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/pca.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_topic_tools/color_utils.h>
#include <Eigen/Geometry> 
#include <geometry_msgs/PoseArray.h>

#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/pcl_util.h"

namespace jsk_pcl_ros
{
  void ClusterPointIndicesDecomposerZAxis::onInit()
  {
    ClusterPointIndicesDecomposer::onInit();
    sort_by_ = "z_axis";
  }

  ClusterPointIndicesDecomposer::~ClusterPointIndicesDecomposer() {
    // message_filters::Synchronizer needs to be called reset
    // before message_filters::Subscriber is freed.
    // Calling reset fixes the following error on shutdown of the nodelet:
    // terminate called after throwing an instance of
    // 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    //     what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    // Also see https://github.com/ros/ros_comm/issues/720 .
    sync_.reset();
    async_.reset();
    sync_align_.reset();
    async_align_.reset();
  }

  void ClusterPointIndicesDecomposer::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("publish_tf", publish_tf_, false);
    if (publish_tf_) {
      br_.reset(new tf::TransformBroadcaster);
    }
    if (!pnh_->getParam("tf_prefix", tf_prefix_))
    {
      if (publish_tf_) {
        NODELET_WARN("~tf_prefix is not specified, using %s", getName().c_str());
      }
      tf_prefix_ = getName();
    }

    // fixed parameters
    pnh_->param("approximate_sync", use_async_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("publish_clouds", publish_clouds_, false);
    if (publish_clouds_) {
      NODELET_WARN("~output%%02d are not published before subscribed, you should subscribe ~debug_output in debuging.");
    }
    pnh_->param("align_boxes", align_boxes_, false);
    if (align_boxes_) {
      pnh_->param("align_boxes_with_plane", align_boxes_with_plane_, true);
    } else if (pnh_->hasParam("align_boxes_with_plane")) {
      NODELET_WARN("Rosparam ~align_boxes_with_plane is used only with ~align_boxes:=true, so ignoring it.");
    }
    if (align_boxes_ && !align_boxes_with_plane_) {
      tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
      if (!pnh_->getParam("target_frame_id", target_frame_id_)) {
        NODELET_FATAL("~target_frame_id is not specified");
        return;
      }
      NODELET_INFO("Aligning bboxes with '%s' using tf transform.", target_frame_id_.c_str());
    } else if (pnh_->hasParam("target_frame_id")) {
      NODELET_WARN("Rosparam ~target_frame_id is used only with ~align_boxes:=true and ~align_boxes_with_plane:=true, so ignoring it.");
    }
    pnh_->param("force_to_flip_z_axis", force_to_flip_z_axis_, true);
    pnh_->param<std::string>("sort_by", sort_by_, "input_indices");
    // dynamic_reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ClusterPointIndicesDecomposer::configCallback, this, _1, _2);
    srv_->setCallback(f);

    negative_indices_pub_ = advertise<pcl_msgs::PointIndices>(*pnh_, "negative_indices", 1);
    pc_pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "debug_output", 1);
    box_pub_ = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "boxes", 1);
    mask_pub_ = advertise<sensor_msgs::Image>(*pnh_, "mask", 1);
    label_pub_ = advertise<sensor_msgs::Image>(*pnh_, "label", 1);
    centers_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "centroid_pose_array", 1);
    indices_pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "cluster_indices", 1);

    onInitPostProcess();
  }

  void ClusterPointIndicesDecomposer::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    max_size_ = config.max_size;
    min_size_ = config.min_size;
    use_pca_ = config.use_pca;
    fill_boxes_label_with_nearest_plane_index_ = config.fill_boxes_label_with_nearest_plane_index;
  }

  void ClusterPointIndicesDecomposer::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_target_.subscribe(*pnh_, "target", 1);
    if (align_boxes_ && align_boxes_with_plane_) {
      sub_polygons_.subscribe(*pnh_, "align_planes", 1);
      sub_coefficients_.subscribe(*pnh_, "align_planes_coefficients", 1);
      if (use_async_) {
        async_align_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncAlignPolicy> >(queue_size_);
        async_align_->connectInput(sub_input_, sub_target_, sub_polygons_, sub_coefficients_);
        async_align_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2, _3, _4));
      }
      else {
        sync_align_ = boost::make_shared<message_filters::Synchronizer<SyncAlignPolicy> >(queue_size_);
        sync_align_->connectInput(sub_input_, sub_target_, sub_polygons_, sub_coefficients_);
        sync_align_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2, _3, _4));
      }
    }
    else {
      if (use_async_) {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
        async_->connectInput(sub_input_, sub_target_);
        async_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2));
      }
      else {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_input_, sub_target_);
        sync_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2));
      }
    }
  }

  void ClusterPointIndicesDecomposer::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_target_.unsubscribe();
    if (align_boxes_ && align_boxes_with_plane_) {
      sub_polygons_.unsubscribe();
      sub_coefficients_.unsubscribe();
    }
  }

  void ClusterPointIndicesDecomposer::sortIndicesOrder(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
    const std::vector<pcl::IndicesPtr> indices_array,
    std::vector<size_t>* argsort)
  {
    std::string sort_by = sort_by_;
    bool reverse = false;
    if (sort_by.compare(0, 1, "-") == 0)
    {
      reverse = true;
      sort_by = sort_by.substr(1, sort_by.size() - 1);
    }
    if (sort_by == "input_indices")
    {
      sortIndicesOrderByIndices(input, indices_array, argsort);
    }
    else if (sort_by == "z_axis")
    {
      sortIndicesOrderByZAxis(input, indices_array, argsort);
    }
    else if (sort_by == "cloud_size")
    {
      sortIndicesOrderByCloudSize(input, indices_array, argsort);
    }
    else
    {
      NODELET_WARN_ONCE("Unsupported ~sort_by param is specified '%s', "
                        "so using the default: 'input_indices'", sort_by_.c_str());
      sortIndicesOrderByIndices(input, indices_array, argsort);
      return;
    }
    if (reverse)
    {
      std::reverse((*argsort).begin(), (*argsort).end());
    }
  }

  void ClusterPointIndicesDecomposer::sortIndicesOrderByIndices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
    const std::vector<pcl::IndicesPtr> indices_array,
    std::vector<size_t>* argsort)
  {
    (*argsort).resize(indices_array.size());
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      (*argsort)[i] = i;
    }
  }

  void ClusterPointIndicesDecomposer::sortIndicesOrderByZAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
    const std::vector<pcl::IndicesPtr> indices_array,
    std::vector<size_t>* argsort)
  {
    std::vector<double> z_values;
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(input);
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      Eigen::Vector4f center;
      ex.setIndices(indices_array[i]);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      ex.filter(*cloud);
      //
      std::vector<int> nan_indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);
      //
      pcl::compute3DCentroid(*cloud, center);
      z_values.push_back(center[2]); // only focus on z value
    }

    // sort centroids
    // https://stackoverflow.com/a/12399290
    (*argsort).resize(indices_array.size());
    std::iota(argsort->begin(), argsort->end(), 0);
    std::sort(argsort->begin(), argsort->end(),
              [&z_values](size_t i1, size_t i2) {return z_values[i1] < z_values[i2];});
  }

  void ClusterPointIndicesDecomposer::sortIndicesOrderByCloudSize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
    const std::vector<pcl::IndicesPtr> indices_array,
    std::vector<size_t>* argsort)
  {
    std::vector<double> cloud_sizes;
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(input);
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      Eigen::Vector4f center;
      ex.setIndices(indices_array[i]);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      ex.filter(*cloud);
      //
      std::vector<int> nan_indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, nan_indices);
      //
      double cloud_size = static_cast<double>(cloud->points.size());
      cloud_sizes.push_back(cloud_size);
    }

    // sort clouds
    (*argsort).resize(indices_array.size());
    std::iota(argsort->begin(), argsort->end(), 0);
    std::sort(argsort->begin(), argsort->end(),
              [&cloud_sizes](size_t i1, size_t i2) {return cloud_sizes[i1] < cloud_sizes[i2];});
  }

  void ClusterPointIndicesDecomposer::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "ClusterPointIndicesDecomposer running");
      jsk_topic_tools::addDiagnosticBooleanStat(
        "publish_clouds", publish_clouds_, stat);
      jsk_topic_tools::addDiagnosticBooleanStat(
        "publish_tf", publish_tf_, stat);
      jsk_topic_tools::addDiagnosticBooleanStat(
        "use_pca", use_pca_, stat);
      jsk_topic_tools::addDiagnosticBooleanStat(
        "align_boxes", align_boxes_, stat);
      stat.add("tf_prefix", tf_prefix_);
      stat.add("Clusters (Ave.)", cluster_counter_.mean());
    }
    DiagnosticNodelet::updateDiagnostic(stat);
  }
  
  int ClusterPointIndicesDecomposer::findNearestPlane(
    const Eigen::Vector4f& center,
    const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
    const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients)
  {
    double min_distance = DBL_MAX;
    int nearest_index = -1;
    for (size_t i = 0; i < coefficients->coefficients.size(); i++) {
      geometry_msgs::PolygonStamped polygon_msg = planes->polygons[i];
      jsk_recognition_utils::Vertices vertices;
      for (size_t j = 0; j < polygon_msg.polygon.points.size(); j++) {
        jsk_recognition_utils::Vertex v;
        v[0] = polygon_msg.polygon.points[j].x;
        v[1] = polygon_msg.polygon.points[j].y;
        v[2] = polygon_msg.polygon.points[j].z;
        vertices.push_back(v);
      }
      jsk_recognition_utils::ConvexPolygon p(vertices, coefficients->coefficients[i].values);
      double distance = p.distanceToPoint(center);
      if (distance < min_distance) {
        min_distance = distance;
        nearest_index = i;
      }
    }
    return nearest_index;
  }

  bool ClusterPointIndicesDecomposer::transformPointCloudToAlignWithPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud_transformed,
    const Eigen::Vector4f center,
    const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
    const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
    Eigen::Matrix4f& m4,
    Eigen::Quaternionf& q,
    int& nearest_plane_index)
  {
    nearest_plane_index = findNearestPlane(center, planes, coefficients);
    if (nearest_plane_index == -1) {
      segmented_cloud_transformed = segmented_cloud;
      NODELET_ERROR("no planes to align boxes are given");
    }
    else {
      Eigen::Vector3f normal, z_axis;
      if (force_to_flip_z_axis_) {
        normal[0] = - coefficients->coefficients[nearest_plane_index].values[0];
        normal[1] = - coefficients->coefficients[nearest_plane_index].values[1];
        normal[2] = - coefficients->coefficients[nearest_plane_index].values[2];
      }
      else {
        normal[0] = coefficients->coefficients[nearest_plane_index].values[0];
        normal[1] = coefficients->coefficients[nearest_plane_index].values[1];
        normal[2] = coefficients->coefficients[nearest_plane_index].values[2];
      }
      normal = normal.normalized();
      Eigen::Quaternionf rot;
      rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal);
      Eigen::AngleAxisf rotation_angle_axis(rot);
      Eigen::Vector3f rotation_axis = rotation_angle_axis.axis();
      double theta = rotation_angle_axis.angle();
      if (std::isnan(theta) ||
          std::isnan(rotation_axis[0]) ||
          std::isnan(rotation_axis[1]) ||
          std::isnan(rotation_axis[2])) {
        segmented_cloud_transformed = segmented_cloud;
        NODELET_ERROR("cannot compute angle to align the point cloud: [%f, %f, %f], [%f, %f, %f]",
                      z_axis[0], z_axis[1], z_axis[2],
                      normal[0], normal[1], normal[2]);
      }
      else {
        Eigen::Matrix3f m = Eigen::Matrix3f::Identity() * rot;
        if (use_pca_) {
          // first project points to the plane
          pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud
            (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::ProjectInliers<pcl::PointXYZ> proj;
          proj.setModelType (pcl::SACMODEL_PLANE);
          pcl::ModelCoefficients::Ptr
            plane_coefficients (new pcl::ModelCoefficients);
          plane_coefficients->values
            = coefficients->coefficients[nearest_plane_index].values;
          proj.setModelCoefficients(plane_coefficients);
          proj.setInputCloud(segmented_cloud);
          proj.filter(*projected_cloud);
          if (projected_cloud->points.size() >= 3) {
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(projected_cloud);
            Eigen::Matrix3f eigen = pca.getEigenVectors();
            m.col(0) = eigen.col(0);
            m.col(1) = eigen.col(1);
            // flip axis to satisfy right-handed system
            if (m.col(0).cross(m.col(1)).dot(m.col(2)) < 0) {
              m.col(0) = - m.col(0);
            }
            if (m.col(0).dot(Eigen::Vector3f::UnitX()) < 0) {
              // rotate around z
              m = m * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
            }
          }
          else {
            NODELET_ERROR("Too small indices for PCA computation");
            return false;
          }
        }
        // m4 <- m
        for (size_t row = 0; row < 3; row++) {
          for (size_t column = 0; column < 3; column++) {
            m4(row, column) = m(row, column);
          }
        }
        q = m;
        q.normalize();
        Eigen::Matrix4f inv_m = m4.inverse();
        pcl::transformPointCloud(*segmented_cloud, *segmented_cloud_transformed, inv_m);
      }
    }
    return true;
  }

  bool ClusterPointIndicesDecomposer::computeCenterAndBoundingBox
  (const pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud,
   const std_msgs::Header header,
   const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
   const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
   geometry_msgs::Pose& center_pose_msg,
   jsk_recognition_msgs::BoundingBox& bounding_box,
   bool& publish_tf)
  {
    bounding_box.header = header;
    if (segmented_cloud->points.size() == 0) {
      publish_tf = false;
      NODELET_WARN("segmented cloud size is zero");
      return true;
    }

    bool is_center_valid = false;
    Eigen::Vector4f center;
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      segmented_cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
    segmented_cloud_transformed->is_dense = segmented_cloud->is_dense;
    // align boxes if possible
    Eigen::Matrix4f m4 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    int nearest_plane_index = 0;
    if (align_boxes_) {
      if (align_boxes_with_plane_) {
        is_center_valid = pcl::compute3DCentroid(*segmented_cloud, center) != 0;
        bool success = transformPointCloudToAlignWithPlane(segmented_cloud, segmented_cloud_transformed,
                                                           center, planes, coefficients, m4, q,
                                                           nearest_plane_index);
        if (!success) {
          return false;
        }
      }
      else {
        // transform point cloud to target frame
        tf::StampedTransform tf_transform;
        try {
          tf_transform = jsk_recognition_utils::lookupTransformWithDuration(
            /*listener=*/tf_listener_,
            /*to_frame=*/header.frame_id,                // box origin
            /*from_frame=*/target_frame_id_,  // sensor origin
            /*time=*/header.stamp,
            /*duration=*/ros::Duration(1.0));
        }
        catch (tf2::TransformException &e) {
          NODELET_ERROR("Transform error: %s", e.what());
          return false;
        }
        Eigen::Affine3f transform;
        tf::transformTFToEigen(tf_transform, transform);
        pcl::transformPointCloud(*segmented_cloud, *segmented_cloud, transform);

        is_center_valid = pcl::compute3DCentroid(*segmented_cloud, center) != 0;

        // compute planes from target frame
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*segmented_cloud, min_pt, max_pt);
        //
        pcl_msgs::ModelCoefficients coef_by_frame;
        coef_by_frame.values.push_back(0);
        coef_by_frame.values.push_back(0);
        coef_by_frame.values.push_back(1);
        coef_by_frame.values.push_back(- min_pt.z);
        jsk_recognition_msgs::ModelCoefficientsArray::Ptr coefficients_by_frame(
          new jsk_recognition_msgs::ModelCoefficientsArray);
        coefficients_by_frame->header.frame_id = target_frame_id_;
        coefficients_by_frame->coefficients.push_back(coef_by_frame);
        //
        geometry_msgs::PolygonStamped plane_by_frame;
        plane_by_frame.header.frame_id = target_frame_id_;
        geometry_msgs::Point32 point;
        point.z = min_pt.z;
        point.x = min_pt.x;
        point.y = min_pt.y;
        plane_by_frame.polygon.points.push_back(point);
        point.x = max_pt.x;
        point.y = min_pt.y;
        plane_by_frame.polygon.points.push_back(point);
        point.x = max_pt.x;
        point.y = max_pt.y;
        plane_by_frame.polygon.points.push_back(point);
        point.x = min_pt.x;
        point.y = max_pt.y;
        plane_by_frame.polygon.points.push_back(point);
        jsk_recognition_msgs::PolygonArray::Ptr planes_by_frame(
          new jsk_recognition_msgs::PolygonArray);
        planes_by_frame->header.frame_id = target_frame_id_;
        planes_by_frame->polygons.push_back(plane_by_frame);
        //
        bool success = transformPointCloudToAlignWithPlane(segmented_cloud, segmented_cloud_transformed,
                                                           center, planes_by_frame, coefficients_by_frame, m4, q,
                                                           nearest_plane_index);
        if (!success) {
          return false;
        }
      }
    }
    else {
      if (use_pca_) {
        if (segmented_cloud->points.size() >= 3) {
          Eigen::Vector4f pca_centroid;
          pcl::compute3DCentroid(*segmented_cloud, pca_centroid);
          Eigen::Matrix3f covariance;
          pcl::computeCovarianceMatrixNormalized(*segmented_cloud, pca_centroid, covariance);
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
          Eigen::Matrix3f eigen_vectors_pca = eigen_solver.eigenvectors();
          // This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
          // the signs are different and the box doesn't get correctly oriented in some cases.
          eigen_vectors_pca.col(2) = eigen_vectors_pca.col(0).cross(eigen_vectors_pca.col(1));
          // Rotate with respect to y-axis to make x-axis the first principal component
          eigen_vectors_pca = eigen_vectors_pca * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY());
          // Rotate around x
          if (eigen_vectors_pca.col(2).dot(Eigen::Vector3f::UnitZ()) < 0) {
            eigen_vectors_pca = eigen_vectors_pca * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
          }
          // Transform the original cloud to the origin where the principal components correspond to the axes.
          Eigen::Matrix4f projection_transform(Eigen::Matrix4f::Identity());
          projection_transform.block<3,3>(0,0) = eigen_vectors_pca.transpose();
          pcl::transformPointCloud(*segmented_cloud, *segmented_cloud_transformed, projection_transform);
          m4.block<3, 3>(0, 0) = eigen_vectors_pca;
          q = eigen_vectors_pca;
          q.normalize();
          is_center_valid = pcl::compute3DCentroid(*segmented_cloud_transformed, center) != 0;
          center = m4 * center;
        } else {
          NODELET_ERROR("Too small indices for PCA computation");
          segmented_cloud_transformed = segmented_cloud;
        }
      } else {
        segmented_cloud_transformed = segmented_cloud;
        is_center_valid = pcl::compute3DCentroid(*segmented_cloud_transformed, center) != 0;
      }
    }
      
    // create a bounding box
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<pcl::PointXYZ>(*segmented_cloud_transformed, minpt, maxpt);

    double xwidth = maxpt[0] - minpt[0];
    double ywidth = maxpt[1] - minpt[1];
    double zwidth = maxpt[2] - minpt[2];
    if (!pcl_isfinite(xwidth) || !pcl_isfinite(ywidth) || !pcl_isfinite(zwidth))
    {
      // all points in cloud are nan or its size is 0
      xwidth = ywidth = zwidth = 0;
    }
    
    Eigen::Vector4f center2((maxpt[0] + minpt[0]) / 2.0, (maxpt[1] + minpt[1]) / 2.0, (maxpt[2] + minpt[2]) / 2.0, 1.0);
    Eigen::Vector4f center_transformed = m4 * center2;
      
    // set centroid pose msg
    if (is_center_valid) {
      center_pose_msg.position.x = center[0];
      center_pose_msg.position.y = center[1];
      center_pose_msg.position.z = center[2];
      center_pose_msg.orientation.x = q.x();
      center_pose_msg.orientation.y = q.y();
      center_pose_msg.orientation.z = q.z();
      center_pose_msg.orientation.w = q.w();
    }
    else {
      // set invalid pose for invalid center
      center_pose_msg = geometry_msgs::Pose();
    }
    
    // set bounding_box msg
    bounding_box.pose.position.x = center_transformed[0];
    bounding_box.pose.position.y = center_transformed[1];
    bounding_box.pose.position.z = center_transformed[2];
    bounding_box.pose.orientation.x = q.x();
    bounding_box.pose.orientation.y = q.y();
    bounding_box.pose.orientation.z = q.z();
    bounding_box.pose.orientation.w = q.w();
    bounding_box.dimensions.x = xwidth;
    bounding_box.dimensions.y = ywidth;
    bounding_box.dimensions.z = zwidth;
    if (align_boxes_ &&
        align_boxes_with_plane_ &&
        fill_boxes_label_with_nearest_plane_index_) {
      bounding_box.label = nearest_plane_index;
    }
    return true;
  }

  void ClusterPointIndicesDecomposer::addToDebugPointCloud
  (const pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud,
   size_t i,
   pcl::PointCloud<pcl::PointXYZRGB>& debug_output)
  {
    uint32_t rgb = colorRGBAToUInt32(jsk_topic_tools::colorCategory20(i));
    for (size_t j = 0; j < segmented_cloud->points.size(); j++) {
      pcl::PointXYZRGB p;
      p.x= segmented_cloud->points[j].x;
      p.y= segmented_cloud->points[j].y;
      p.z= segmented_cloud->points[j].z;
      p.rgb = *reinterpret_cast<float*>(&rgb);
      debug_output.points.push_back(p);
    }
  }

  void ClusterPointIndicesDecomposer::publishNegativeIndices(
    const sensor_msgs::PointCloud2ConstPtr &input,
    const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices_input)
  {
    if (negative_indices_pub_.getNumSubscribers() <= 0) {
      return;
    }
    std::vector<int> all_indices;

    boost::copy(
      boost::irange(0, (int)(input->width * input->height)),
      std::inserter(all_indices, all_indices.begin()));

    for (size_t i = 0; i < indices_input->cluster_indices.size(); i++) {
      for (size_t j = 0; j < indices_input->cluster_indices[i].indices.size(); ++j) {
        all_indices[indices_input->cluster_indices[i].indices[j]] = -1;
      }
    }
    all_indices.erase(std::remove(all_indices.begin(), all_indices.end(), -1), all_indices.end());

    // publish all_indices
    pcl_msgs::PointIndices ros_indices;
    ros_indices.indices = std::vector<int>(all_indices.begin(), all_indices.end());
    ros_indices.header = input->header;
    negative_indices_pub_.publish(ros_indices);
  }
    
  void ClusterPointIndicesDecomposer::extract(
    const sensor_msgs::PointCloud2ConstPtr &input,
    const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices_input,
    const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
    const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients)
  {
    vital_checker_->poke();
    if (publish_clouds_) {
      allocatePublishers(indices_input->cluster_indices.size());
    }
    publishNegativeIndices(input, indices_input);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    cluster_counter_.add(indices_input->cluster_indices.size());
    
    std::vector<pcl::IndicesPtr> converted_indices;
    
    for (size_t i = 0; i < indices_input->cluster_indices.size(); i++)
    {
      pcl::IndicesPtr vindices;
      // skip indices with points size
      if (min_size_ > 0 &&
          indices_input->cluster_indices[i].indices.size() < min_size_) {
        vindices.reset (new std::vector<int> ());
        converted_indices.push_back(vindices);
        continue;
      }
      if (max_size_ > 0 &&
          indices_input->cluster_indices[i].indices.size() > max_size_) {
        vindices.reset (new std::vector<int> ());
        converted_indices.push_back(vindices);
        continue;
      }
      vindices.reset (new std::vector<int> (indices_input->cluster_indices[i].indices));
      converted_indices.push_back(vindices);
    }

    std::vector<size_t> argsort;
    sortIndicesOrder(cloud, converted_indices, &argsort);
    extract.setInputCloud(cloud);

    // point cloud from camera not laser
    bool is_sensed_with_camera = (input->height != 1);

    cv::Mat mask = cv::Mat::zeros(input->height, input->width, CV_8UC1);
    cv::Mat label = cv::Mat::zeros(input->height, input->width, CV_32SC1);
    pcl::PointCloud<pcl::PointXYZRGB> debug_output;
    jsk_recognition_msgs::BoundingBoxArray bounding_box_array;
    jsk_recognition_msgs::ClusterPointIndices out_cluster_indices;
    bounding_box_array.header = input->header;
    geometry_msgs::PoseArray center_pose_array;
    center_pose_array.header = input->header;
    out_cluster_indices.header = input->header;
    for (size_t i = 0; i < argsort.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      segmented_cloud->is_dense = cloud->is_dense;
      
      pcl_msgs::PointIndices out_indices_msg;
      out_indices_msg.header = input->header;
      out_indices_msg.indices = *(converted_indices[argsort[i]]);
      out_cluster_indices.cluster_indices.push_back(out_indices_msg);

      pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
      extract.setIndices(converted_indices[argsort[i]]);
      extract.filter(*segmented_cloud);
      if (publish_clouds_) {
        sensor_msgs::PointCloud2::Ptr out_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*segmented_cloud, *out_cloud);
        out_cloud->header = input->header;
        publishers_[i].publish(out_cloud);
      }
      // label
      if (is_sensed_with_camera) {
        // create mask & label image from cluster indices
        for (size_t j = 0; j < converted_indices[argsort[i]]->size(); j++) {
          int index = converted_indices[argsort[i]]->data()[j];
          int width_index = index % input->width;
          int height_index = index / input->width;
          mask.at<uchar>(height_index, width_index) = 255;
          // 0 should be skipped,
          // because it is to label image as the black region is to mask image
          label.at<int>(height_index, width_index) = (int)i + 1;
        }
      }
      // adding the pointcloud into debug_output
      addToDebugPointCloud(segmented_cloud, i, debug_output);

      // compute centoid and bounding box
      geometry_msgs::Pose pose_msg;
      jsk_recognition_msgs::BoundingBox bounding_box;
      bounding_box.label = static_cast<int>(argsort[i]);

      if (!segmented_cloud->is_dense) {
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*segmented_cloud, *segmented_cloud, nan_indices);
      }

      bool publish_tf = publish_tf_;
      bool successp = computeCenterAndBoundingBox(
        segmented_cloud, input->header, planes, coefficients, pose_msg, bounding_box, publish_tf);
      if (!successp) {
        return;
      }
      std::string target_frame;
      if (align_boxes_ && !align_boxes_with_plane_) {
        target_frame = target_frame_id_;
      }
      else {
        target_frame = input->header.frame_id;
      }
      center_pose_array.poses.push_back(pose_msg);
      bounding_box.header.frame_id = target_frame;
      bounding_box_array.boxes.push_back(bounding_box);
      if (publish_tf) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z));
        transform.setRotation(tf::createIdentityQuaternion());
        br_->sendTransform(tf::StampedTransform(transform, input->header.stamp, target_frame,
                                                tf_prefix_ + (boost::format("output%02u") % (i)).str()));
      }
    } // for each indices

    // Both bounding box and centroid are computed with transformed point cloud for the target frame.
    if (align_boxes_ && !align_boxes_with_plane_) {
      bounding_box_array.header.frame_id = target_frame_id_;
      center_pose_array.header.frame_id = target_frame_id_;
    }

    if (is_sensed_with_camera) {
      // publish mask
      cv_bridge::CvImage mask_bridge(indices_input->header,
                                    sensor_msgs::image_encodings::MONO8,
                                    mask);
      mask_pub_.publish(mask_bridge.toImageMsg());
      // publish label
      cv_bridge::CvImage label_bridge(indices_input->header,
                                      sensor_msgs::image_encodings::TYPE_32SC1,
                                      label);
      label_pub_.publish(label_bridge.toImageMsg());
    }
    
    sensor_msgs::PointCloud2 debug_ros_output;
    pcl::toROSMsg(debug_output, debug_ros_output);
    debug_ros_output.header = input->header;
    debug_ros_output.is_dense = false;
    pc_pub_.publish(debug_ros_output);
    centers_pub_.publish(center_pose_array);
    box_pub_.publish(bounding_box_array);
    indices_pub_.publish(out_cluster_indices);
  }
  
  void ClusterPointIndicesDecomposer::extract
  (const sensor_msgs::PointCloud2ConstPtr &input,
   const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices_input)
  {
    extract(input, indices_input,
            jsk_recognition_msgs::PolygonArrayConstPtr(),
            jsk_recognition_msgs::ModelCoefficientsArrayConstPtr());
  }

  void ClusterPointIndicesDecomposer::allocatePublishers(size_t num)
  {
    if (num > publishers_.size())
    {
        for (size_t i = publishers_.size(); i < num; i++)
        {
            std::string topic_name = (boost::format("output%02u") % (i)).str();
            NODELET_INFO("advertising %s", topic_name.c_str());
            ros::Publisher publisher = pnh_->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
            publishers_.push_back(publisher);
        }
    }
  }

}  // namespace jsk_pcl_ros

PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ClusterPointIndicesDecomposer, nodelet::Nodelet);
// TODO: deprecate below export. because sorting by z_axis is achieved in parent class with sort_by_ = "z_axis"
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ClusterPointIndicesDecomposerZAxis, nodelet::Nodelet);
