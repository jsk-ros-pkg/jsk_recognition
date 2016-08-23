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
        JSK_ROS_WARN("~tf_prefix is not specified, using %s", getName().c_str());
      }
      tf_prefix_ = getName();
    }

    // fixed parameters
    pnh_->param("approximate_sync", use_async_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("publish_clouds", publish_clouds_, false);
    if (publish_clouds_) {
      JSK_ROS_WARN("~output%%02d are not published before subscribed, you should subscribe ~debug_output in debuging.");
    }
    pnh_->param("align_boxes", align_boxes_, false);
    if (align_boxes_) {
      pnh_->param("align_boxes_with_plane", align_boxes_with_plane_, true);
    }
    if (align_boxes_ && !align_boxes_with_plane_) {
      tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
      if (!pnh_->getParam("target_frame_id", target_frame_id_)) {
        JSK_ROS_FATAL("~target_frame_id is not specified");
        return;
      }
      JSK_ROS_INFO("Aligning bboxes with '%s' using tf transform.", target_frame_id_.c_str());
    }
    pnh_->param("use_pca", use_pca_, false);
    pnh_->param("force_to_flip_z_axis", force_to_flip_z_axis_, true);
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

    onInitPostProcess();
  }

  void ClusterPointIndicesDecomposer::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    max_size_ = config.max_size;
    min_size_ = config.min_size;
  }

  void ClusterPointIndicesDecomposer::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_target_.subscribe(*pnh_, "target", 1);
    if (align_boxes_ && align_boxes_with_plane_) {
      sync_align_ = boost::make_shared<message_filters::Synchronizer<SyncAlignPolicy> >(queue_size_);
      sub_polygons_.subscribe(*pnh_, "align_planes", 1);
      sub_coefficients_.subscribe(*pnh_, "align_planes_coefficients", 1);
      sync_align_->connectInput(sub_input_, sub_target_, sub_polygons_, sub_coefficients_);
      sync_align_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2, _3, _4));
    }
    else if (use_async_) {
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

  void ClusterPointIndicesDecomposer::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_target_.unsubscribe();
    if (align_boxes_ && align_boxes_with_plane_) {
      sub_polygons_.unsubscribe();
      sub_coefficients_.unsubscribe();
    }
  }
  
  void ClusterPointIndicesDecomposer::sortIndicesOrder
  (pcl::PointCloud<pcl::PointXYZ>::Ptr input,
   std::vector<pcl::IndicesPtr> indices_array,
   std::vector<pcl::IndicesPtr> &output_array)
  {
    output_array.resize(indices_array.size());
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      output_array[i] = indices_array[i];
    }
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
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "ClusterPointIndicesDecomposer", vital_checker_, stat);
    }
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
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud_transformed,
    const Eigen::Vector4f center,
    const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
    const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
    Eigen::Matrix4f& m4,
    Eigen::Quaternionf& q)
  {
    int nearest_plane_index = findNearestPlane(center, planes, coefficients);
    if (nearest_plane_index == -1) {
      segmented_cloud_transformed = segmented_cloud;
      JSK_NODELET_ERROR("no planes to align boxes are given");
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
      if (isnan(theta) ||
          isnan(rotation_axis[0]) ||
          isnan(rotation_axis[1]) ||
          isnan(rotation_axis[2])) {
        segmented_cloud_transformed = segmented_cloud;
        JSK_NODELET_ERROR("cannot compute angle to align the point cloud: [%f, %f, %f], [%f, %f, %f]",
                      z_axis[0], z_axis[1], z_axis[2],
                      normal[0], normal[1], normal[2]);
      }
      else {
        Eigen::Matrix3f m = Eigen::Matrix3f::Identity() * rot;
        if (use_pca_) {
          // first project points to the plane
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud
            (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::ProjectInliers<pcl::PointXYZRGB> proj;
          proj.setModelType (pcl::SACMODEL_PLANE);
          pcl::ModelCoefficients::Ptr
            plane_coefficients (new pcl::ModelCoefficients);
          plane_coefficients->values
            = coefficients->coefficients[nearest_plane_index].values;
          proj.setModelCoefficients(plane_coefficients);
          proj.setInputCloud(segmented_cloud);
          proj.filter(*projected_cloud);
          if (projected_cloud->points.size() >= 3) {
            pcl::PCA<pcl::PointXYZRGB> pca;
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
            JSK_NODELET_ERROR("Too small indices for PCA computation");
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
        Eigen::Matrix4f inv_m = m4.inverse();
        pcl::transformPointCloud(*segmented_cloud, *segmented_cloud_transformed, inv_m);
      }
    }
    return true;
  }

  bool ClusterPointIndicesDecomposer::computeCenterAndBoundingBox
  (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
   const std_msgs::Header header,
   const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
   const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
   geometry_msgs::Pose& center_pose_msg,
   jsk_recognition_msgs::BoundingBox& bounding_box)
  {
    bounding_box.header = header;

    Eigen::Vector4f center;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      segmented_cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    // align boxes if possible
    Eigen::Matrix4f m4 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    if (align_boxes_) {
      if (align_boxes_with_plane_) {
        pcl::compute3DCentroid(*segmented_cloud, center);
        bool success = transformPointCloudToAlignWithPlane(segmented_cloud, segmented_cloud_transformed,
                                                           center, planes, coefficients, m4, q);
        if (!success) {
          return false;
        }
      }
      else {
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
          JSK_NODELET_ERROR("Transform error: %s", e.what());
          return false;
        }
        Eigen::Affine3f transform;
        tf::transformTFToEigen(tf_transform, transform);
        pcl::transformPointCloud(*segmented_cloud, *segmented_cloud_transformed, transform);
        pcl::compute3DCentroid(*segmented_cloud_transformed, center);
        bounding_box.header.frame_id = target_frame_id_;
      }
    }
    else {
      segmented_cloud_transformed = segmented_cloud;
      pcl::compute3DCentroid(*segmented_cloud_transformed, center);
    }
      
    // create a bounding box
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*segmented_cloud_transformed, minpt, maxpt);

    double xwidth = maxpt[0] - minpt[0];
    double ywidth = maxpt[1] - minpt[1];
    double zwidth = maxpt[2] - minpt[2];
    
    Eigen::Vector4f center2((maxpt[0] + minpt[0]) / 2.0, (maxpt[1] + minpt[1]) / 2.0, (maxpt[2] + minpt[2]) / 2.0, 1.0);
    Eigen::Vector4f center_transformed = m4 * center2;
      
    // set centroid pose msg
    center_pose_msg.position.x = center[0];
    center_pose_msg.position.y = center[1];
    center_pose_msg.position.z = center[2];
    center_pose_msg.orientation.x = 0;
    center_pose_msg.orientation.y = 0;
    center_pose_msg.orientation.z = 0;
    center_pose_msg.orientation.w = 1;
    
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
    return true;
  }

  void ClusterPointIndicesDecomposer::addToDebugPointCloud
  (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
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
    std::set<int> all_indices;
    boost::copy(
      boost::irange(0, (int)(input->width * input->height)),
      std::inserter(all_indices, all_indices.begin()));
    for (size_t i = 0; i < indices_input->cluster_indices.size(); i++) {
      std::set<int> indices_set(indices_input->cluster_indices[i].indices.begin(),
                                indices_input->cluster_indices[i].indices.end());
      std::set<int> diff_indices;
      std::set_difference(all_indices.begin(), all_indices.end(),
                          indices_set.begin(), indices_set.end(),
                          std::inserter(diff_indices, diff_indices.begin()));
      all_indices = diff_indices;
    }
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
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::fromROSMsg(*input, *cloud_xyz);
    cluster_counter_.add(indices_input->cluster_indices.size());
    
    std::vector<pcl::IndicesPtr> converted_indices;
    std::vector<pcl::IndicesPtr> sorted_indices;
    
    for (size_t i = 0; i < indices_input->cluster_indices.size(); i++)
    {
      pcl::IndicesPtr vindices;
      // skip indices with points size
      if (min_size_ > 0 &&
          indices_input->cluster_indices[i].indices.size() < min_size_) {
        continue;
      }
      if (max_size_ > 0 &&
          indices_input->cluster_indices[i].indices.size() > max_size_) {
        continue;
      }
      vindices.reset (new std::vector<int> (indices_input->cluster_indices[i].indices));
      converted_indices.push_back(vindices);
    }
    
    sortIndicesOrder(cloud_xyz, converted_indices, sorted_indices);
    extract.setInputCloud(cloud);

    // point cloud from camera not laser
    bool is_sensed_with_camera = (input->height != 1);

    cv::Mat mask = cv::Mat::zeros(input->height, input->width, CV_8UC1);
    cv::Mat label = cv::Mat::zeros(input->height, input->width, CV_32SC1);
    pcl::PointCloud<pcl::PointXYZRGB> debug_output;
    jsk_recognition_msgs::BoundingBoxArray bounding_box_array;
    bounding_box_array.header = input->header;
    geometry_msgs::PoseArray center_pose_array;
    center_pose_array.header = input->header;
    for (size_t i = 0; i < sorted_indices.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
      extract.setIndices(sorted_indices[i]);
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
        for (size_t j = 0; j < sorted_indices[i]->size(); j++) {
          int index = sorted_indices[i]->data()[j];
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
      bool successp = computeCenterAndBoundingBox(
        segmented_cloud, input->header, planes, coefficients, pose_msg, bounding_box);
      if (!successp) {
        return;
      }
      center_pose_array.poses.push_back(pose_msg);
      bounding_box_array.boxes.push_back(bounding_box);
      if (publish_tf_) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z));
        transform.setRotation(tf::createIdentityQuaternion());
        std::string target_frame;
        if (align_boxes_ && !align_boxes_with_plane_) {
          target_frame = target_frame_id_;
        }
        else {
          target_frame = input->header.frame_id;
        }
        br_->sendTransform(tf::StampedTransform(transform, input->header.stamp, target_frame,
                                                tf_prefix_ + (boost::format("output%02u") % (i)).str()));
      }
    }

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
            JSK_ROS_INFO("advertising %s", topic_name.c_str());
            ros::Publisher publisher = pnh_->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
            publishers_.push_back(publisher);
        }
    }
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ClusterPointIndicesDecomposer,
                        nodelet::Nodelet);

