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
#include "jsk_pcl_ros/attention_clipper.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/transforms.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include "jsk_recognition_utils/pcl_ros_util.h"
#include "jsk_recognition_utils/pcl_util.h"
#include <algorithm>
#include <set>
namespace jsk_pcl_ros
{
  void AttentionClipper::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_ = TfListenerSingleton::getInstance();
    pnh_->param("negative", negative_, false);
    pnh_->param("use_multiple_attention", use_multiple_attention_, false);
    if (!use_multiple_attention_) {
      Eigen::Affine3f pose = Eigen::Affine3f::Identity();
      std::vector<double> initial_pos;
      if (jsk_topic_tools::readVectorParameter(*pnh_,
                                               "initial_pos",
                                               initial_pos))
      {
        pose.translation() = Eigen::Vector3f(initial_pos[0],
                                              initial_pos[1],
                                              initial_pos[2]);
      }

      std::vector<double> initial_rot;
      if (jsk_topic_tools::readVectorParameter(*pnh_,
                                               "initial_rot",
                                               initial_rot))
      {
        pose = pose
          * Eigen::AngleAxisf(initial_rot[0],
                              Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(initial_rot[1],
                              Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(initial_rot[2],
                              Eigen::Vector3f::UnitZ());
      }

      // parameter
      // backward compatibility
      double dimension_x, dimension_y, dimension_z;
      pnh_->param("dimension_x", dimension_x, 0.1);
      pnh_->param("dimension_y", dimension_y, 0.1);
      pnh_->param("dimension_z", dimension_z, 0.1);
      dimensions_.push_back(Eigen::Vector3f(dimension_x,
                                            dimension_y,
                                            dimension_z));
      std::string frame_id;
      pnh_->param("frame_id", frame_id, std::string("base_link"));
      frame_id_list_.push_back(frame_id);
      pose_list_.push_back(pose);
    }
    else {                      // multiple interst
      // ~initial_pos_list
      //   -> [[0, 0, 0], ...]
      std::vector<std::vector<double> > initial_pos_list;
      std::vector<std::vector<double> > initial_rot_list;
      std::vector<std::vector<double> > dimensions;
      jsk_topic_tools::readVectorParameter(*pnh_, "initial_pos_list",
                                           initial_pos_list);
      jsk_topic_tools::readVectorParameter(*pnh_, "initial_rot_list",
                                           initial_rot_list);
      jsk_topic_tools::readVectorParameter(*pnh_, "dimensions", dimensions);
      jsk_topic_tools::readVectorParameter(*pnh_, "prefixes", prefixes_);
      if (initial_pos_list.size() != 0) {
        initializePoseList(initial_pos_list.size());
        for (size_t i = 0; i < initial_pos_list.size(); i++) {
          if (initial_pos_list[i].size() != 3) {
            NODELET_FATAL("element of ~initial_pos_list should be [x, y, z]");
            return;
          }
          pose_list_[i].translation() = Eigen::Vector3f(initial_pos_list[i][0],
                                                        initial_pos_list[i][1],
                                                        initial_pos_list[i][2]);
        }
      }
      // ~initial_rot_list
      //   -> [[0, 0, 0], ...]
      if (initial_rot_list.size() != 0) {
        // error check
        if (initial_pos_list.size() != 0 &&
            initial_rot_list.size() != initial_pos_list.size()) {
          NODELET_FATAL(
            "the size of ~initial_pos_list and ~initial_rot_list are different");
          return;
        }
        if (initial_pos_list.size() == 0) {
          initializePoseList(initial_rot_list.size());
        }
        for (size_t i = 0; i < initial_rot_list.size(); i++) {
          if (initial_rot_list[i].size() != 3) {
            NODELET_FATAL("element of ~initial_rot_list should be [rx, ry, rz]");
            return;
          }
          pose_list_[i] = pose_list_[i]
            * Eigen::AngleAxisf(initial_rot_list[i][0],
                                Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(initial_rot_list[i][1],
                                Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(initial_rot_list[i][2],
                                Eigen::Vector3f::UnitZ());
        }
      }
      // ~dimensions
      //   -> [[x, y, z], [x, y, z] ...]
      if (dimensions.size() != 0) {
        // error check
        if (pose_list_.size() != 0 &&
            pose_list_.size() != dimensions.size()) {
          NODELET_FATAL(
            "the size of ~dimensions and ~initial_pos_list or ~initial_rot_list are different");
          return;
        }
        if (pose_list_.size() == 0) {
          initializePoseList(dimensions.size());
        }
        for (size_t i = 0; i < dimensions.size(); i++) {
          dimensions_.push_back(Eigen::Vector3f(dimensions[i][0],
                                                dimensions[i][1],
                                                dimensions[i][2]));
        }
      }

      // ~prefixes
      //   -> [left_hand, right_hand ...]
      if (prefixes_.size() != 0) {
        // error check
        if (prefixes_.size() != dimensions.size()) {
          NODELET_FATAL(
            "the size of ~prefixes and ~dimensions are different");
          return;
        }
        for(int i = 0; i < prefixes_.size(); i++){
          multiple_pub_indices_.push_back(advertise<PCLIndicesMsg>(*pnh_, prefixes_[i]+std::string("/point_indices"), 1));
        }
      }

      jsk_topic_tools::readVectorParameter(*pnh_, "frame_id_list", frame_id_list_);
    }
    pub_camera_info_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output", 1);
    pub_bounding_box_array_
      = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "output/box_array", 1);
    pub_mask_ = advertise<sensor_msgs::Image>(*pnh_, "output/mask", 1);
    pub_indices_ = advertise<PCLIndicesMsg>(*pnh_, "output/point_indices", 1);
    pub_cluster_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output/cluster_point_indices", 1);

    onInitPostProcess();
  }

  void AttentionClipper::initializePoseList(size_t num)
  {
    pose_list_.resize(num);
    for (size_t i = 0; i < pose_list_.size(); i++) {
      pose_list_[i].setIdentity();
    }
  }

  void AttentionClipper::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &AttentionClipper::clip, this);
    sub_points_ = pnh_->subscribe("input/points", 1,
                                  &AttentionClipper::clipPointcloud, this);
    if (!use_multiple_attention_) {
      sub_pose_ = pnh_->subscribe("input/pose",
                                  1, &AttentionClipper::poseCallback, this);
      sub_box_ = pnh_->subscribe("input/box",
                                 1, &AttentionClipper::boxCallback, this);
    }
    else {
      sub_pose_ = pnh_->subscribe("input/pose_array",
                                  1, &AttentionClipper::poseArrayCallback, this);
      sub_box_ = pnh_->subscribe("input/box_array",
                                 1, &AttentionClipper::boxArrayCallback, this);
    }
  }

  void AttentionClipper::unsubscribe()
  {
    sub_.shutdown();
    sub_points_.shutdown();
    sub_pose_.shutdown();
    sub_box_.shutdown();
  }

  jsk_recognition_utils::Vertices AttentionClipper::cubeVertices(Eigen::Vector3f& dimension)
  {
    jsk_recognition_utils::Vertices nonoffsetted_vertices;
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension[0]/2, -dimension[1]/2, -dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension[0]/2, -dimension[1]/2, dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension[0]/2, dimension[1]/2, -dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension[0]/2, dimension[1]/2, dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension[0]/2, -dimension[1]/2, -dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension[0]/2, -dimension[1]/2, dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension[0]/2, dimension[1]/2, -dimension[2]/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension[0]/2, dimension[1]/2, dimension[2]/2));
    return nonoffsetted_vertices;
  }

  // callback only for one interaction
  void AttentionClipper::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& pose)
  {
    boost::mutex::scoped_lock lock(mutex_);
    frame_id_list_[0] = pose->header.frame_id;
    tf::poseMsgToEigen(pose->pose, pose_list_[0]);
  }

  void AttentionClipper::boxCallback(
    const jsk_recognition_msgs::BoundingBox::ConstPtr& box)
  {
    boost::mutex::scoped_lock lock(mutex_);
    dimensions_[0][0] = box->dimensions.x;
    dimensions_[0][1] = box->dimensions.y;
    dimensions_[0][2] = box->dimensions.z;
    frame_id_list_[0] = box->header.frame_id;
    tf::poseMsgToEigen(box->pose, pose_list_[0]);
  }

  // callback for multiple interactions
  void AttentionClipper::poseArrayCallback(
    const geometry_msgs::PoseArray::ConstPtr& pose_array)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // resize
    initializePoseList(pose_array->poses.size());
    frame_id_list_.resize(pose_array->poses.size());
    std::fill(frame_id_list_.begin(), frame_id_list_.end(),
              pose_array->header.frame_id);
    for (size_t i = 0; i < pose_list_.size(); i++) {
      tf::poseMsgToEigen(pose_array->poses[i], pose_list_[i]);
    }
  }

  void AttentionClipper::boxArrayCallback(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array)
  {
    boost::mutex::scoped_lock lock(mutex_);
    initializePoseList(box_array->boxes.size());
    frame_id_list_.resize(box_array->boxes.size());
    dimensions_.resize(box_array->boxes.size());
    for (size_t i = 0; i < pose_list_.size(); i++) {
      tf::poseMsgToEigen(box_array->boxes[i].pose, pose_list_[i]);
      frame_id_list_[i] = box_array->boxes[i].header.frame_id;
      dimensions_[i] = Eigen::Vector3f(box_array->boxes[i].dimensions.x,
                                       box_array->boxes[i].dimensions.y,
                                       box_array->boxes[i].dimensions.z);
    }
  }

  void AttentionClipper::publishBoundingBox(
    const std_msgs::Header& header)
  {
    jsk_recognition_msgs::BoundingBoxArray box_array;
    box_array.header.frame_id = header.frame_id;
    box_array.header.stamp = header.stamp;
    for (size_t i = 0; i < pose_list_.size(); i++) {
      jsk_recognition_msgs::BoundingBox box;
      box.header = header;
      tf::poseEigenToMsg(transformed_pose_list_[i], box.pose);
      jsk_recognition_utils::pointFromVectorToXYZ(dimensions_[i], box.dimensions);
      box_array.boxes.push_back(box);
    }
    pub_bounding_box_array_.publish(box_array);
  }

  void AttentionClipper::computeROI(
    const sensor_msgs::CameraInfo::ConstPtr& msg,
    std::vector<cv::Point2d>& points,
    cv::Mat& mask)
  {
    double min_u, min_v, max_u, max_v;
    min_u = msg->width;
    min_v = msg->height;
    max_u = max_v = 0;
    for (size_t i = 0; i < points.size(); i++) {
      cv::Point2d uv(points[i]);
      // check limit
      if (uv.x < 0) {
        uv.x = 0;
      }
      if (uv.y < 0) {
        uv.y = 0;
      }
      if (uv.x > msg->width) {
        uv.x = msg->width;
      }

      if (uv.y > msg->height) {
        uv.y = msg->height;
      }
      if (min_u > uv.x) {
        min_u = uv.x;
      }
      if (max_u < uv.x) {
        max_u = uv.x;
      }
      if (min_v > uv.y) {
        min_v = uv.y;
      }

      if (max_v < uv.y) {
        max_v = uv.y;
      }
    }
    // now we have min/max of u/v
    cv::Rect raw_roi(min_u, min_v, (max_u - min_u), (max_v - min_v));
    cv::Rect original(0, 0, msg->width, msg->height);
    cv::Rect roi_region = raw_roi & original;
    sensor_msgs::CameraInfo roi(*msg);
    roi.roi.x_offset = roi_region.x;
    roi.roi.y_offset = roi_region.y;
    roi.roi.width = roi_region.width;
    roi.roi.height = roi_region.height;
    roi.roi.do_rectify = true;
    pub_camera_info_.publish(roi);
    // mask computation
    mask = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    cv::Size roi_size(roi_region.width, roi_region.height);
    cv::Rect roi_rect(cv::Point(roi_region.x, roi_region.y), roi_size);
    const cv::Scalar white(255);
    cv::rectangle(mask, roi_rect, white, CV_FILLED);
  }

  void AttentionClipper::clipPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("clipPointcloud");
    vital_checker_->poke();
    try {
      // 1. transform pointcloud
      // 2. crop by boundingbox
      // 3. publish indices
      pcl::PointIndices::Ptr all_indices (new pcl::PointIndices);
      jsk_recognition_msgs::ClusterPointIndices cluster_indices_msg;
      std::map<std::string, tf::StampedTransform> transforms;
      transformed_pose_list_.clear();
      for (size_t i = 0; i < pose_list_.size(); i++) {
        std::string frame_id = frame_id_list_[i];
        // check transform cache
        if (transforms.find(frame_id) == transforms.end()) {
          tf::StampedTransform new_transform = lookupTransformWithDuration(
            /*listener=*/tf_listener_,
            /*to_frame=*/frame_id,                // box origin
            /*from_frame=*/msg->header.frame_id,  // sensor origin
            /*time=*/msg->header.stamp,
            /*duration=*/ros::Duration(1.0));
          transforms[frame_id] = new_transform; // sensor to box
        }
        tf::StampedTransform tf_transform = transforms[frame_id];
        Eigen::Affine3f transform;
        tf::transformTFToEigen(tf_transform, transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        Eigen::Affine3f transformed_box_pose = transform * pose_list_[i];
        transformed_pose_list_.push_back(transformed_box_pose);

        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        jsk_recognition_msgs::BoundingBox bbox_msg;
        bbox_msg.header.frame_id = cloud->header.frame_id;
        tf::poseEigenToMsg(transformed_box_pose, bbox_msg.pose);
        bbox_msg.dimensions.x = dimensions_[i][0];
        bbox_msg.dimensions.y = dimensions_[i][1];
        bbox_msg.dimensions.z = dimensions_[i][2];
        jsk_recognition_utils::cropPointCloud<pcl::PointXYZ>(cloud, bbox_msg, &(indices->indices));

        // indices->indices may include NaN and inf points
        // https://github.com/jsk-ros-pkg/jsk_recognition/issues/888
        pcl::PointIndices non_nan_indices;
        for (size_t j = 0; j < indices->indices.size(); j++) {
          pcl::PointXYZ p = cloud->points[indices->indices[j]];
          if (pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z)) {
            non_nan_indices.indices.push_back(indices->indices[j]);
          }
        }
        PCLIndicesMsg indices_msg;
        pcl_conversions::fromPCL(non_nan_indices, indices_msg);
        cluster_indices_msg.cluster_indices.push_back(indices_msg);
        if(prefixes_.size()){
          indices_msg.header = msg->header;
          multiple_pub_indices_[i].publish(indices_msg);
        }

        all_indices = jsk_recognition_utils::addIndices(*all_indices, non_nan_indices);
      }
      if (negative_) {
        // Publish indices which is NOT inside of box.
        pcl::PointIndices::Ptr tmp_indices (new pcl::PointIndices);
        std::set<int> positive_indices(all_indices->indices.begin(), all_indices->indices.end());
        for (size_t i = 0; i < msg->width * msg->height; i++) {
          if (positive_indices.find(i) == positive_indices.end()) {
            tmp_indices->indices.push_back(i);
          }
        }
        all_indices = tmp_indices;
      }
      PCLIndicesMsg indices_msg;
      pcl_conversions::fromPCL(*all_indices, indices_msg);
      cluster_indices_msg.header = indices_msg.header = msg->header;
      pub_indices_.publish(indices_msg);
      pub_cluster_indices_.publish(cluster_indices_msg);
      publishBoundingBox(msg->header);
    }
    catch (std::runtime_error &e) {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    } 
  }

  void AttentionClipper::clip(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    // resolve tf for all interest
    try {
      image_geometry::PinholeCameraModel model;
      cv::Mat all_mask_image = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
      bool model_success_p = model.fromCameraInfo(msg);
      if (!model_success_p) {
        ROS_ERROR("failed to create camera model");
        return;
      }
      for (size_t i = 0; i < pose_list_.size(); i++) {
        std::string frame_id = frame_id_list_[i];
        tf_listener_->waitForTransform(frame_id,
                                       msg->header.frame_id,
                                       msg->header.stamp,
                                       ros::Duration(1.0));
        Eigen::Affine3f offset = pose_list_[i];
        if (tf_listener_->canTransform(msg->header.frame_id,
                                       frame_id,
                                       msg->header.stamp)) {
          tf::StampedTransform transform; // header -> frame_id_
          tf_listener_->lookupTransform(frame_id, msg->header.frame_id,
                                        msg->header.stamp, transform);
          Eigen::Affine3f eigen_transform;
          tf::transformTFToEigen(transform, eigen_transform);
          jsk_recognition_utils::Vertices original_vertices = cubeVertices(dimensions_[i]);
          jsk_recognition_utils::Vertices vertices;
          for (size_t i = 0; i < original_vertices.size(); i++) {
            vertices.push_back(eigen_transform.inverse()
                               * (offset * original_vertices[i]));
          }
          std::vector<cv::Point2d> local_points;
          for (size_t i = 0; i < vertices.size(); i++) {
            cv::Point3d p(vertices[i][0], vertices[i][1], vertices[i][2]);
            cv::Point2d uv = model.project3dToPixel(p);
            local_points.push_back(uv);
          }
          cv::Mat mask_image;
          computeROI(msg, local_points, mask_image);
          all_mask_image = all_mask_image | mask_image;
        }
      }
      // publish
      cv_bridge::CvImage mask_bridge(msg->header,
                                     sensor_msgs::image_encodings::MONO8,
                                     all_mask_image);
      pub_mask_.publish(mask_bridge.toImageMsg());
      //publishBoundingBox(msg->header);
    }
    catch (std::runtime_error &e) {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    } 
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::AttentionClipper, nodelet::Nodelet);
