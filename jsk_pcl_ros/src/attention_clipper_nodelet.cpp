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

#include "jsk_pcl_ros/attention_clipper.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"


namespace jsk_pcl_ros
{
  void AttentionClipper::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_ = TfListenerSingleton::getInstance();
    // parameter
    pnh_->param("dimension_x", dimension_x_, 0.1);
    pnh_->param("dimension_y", dimension_y_, 0.1);
    pnh_->param("dimension_z", dimension_z_, 0.1);
    pnh_->param("frame_id", frame_id_, std::string("base_link"));
    pose_.setIdentity();
    pub_camera_info_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output", 1);
    pub_bounding_box_array_
      = advertise<jsk_pcl_ros::BoundingBoxArray>(*pnh_, "output/box_array", 1);
    pub_mask_ = advertise<sensor_msgs::Image>(*pnh_, "output/mask", 1);
    pub_indices_ = advertise<PCLIndicesMsg>(*pnh_, "output/point_indices", 1);
  }

  void AttentionClipper::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &AttentionClipper::clip, this);
    sub_points_ = pnh_->subscribe("input/points", 1,
                                  &AttentionClipper::clipPointcloud, this);
    sub_pose_ = pnh_->subscribe("input/pose",
                                1, &AttentionClipper::poseCallback, this);
    sub_box_ = pnh_->subscribe("input/box",
                               1, &AttentionClipper::boxCallback, this);
  }

  void AttentionClipper::unsubscribe()
  {
    sub_.shutdown();
    sub_pose_.shutdown();
    sub_box_.shutdown();
  }
  
  Vertices AttentionClipper::cubeVertices()
  {
    Vertices nonoffsetted_vertices;
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension_x_/2, -dimension_y_/2, -dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension_x_/2, -dimension_y_/2, dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension_x_/2, dimension_y_/2, -dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(-dimension_x_/2, dimension_y_/2, dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension_x_/2, -dimension_y_/2, -dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension_x_/2, -dimension_y_/2, dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension_x_/2, dimension_y_/2, -dimension_z_/2));
    nonoffsetted_vertices.push_back(
      Eigen::Vector3f(dimension_x_/2, dimension_y_/2, dimension_z_/2));
    return nonoffsetted_vertices;
  }

  void AttentionClipper::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& pose)
  {
    boost::mutex::scoped_lock lock(mutex_);
    Eigen::Affine3d affine;
    tf::poseMsgToEigen(pose->pose, affine);
    frame_id_ = pose->header.frame_id;
    convertEigenAffine3(affine, pose_);
  }

  void AttentionClipper::boxCallback(
    const jsk_pcl_ros::BoundingBox::ConstPtr& box)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      dimension_x_ = box->dimensions.x;
      dimension_y_ = box->dimensions.y;
      dimension_z_ = box->dimensions.z;
      Eigen::Affine3d affine;
      tf::poseMsgToEigen(box->pose, affine);
      frame_id_ = box->header.frame_id;
      convertEigenAffine3(affine, pose_);
    }
    
  }

  void AttentionClipper::computeROI(
    const sensor_msgs::CameraInfo::ConstPtr& msg,
    std::vector<cv::Point2d>& points)
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
    cv::Mat mask = cv::Mat::zeros(msg->height, msg->width, CV_8UC1);
    cv::Size roi_size(roi_region.width, roi_region.height);
    cv::Rect roi_rect(cv::Point(roi_region.x, roi_region.y), roi_size);
    const cv::Scalar white(255, 255, 255);
    cv::rectangle(mask, roi_rect, white, CV_FILLED);
    cv_bridge::CvImage mask_bridge(msg->header,
                                   sensor_msgs::image_encodings::MONO8,
                                   mask);
    pub_mask_.publish(mask_bridge.toImageMsg());
  }

  void AttentionClipper::publishBoundingBox(const std_msgs::Header& header,
                                            Eigen::Affine3f& pose)
  {
    BoundingBoxArray box_array;
    BoundingBox box;
    box.header.stamp = header.stamp;
    box.header.frame_id = frame_id_;
    Eigen::Affine3d posed;
    convertEigenAffine3(pose, posed);
    tf::poseEigenToMsg(posed, box.pose);
    box.dimensions.x = dimension_x_;
    box.dimensions.y = dimension_y_;
    box.dimensions.z = dimension_z_;
    box_array.boxes.push_back(box);
    box_array.header.stamp = header.stamp;
    box_array.header.frame_id = frame_id_;
    pub_bounding_box_array_.publish(box_array);
  }

  void AttentionClipper::clipPointcloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    try {
      // 1. transform pointcloud
      // 2. crop by boundingbox
      // 3. publish indices
      sensor_msgs::PointCloud2 transformed_cloud;
      if (pcl_ros::transformPointCloud(frame_id_, *msg, transformed_cloud,
                                       *tf_listener_)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_cloud, *cloud);
        //pcl::fromROSMsg(*msg, *cloud);
        pcl::CropBox<pcl::PointXYZ> crop_box(false);
        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        Eigen::Vector4f
          max_points(dimension_x_/2, dimension_y_/2, dimension_z_/2, 0);
        Eigen::Vector4f
          min_points(-dimension_x_/2, -dimension_y_/2, -dimension_z_/2, 0);
        
        float roll, pitch, yaw;
        pcl::getEulerAngles(pose_, roll, pitch, yaw);
        crop_box.setInputCloud(cloud);
        crop_box.setMax(max_points);
        crop_box.setMin(min_points);
        crop_box.setTranslation(pose_.translation());
        crop_box.setRotation(Eigen::Vector3f(roll, pitch, yaw));
        crop_box.filter(indices->indices);
        PCLIndicesMsg indices_msg;
        pcl_conversions::fromPCL(*indices, indices_msg);
        indices_msg.header = msg->header;
        pub_indices_.publish(indices_msg);
      }
    }
    catch (tf2::ConnectivityException &e) {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e) {
      NODELET_ERROR("Transform error: %s", e.what());
    }
  }
  
  void AttentionClipper::clip(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    // resolve tf
    try {
      tf_listener_->waitForTransform(frame_id_,
                                     msg->header.frame_id,
                                     msg->header.stamp,
                                     ros::Duration(1.0));
      if (tf_listener_->canTransform(msg->header.frame_id,
                                     frame_id_,
                                     msg->header.stamp)) {
        tf::StampedTransform transform; // header -> frame_id_
        tf_listener_->lookupTransform(frame_id_, msg->header.frame_id,
                                      msg->header.stamp, transform);
        Eigen::Affine3d eigen_transformd;
        tf::transformTFToEigen(transform, eigen_transformd);
        Eigen::Affine3f eigen_transform;
        convertEigenAffine3(eigen_transformd, eigen_transform);
        Eigen::Affine3f offset_from_camera = pose_ * eigen_transform.inverse();
        publishBoundingBox(msg->header, pose_);
        Vertices original_vertices = cubeVertices();
        Vertices vertices;
        for (size_t i = 0; i < original_vertices.size(); i++) {
          vertices.push_back(eigen_transform.inverse() *  (pose_ * original_vertices[i]));
        }
        // compute pinhole camera model
        image_geometry::PinholeCameraModel model;
        bool model_success_p = model.fromCameraInfo(msg);
        if (!model_success_p) {
          ROS_ERROR("failed to create camera model");
          return;
        }
        std::vector<cv::Point2d> local_points;
        for (size_t i = 0; i < vertices.size(); i++) {
          cv::Point3d p(vertices[i][0], vertices[i][1], vertices[i][2]);
          cv::Point2d uv = model.project3dToPixel(p);
          local_points.push_back(uv);
        }
        computeROI(msg, local_points);
      }
    }
    catch (tf2::ConnectivityException &e) {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e) {
      NODELET_ERROR("Transform error: %s", e.what());
    }
  }
  
  void AttentionClipper::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "AttentionClipper running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "AttentionClipper", vital_checker_, stat);
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::AttentionClipper, nodelet::Nodelet);
