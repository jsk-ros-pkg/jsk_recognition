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

#include "jsk_pcl_ros/kinfu.h"
#include <geometry_msgs/PoseStamped.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <pcl/common/transforms.h>
#include <cv_bridge/cv_bridge.h>

namespace jsk_pcl_ros
{
  void Kinfu::subscribe()
  {

  }

  void Kinfu::unsubscribe()
  {

  }

  void Kinfu::onInit()
  {
    DiagnosticNodelet::onInit();
    initialized_ = false;
    tf_listener_ = TfListenerSingleton::getInstance();
    pnh_->param("parent_frame_id", parent_frame_id_, std::string("map"));
    pnh_->param("child_frame_id", child_frame_id_, std::string("odom"));
    pnh_->param("kinfu_origin_frame_id", kinfu_origin_frame_id_, std::string("kinfu_origin"));
    pcl::gpu::setDevice(0);
    pcl::gpu::printShortCudaDeviceInfo(0);
    volume_size_ = pcl::device::kinfuLS::VOLUME_SIZE;
    shift_distance_ = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
    snapshot_rate_ = pcl::device::kinfuLS::SNAPSHOT_RATE;
    pub_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output", 1);
    pub_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/cloud", 1);
    sub_depth_image_.subscribe(*pnh_, "input/depth", 1);
    sub_color_image_.subscribe(*pnh_, "input/color", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_depth_image_, sub_color_image_);
    sync_->registerCallback(boost::bind(&Kinfu::callback, this, _1, _2));
    sub_info_ = pnh_->subscribe("input/info", 1, &Kinfu::infoCallback, this);
  }

  void Kinfu::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    info_msg_ = info_msg;
  }

  void Kinfu::callback(const sensor_msgs::Image::ConstPtr& depth_image,
                       const sensor_msgs::Image::ConstPtr& rgb_image)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!info_msg_) {
      JSK_NODELET_WARN("camera info is not yet ready");
      return;
    }
    if (!initialized_) {
      Eigen::Vector3f volume_size = Eigen::Vector3f::Constant (volume_size_);
      
      // Setup kinfu
      kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, 
                                                   shift_distance_,
                                                   info_msg_->height,
                                                   info_msg_->width);
      kinfu_->setInitialCameraPose(Eigen::Affine3f::Identity());
      kinfu_->volume().setTsdfTruncDist (0.030f/*meters*/);
      kinfu_->setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
      kinfu_->setCameraMovementThreshold(0.001f);
      kinfu_->setDepthIntrinsics (info_msg_->K[0], info_msg_->K[4], info_msg_->K[2], info_msg_->K[5]);
      initialized_ = true;
      initial_camera_pose_acquired_ = false;
    }
    if (kinfu_->icpIsLost()) {
      kinfu_->reset();
      JSK_NODELET_FATAL("kinfu is reset");
    }
    
    if (!initial_camera_pose_acquired_) {
      // First we need to check odometry is available or not
      try {
        // odom -> camera
        tf::StampedTransform trans = 
          lookupTransformWithDuration(tf_listener_,
                                      info_msg_->header.frame_id,
                                      child_frame_id_,
                                      depth_image->header.stamp,
                                      ros::Duration(1.0));
        tf::transformTFToEigen(trans, initial_camera_pose_);
      }
      catch (...) {
        JSK_NODELET_ERROR("Failed to lookup transform from %s to %s",
                          child_frame_id_.c_str(),
                          info_msg_->header.frame_id.c_str());
        return;
      }
    }

    cv::Mat depth_m_image = cv_bridge::toCvShare(depth_image, "32FC1")->image;
    cv::Mat depth_mm_image = depth_m_image * 1000.0;
    cv::Mat depth_mm_sc_image;
    depth_mm_image.convertTo(depth_mm_sc_image, CV_16UC1);
    depth_device_.upload(&(depth_mm_sc_image.data[0]), depth_image->width * 2, depth_image->height, depth_image->width);
    //colors_device_.upload(&(rgb_image->data[0]), rgb_image->step, rgb_image->height, rgb_image->width);

    //(*kinfu_)(depth_device_, colors_device_);
    (*kinfu_)(depth_device_);
    Eigen::Affine3f camera_pose = kinfu_->getCameraPose();
    if (!initial_camera_pose_acquired_) {
      initial_kinfu_pose_ = camera_pose;
      initial_camera_pose_acquired_ = true;
    }
    else {
      Eigen::Affine3f K = initial_kinfu_pose_.inverse() * camera_pose;
      // TODO: we should use tf message filter
      try {
        // odom -> camera
        tf::StampedTransform trans = 
          lookupTransformWithDuration(tf_listener_,
                                      depth_image->header.frame_id,
                                      child_frame_id_,
                                      depth_image->header.stamp,
                                      ros::Duration(0.1));
        Eigen::Affine3f odom_camera;
        tf::transformTFToEigen(trans, odom_camera);
        Eigen::Affine3f map = initial_camera_pose_ * K * odom_camera.inverse();
        tf::Transform map_odom_transform;
        tf::transformEigenToTF(map, map_odom_transform);
        tf_broadcaster_.sendTransform(tf::StampedTransform(
                                                           map_odom_transform,
                                                           depth_image->header.stamp,
                                                           parent_frame_id_,
                                                           child_frame_id_));
        tf::Transform kinfu_origin;
        Eigen::Affine3f inverse_camera_pose = camera_pose.inverse();
        tf::transformEigenToTF(inverse_camera_pose, kinfu_origin);
        tf_broadcaster_.sendTransform(tf::StampedTransform(
                                                           kinfu_origin,
                                                           depth_image->header.stamp,
                                                           depth_image->header.frame_id,
                                                           kinfu_origin_frame_id_));
        Eigen::Affine3f camera_diff = K.inverse();
        geometry_msgs::PoseStamped ros_camera_diff;
        tf::poseEigenToMsg(camera_diff, ros_camera_diff.pose);
        ros_camera_diff.header = depth_image->header;
        pub_pose_.publish(ros_camera_diff);

        // scene cloud.
        pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = kinfu_->volume().fetchCloud(cloud_buffer_device_);             
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extracted.download (cloud->points);
        cloud->width = (int)cloud->points.size ();
        cloud->height = 1;
        //pcl::transformPointCloud(*cloud, *transformed_cloud, camera_pose.inverse());
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.stamp = depth_image->header.stamp;
        ros_cloud.header.frame_id = kinfu_origin_frame_id_;
        pub_cloud_.publish(ros_cloud);
      }
      catch (...) {
        JSK_NODELET_ERROR("Failed to lookup transform from %s to %s",
                          child_frame_id_.c_str(),
                          depth_image->header.frame_id.c_str());
        return;
      }
    }
  }
                       
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::Kinfu,
                        nodelet::Nodelet);
