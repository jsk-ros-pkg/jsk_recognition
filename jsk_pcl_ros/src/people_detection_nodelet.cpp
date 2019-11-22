// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/people_detection.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/pcl_util.h"

#include <pcl/people/ground_based_people_detection_app.h>

namespace jsk_pcl_ros {
  void PeopleDetection::onInit() {
    DiagnosticNodelet::onInit();

    pnh_->param("people_height_threshold", people_height_threshold_, 0.5);
    pnh_->param("min_confidence", min_confidence_, -1.5);
    pnh_->param("queue_size", queue_size_, 1);
    pnh_->param("voxel_size", voxel_size_, 0.03);
    pnh_->param("box_width", box_width_, 0.5);
    pnh_->param("box_depth", box_depth_, 0.5);
    pnh_->param<std::string>("filename", trained_filename_, "");

    if (trained_filename_ == "") {
      NODELET_FATAL("Please set svm file name");
    }

    person_classifier_.loadSVMFromFile(trained_filename_);  // load trained SVM
    people_detector_.setVoxelSize(voxel_size_);  // set the voxel size

    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0,
      1.0;  // Kinect RGB camera intrinsics

    people_detector_.setIntrinsics(
      rgb_intrinsics_matrix);  // set RGB camera intrinsic parameters
    people_detector_.setClassifier(
      person_classifier_);  // set person classifier

    ground_coeffs_.resize(4);
    ground_coeffs_[0] = 0.0;
    ground_coeffs_[1] = 0.0;
    ground_coeffs_[2] = 1.0;
    ground_coeffs_[3] = 0.0;

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&PeopleDetection::configCallback, this, _1, _2);
    srv_->setCallback(f);

    ////////////////////////////////////////////////////////
    // Publisher
    ////////////////////////////////////////////////////////
    pub_box_ =
      advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "boxes", 1);

    onInitPostProcess();
  }

  void PeopleDetection::configCallback(Config& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    voxel_size_ = config.voxel_size;
    min_confidence_ = config.min_confidence;
    people_height_threshold_ = config.people_height_threshold;
    box_width_ = config.box_width;
    box_depth_ = config.box_depth;

    people_detector_.setVoxelSize(voxel_size_);  // set the voxel size
  }

  void PeopleDetection::subscribe() {
    sub_info_ =
      pnh_->subscribe("input/info", 1, &PeopleDetection::infoCallback, this);
    sub_cloud_ = pnh_->subscribe("input", 1, &PeopleDetection::detect, this);
    sub_coefficients_ = pnh_->subscribe(
      "input/coefficients", 1, &PeopleDetection::ground_coeffs_callback, this);
  }

  void PeopleDetection::unsubscribe() {
    sub_cloud_.shutdown();
    sub_coefficients_.shutdown();
    sub_info_.shutdown();
  }

  void PeopleDetection::detect(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*cloud_msg, *input_cloud);

    std::vector<pcl::people::PersonCluster<pcl::PointXYZRGBA> >
      clusters;  // vector containing persons clusters
    people_detector_.setInputCloud(input_cloud);
    people_detector_.setGround(ground_coeffs_);  // set floor coefficients
    people_detector_.compute(clusters);          // perform people detection

    jsk_recognition_msgs::BoundingBoxArray bounding_box_array;
    bounding_box_array.header = cloud_msg->header;
    jsk_recognition_msgs::BoundingBox bounding_box;
    bounding_box.header = cloud_msg->header;

    for (std::vector<pcl::people::PersonCluster<pcl::PointXYZRGBA> >::iterator it =
             clusters.begin();
         it != clusters.end(); ++it) {
      if (it->getPersonConfidence() > min_confidence_ &&
          it->getHeight() > people_height_threshold_) {
        bounding_box.pose.position.x = it->getCenter()[0];
        bounding_box.pose.position.y = it->getCenter()[1] + it->getHeight() / 2;
        bounding_box.pose.position.z = it->getCenter()[2];

        bounding_box.pose.orientation.x = 0.0;
        bounding_box.pose.orientation.y = 0.0;
        bounding_box.pose.orientation.z = 0.0;
        bounding_box.pose.orientation.w = 1.0;

        bounding_box.dimensions.x = box_width_;
        bounding_box.dimensions.y = it->getHeight() + 0.3;
        bounding_box.dimensions.z = box_depth_;

        bounding_box_array.boxes.push_back(bounding_box);
      }
    }
    pub_box_.publish(bounding_box_array);
  }

  void PeopleDetection::ground_coeffs_callback(
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr&
      coefficients_msg) {
    if (coefficients_msg->coefficients.size() >= 1) {
      set_ground_coeffs(coefficients_msg->coefficients[0]);
    }
  }

  void PeopleDetection::set_ground_coeffs(
    const pcl_msgs::ModelCoefficients& coefficients) {
    boost::mutex::scoped_lock lock(mutex_);
    for (int i = 0; i < 4; ++i) {
      ground_coeffs_[i] = coefficients.values[i];
    }
  }

  void PeopleDetection::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg) {
    boost::mutex::scoped_lock lock(mutex_);
    latest_camera_info_ = info_msg;

    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << info_msg->K[0], info_msg->K[1], info_msg->K[2],
      info_msg->K[3], info_msg->K[4], info_msg->K[5], info_msg->K[6],
      info_msg->K[7], info_msg->K[8];
    people_detector_.setIntrinsics(
      rgb_intrinsics_matrix);  // set RGB camera intrinsic parameters
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::PeopleDetection, nodelet::Nodelet);
