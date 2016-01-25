// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_pcl_ros/boundingbox_occlusion_rejector.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  void BoundingBoxOcclusionRejector::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_ = TfListenerSingleton::getInstance();
    pub_ = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "output", 1);
    pub_target_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/target_image", 1);
    pub_candidate_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/candidate_image", 1);
    onInitPostProcess();
  }

  void BoundingBoxOcclusionRejector::subscribe()
  {
    sub_camera_info_ = pnh_->subscribe("input/camera_info", 1, &BoundingBoxOcclusionRejector::infoCallback, this);
    sub_target_boxes_ = pnh_->subscribe("input/target_boxes", 1, &BoundingBoxOcclusionRejector::targetBoxesCallback, this);
    sub_candidate_boxes_ = pnh_->subscribe("input/candidate_boxes", 1, &BoundingBoxOcclusionRejector::reject, this);
  }

  void BoundingBoxOcclusionRejector::unsubscribe()
  {
    sub_camera_info_.shutdown();
    sub_target_boxes_.shutdown();
    sub_candidate_boxes_.shutdown();
  }

  void BoundingBoxOcclusionRejector::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_info_msg_ = info_msg;
  }

  void BoundingBoxOcclusionRejector::targetBoxesCallback(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& target_boxes_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_target_boxes_msg_ = target_boxes_msg;
  }

  
  void BoundingBoxOcclusionRejector::reject(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& candidate_boxes_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!latest_info_msg_ || !latest_target_boxes_msg_) {
      NODELET_ERROR("No camera info or target_boxes is available");
      return;
    }
    
    // All the header should be same
    if (latest_info_msg_->header.frame_id != latest_target_boxes_msg_->header.frame_id ||
        latest_target_boxes_msg_->header.frame_id != candidate_boxes_msg->header.frame_id) {
      NODELET_ERROR("Different frame_id");
      return;
    }

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(*latest_info_msg_);
    cv::Mat image = cv::Mat::zeros(latest_info_msg_->height, latest_info_msg_->width, CV_8UC1);
    std::vector<std::vector<cv::Point2i> > faces;
    // Draw target boxes
    for (size_t i = 0; i < latest_target_boxes_msg_->boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = latest_target_boxes_msg_->boxes[i];
      // Get vertices
      std::vector<cv::Point3d> vertices = getVertices(box);
      std::vector<cv::Point2i> projected_vertices = projectVertices(vertices, model);
      std::vector<std::vector<cv::Point2i> > projected_faces = separateIntoFaces(projected_vertices);
      for (size_t j = 0; j < projected_faces.size(); j++) {
        faces.push_back(projected_faces[j]);
      }
    }
    for (size_t i = 0; i < faces.size(); i++) {
      std::vector<std::vector<cv::Point2i> > one_face;
      one_face.push_back(faces[i]);
      cv::fillPoly(image, one_face, cv::Scalar(255));
    }

    // Publish image
    if (pub_target_image_.getNumSubscribers() > 0) {
      cv_bridge::CvImage target_image_bridge(
        latest_info_msg_->header,
        sensor_msgs::image_encodings::MONO8,
        image);
      pub_target_image_.publish(target_image_bridge.toImageMsg());
    }

    // Reject
    std::vector<size_t> candidate_indices;
    for (size_t i = 0; i < candidate_boxes_msg->boxes.size(); i++) {
      //cv::Mat copied_image = image.clone();
      cv::Mat candidate_image = cv::Mat::zeros(latest_info_msg_->height, latest_info_msg_->width, CV_8UC1);
      jsk_recognition_msgs::BoundingBox box = candidate_boxes_msg->boxes[i];
      // Get vertices
      std::vector<cv::Point3d> vertices = getVertices(box);
      std::vector<cv::Point2i> projected_vertices = projectVertices(vertices, model);
      std::vector<std::vector<cv::Point2i> > projected_faces = separateIntoFaces(projected_vertices);
      for (size_t j = 0; j < projected_faces.size(); j++) {
        std::vector<std::vector<cv::Point2i> > one_face;
        one_face.push_back(projected_faces[j]);
        cv::fillPoly(candidate_image, one_face, cv::Scalar(255));
        if (pub_candidate_image_.getNumSubscribers() > 0) {
          pub_candidate_image_.publish(cv_bridge::CvImage(latest_info_msg_->header, sensor_msgs::image_encodings::MONO8,
                                                          candidate_image).toImageMsg());
        }
      }
      cv::bitwise_and(image, candidate_image, candidate_image);
      if (cv::countNonZero(candidate_image) == 0) {
        candidate_indices.push_back(i);
      }
    }
    jsk_recognition_msgs::BoundingBoxArray rejected_boxes;
    rejected_boxes.header = candidate_boxes_msg->header;
    for (size_t i = 0; i < candidate_indices.size(); i++) {
      rejected_boxes.boxes.push_back(candidate_boxes_msg->boxes[candidate_indices[i]]);
    }
    pub_.publish(rejected_boxes);
  }

  std::vector<std::vector<cv::Point2i> > BoundingBoxOcclusionRejector::separateIntoFaces(
    const std::vector<cv::Point2i>& vertices)
  {
    // a, b, c, d, e, f, g, h
    std::vector<std::vector<cv::Point2i> > ret;
    std::vector<cv::Point2i> face0, face1, face2, face3, face4, face5;
    cv::Point2i a = vertices[0];
    cv::Point2i b = vertices[1];
    cv::Point2i c = vertices[2];
    cv::Point2i d = vertices[3];
    cv::Point2i e = vertices[4];
    cv::Point2i f = vertices[5];
    cv::Point2i g = vertices[6];
    cv::Point2i h = vertices[7];
    face0.push_back(a); face0.push_back(e); face0.push_back(f); face0.push_back(b);
    face1.push_back(b); face1.push_back(f); face1.push_back(g); face1.push_back(c);
    face2.push_back(c); face2.push_back(g); face2.push_back(h); face2.push_back(d);
    face3.push_back(d); face3.push_back(h); face3.push_back(e); face3.push_back(a);
    face4.push_back(a); face4.push_back(b); face4.push_back(c); face4.push_back(d);
    face5.push_back(e); face5.push_back(h); face5.push_back(g); face5.push_back(f);
    
    ret.push_back(face0);
    ret.push_back(face1);
    ret.push_back(face2);
    ret.push_back(face3);
    ret.push_back(face4);
    ret.push_back(face5);
    return ret;
  }
  
  std::vector<cv::Point2i> BoundingBoxOcclusionRejector::projectVertices(const std::vector<cv::Point3d>& vertices,
                                                                         const image_geometry::PinholeCameraModel& model)
  {
    std::vector<cv::Point2i> ret;
    for (size_t i = 0; i < vertices.size(); i++) {
      ret.push_back(model.project3dToPixel(vertices[i]));
    }
    return ret;
  }
  
  std::vector<cv::Point3d> BoundingBoxOcclusionRejector::getVertices(const jsk_recognition_msgs::BoundingBox& box)
  {
    Eigen::Affine3f pose;
    tf::poseMsgToEigen(box.pose, pose);
    Eigen::Vector3f local_a(box.dimensions.x / 2, box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_b(-box.dimensions.x / 2, box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_c(-box.dimensions.x / 2, -box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_d(box.dimensions.x / 2, -box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_e(box.dimensions.x / 2, box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f local_f(-box.dimensions.x / 2, box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f local_g(-box.dimensions.x / 2, -box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f local_h(box.dimensions.x / 2, -box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f a = pose * local_a;
    Eigen::Vector3f b = pose * local_b;
    Eigen::Vector3f c = pose * local_c;
    Eigen::Vector3f d = pose * local_d;
    Eigen::Vector3f e = pose * local_e;
    Eigen::Vector3f f = pose * local_f;
    Eigen::Vector3f g = pose * local_g;
    Eigen::Vector3f h = pose * local_h;
    cv::Point3d cv_a(a[0], a[1], a[2]);
    cv::Point3d cv_b(b[0], b[1], b[2]);
    cv::Point3d cv_c(c[0], c[1], c[2]);
    cv::Point3d cv_d(d[0], d[1], d[2]);
    cv::Point3d cv_e(e[0], e[1], e[2]);
    cv::Point3d cv_f(f[0], f[1], f[2]);
    cv::Point3d cv_g(g[0], g[1], g[2]);
    cv::Point3d cv_h(h[0], h[1], h[2]);
    std::vector<cv::Point3d> ret;
    ret.push_back(cv_a);
    ret.push_back(cv_b);
    ret.push_back(cv_c);
    ret.push_back(cv_d);
    ret.push_back(cv_e);
    ret.push_back(cv_f);
    ret.push_back(cv_g);
    ret.push_back(cv_h);
    return ret;
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::BoundingBoxOcclusionRejector, nodelet::Nodelet);
