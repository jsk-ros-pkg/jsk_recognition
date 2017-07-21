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

#include "jsk_perception/bounding_box_to_rect.h"
#include <jsk_recognition_utils/geo/cube.h>
#include <jsk_recognition_utils/sensor_model_utils.h>

namespace jsk_perception
{
  void BoundingBoxToRect::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("tf_queue_size", tf_queue_size_, 10);
    pub_ = advertise<jsk_recognition_msgs::RectArray>(*pnh_, "output", 1);
    pub_internal_ = pnh_->advertise<jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo>("internal", 1);
    sub_box_with_info_.subscribe(*pnh_, "internal", 1);
    //onInitPosrPocess();
  }

  void BoundingBoxToRect::subscribe()
  {
    // camera_info + bounding box array
    sub_info_.subscribe(*pnh_, "input/info", 1);
    sub_boxes_.subscribe(*pnh_, "input", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_); 
      async_->connectInput(sub_info_, sub_boxes_);
      async_->registerCallback(boost::bind(&BoundingBoxToRect::inputCallback, this, _1, _2));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_info_, sub_boxes_);
      sync_->registerCallback(boost::bind(&BoundingBoxToRect::inputCallback, this, _1, _2));
    }
    // camera_info + bounding box
    sub_box_.subscribe(*pnh_, "input/box", 1);
    if (approximate_sync_) {
      async_box_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicyBox> >(queue_size_);
      async_box_->connectInput(sub_info_, sub_box_);
      async_box_->registerCallback(boost::bind(&BoundingBoxToRect::inputBoxCallback, this, _1, _2));
    } else {
      sync_box_ = boost::make_shared<message_filters::Synchronizer<SyncPolicyBox> >(queue_size_);
      sync_box_->connectInput(sub_info_, sub_box_);
      sync_box_->registerCallback(boost::bind(&BoundingBoxToRect::inputBoxCallback, this, _1, _2));
    }
  }

  void BoundingBoxToRect::unsubscribe()
  {
    sub_info_.unsubscribe();
    sub_boxes_.unsubscribe();
    sub_box_.unsubscribe();
    frame_id_ = "";
  }

  void BoundingBoxToRect::inputBoxCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                                           const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg)
  {
    jsk_recognition_msgs::BoundingBoxArray::Ptr boxes_msg(
      new jsk_recognition_msgs::BoundingBoxArray());
    boxes_msg->header = box_msg->header;
    boxes_msg->boxes.push_back(*box_msg);
    inputCallback(info_msg, boxes_msg);
  }

  void BoundingBoxToRect::inputCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                                        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& boxes_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (frame_id_.empty()) {
      // setup tf message filters
      frame_id_ = boxes_msg->header.frame_id;
      tf_filter_.reset(new tf::MessageFilter<jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo>(
                         sub_box_with_info_,
                         *tf_listener_,
                         frame_id_,
                         tf_queue_size_));
      tf_filter_->registerCallback(boost::bind(&BoundingBoxToRect::internalCallback, this, _1));
    }
    jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo internal_msg;
    internal_msg.header = boxes_msg->header;
    internal_msg.boxes = *boxes_msg;
    internal_msg.camera_info = *info_msg;
    pub_internal_.publish(internal_msg);
  }

  void BoundingBoxToRect::internalCallback(
    const jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    tf::StampedTransform box_to_info_tf = jsk_recognition_utils::lookupTransformWithDuration(
      tf_listener_,
      msg->boxes.header.frame_id,
      msg->camera_info.header.frame_id,
      msg->boxes.header.stamp,
      ros::Duration(0.0));
    Eigen::Affine3f box_to_info;
    tf::transformTFToEigen(box_to_info_tf, box_to_info);
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(msg->camera_info);
    jsk_recognition_msgs::RectArray rect_array;
    rect_array.header = msg->camera_info.header;
    for (size_t i = 0; i < msg->boxes.boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = msg->boxes.boxes[i];
      jsk_recognition_utils::Cube cube(box);
      jsk_recognition_utils::Vertices vertices = cube.transformVertices(box_to_info);
      std::vector<cv::Point> points = jsk_recognition_utils::project3DPointstoPixel(model, vertices);
      cv::Rect rect = cv::boundingRect(points);
      jsk_recognition_msgs::Rect ros_rect;
      ros_rect.x = rect.x;
      ros_rect.y = rect.y;
      ros_rect.width = rect.width;
      ros_rect.height = rect.height;
      rect_array.rects.push_back(ros_rect);
    }
    pub_.publish(rect_array);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::BoundingBoxToRect, nodelet::Nodelet);
