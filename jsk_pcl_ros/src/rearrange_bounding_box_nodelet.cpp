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
#include "jsk_pcl_ros/rearrange_bounding_box.h"

namespace jsk_pcl_ros
{
  void RearrangeBoundingBox::onInit () {
    ConnectionBasedNodelet::onInit();

    pnh_->param("offset_x", offset_x_, 0.0);
    pnh_->param("offset_y", offset_y_, 0.0);
    pnh_->param("offset_z", offset_z_, 0.0);
    pnh_->param("scale_x", scale_x_, 1.0);
    pnh_->param("scale_y", scale_y_, 1.0);
    pnh_->param("scale_z", scale_z_, 1.0);
    pnh_->param("rotate_x", rotate_x_, 0.0);
    pnh_->param("rotate_y", rotate_y_, 0.0);
    pnh_->param("rotate_z", rotate_z_, 0.0);
    q_ = tf2::Quaternion();
    q_.setEuler(rotate_y_, rotate_x_, rotate_z_);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&RearrangeBoundingBox::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_bouding_box_array_ = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void RearrangeBoundingBox::configCallback(Config &config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    offset_x_ = config.offset_x;
    offset_y_ = config.offset_y;
    offset_z_ = config.offset_z;
    scale_x_ = config.scale_x;
    scale_y_ = config.scale_y;
    scale_z_ = config.scale_z;
    rotate_x_ = config.rotate_x;
    rotate_y_ = config.rotate_y;
    rotate_z_ = config.rotate_z;
    q_ = tf2::Quaternion();
    q_.setEuler(rotate_y_, rotate_x_, rotate_z_);
  }

  void RearrangeBoundingBox::subscribe() {
    sub_bounding_box_array_ = pnh_->subscribe("input", 1,
                                              &RearrangeBoundingBox::rearrangeBoundingBoxCallback, this);
  }

  void RearrangeBoundingBox::unsubscribe() {
    sub_bounding_box_array_.shutdown();
  }

  void RearrangeBoundingBox::rearrangeBoundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array) {
    boost::mutex::scoped_lock lock(mutex_);
    jsk_recognition_msgs::BoundingBoxArray bba;
    bba.header = box_array->header;
    bba.boxes = box_array->boxes;
    for(size_t i = 0; i < box_array->boxes.size(); ++i) {
      bba.boxes[i].pose.position.x += offset_x_;
      bba.boxes[i].pose.position.y += offset_y_;
      bba.boxes[i].pose.position.z += offset_z_;
      bba.boxes[i].dimensions.x *= scale_x_;
      bba.boxes[i].dimensions.y *= scale_y_;
      bba.boxes[i].dimensions.z *= scale_z_;
      bba.boxes[i].pose.orientation.x += q_.x();
      bba.boxes[i].pose.orientation.y += q_.y();
      bba.boxes[i].pose.orientation.z += q_.z();
      bba.boxes[i].pose.orientation.w += q_.w();
    }
    pub_bouding_box_array_.publish(bba);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::RearrangeBoundingBox, nodelet::Nodelet);
