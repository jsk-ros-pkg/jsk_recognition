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
#include "jsk_pcl_ros/tilt_laser_listener.h"
#include <laser_assembler/AssembleScans2.h>

namespace jsk_pcl_ros
{
  StampedJointAngle::StampedJointAngle(const std_msgs::Header& header_arg, const double& value):
    header(header_arg), value_(value)
  {

  }
  
  void TiltLaserListener::onInit()
  {
    DiagnosticNodelet::onInit();

    if (pnh_->hasParam("joint_name")) {
      pnh_->getParam("joint_name", joint_name_);
    }
    else {
      NODELET_ERROR("no ~joint_state is specified");
      return;
    }
    pnh_->param("overwrap_angle", overwrap_angle_, 0.0);
    std::string laser_type;
    pnh_->param("laser_type", laser_type, std::string("tilt_half_down"));
    if (laser_type == "tilt_half_up") {
      laser_type_ = TILT_HALF_UP;
    }
    else if (laser_type == "tilt_half_down") {
      laser_type_ = TILT_HALF_DOWN;
    }
    else if (laser_type == "tilt") {
      laser_type_ = TILT;
    }
    else if (laser_type == "infinite_spindle") {
      laser_type_ = INFINITE_SPINDLE;
    }
    else if (laser_type == "infinite_spindle_half") {
      laser_type_ = INFINITE_SPINDLE_HALF;
    }
    else {
      NODELET_ERROR("unknown ~laser_type: %s", laser_type.c_str());
      return;
    }
    pnh_->param("use_laser_assembler", use_laser_assembler_, false);
    if (use_laser_assembler_) {
      assemble_cloud_srv_
        = pnh_->serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
      cloud_pub_
        = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_cloud", 1);
    }
    prev_angle_ = 0;
    prev_velocity_ = 0;
    start_time_ = ros::Time::now();
    clear_cache_service_ = pnh_->advertiseService(
      "clear_cache", &TiltLaserListener::clearCacheCallback,
      this);
    trigger_pub_ = advertise<jsk_recognition_msgs::TimeRange>(*pnh_, "output", 1);
    
  }

  void TiltLaserListener::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &TiltLaserListener::jointCallback, this);
  }

  void TiltLaserListener::unsubscribe()
  {
    sub_.shutdown();
  }

  void TiltLaserListener::updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {

  }

  bool TiltLaserListener::clearCacheCallback(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    buffer_.clear();
    return true;
  }
  
  void TiltLaserListener::publishTimeRange(
    const ros::Time& stamp,
    const ros::Time& start,
    const ros::Time& end)
  {
    jsk_recognition_msgs::TimeRange range;
    range.header.stamp = stamp;
    range.start = start;
    range.end = end;
    trigger_pub_.publish(range);
    if (use_laser_assembler_) {
      laser_assembler::AssembleScans2 srv;
      srv.request.begin = start;
      srv.request.end = end;
      assemble_cloud_srv_.call(srv);
      sensor_msgs::PointCloud2 output_cloud = srv.response.cloud;
      output_cloud.header.stamp = stamp;
      cloud_pub_.publish(output_cloud);
    }
  }
  
  void TiltLaserListener::processTiltHalfUp(
    const ros::Time& stamp, const double& joint_angle)
  {
    double velocity = joint_angle - prev_angle_;
    if (velocity > 0 && prev_velocity_ <= 0) {
      start_time_ = stamp;
    }
    else if (velocity < 0 && prev_velocity_ >= 0) {
      publishTimeRange(stamp, start_time_, stamp);
    }
    prev_angle_ = joint_angle;
    prev_velocity_ = velocity;
  }

  void TiltLaserListener::processTiltHalfDown(
    const ros::Time& stamp, const double& joint_angle)
  {
    double velocity = joint_angle - prev_angle_;
    if (velocity < 0 && prev_velocity_ >= 0) {
      start_time_ = stamp;
    }
    else if (velocity > 0 && prev_velocity_ <= 0) {
      publishTimeRange(stamp, start_time_, stamp);
    }
    prev_angle_ = joint_angle;
    prev_velocity_ = velocity;
  }

  void TiltLaserListener::processTilt(
    const ros::Time& stamp, const double& joint_angle)
  {
    
    if (buffer_.size() > 3) {
      double direction = buffer_[buffer_.size() - 1]->getValue() - joint_angle;
      int change_count = 0;
      for (size_t i = buffer_.size() - 1; i > 0; i--) {
        double current_direction = buffer_[i-1]->getValue() - buffer_[i]->getValue();
        if (direction * current_direction < 0) {
          ++change_count;
        }
        if (change_count == 2) {
          ros::Time start_time = buffer_[i]->header.stamp;
          publishTimeRange(stamp, start_time, stamp);
          buffer_.removeBefore(buffer_[i-1]->header.stamp);
          break;
        }
        direction = current_direction;
      }
    }
    std_msgs::Header header;
    header.stamp = stamp;
    StampedJointAngle::Ptr j(new StampedJointAngle(header, joint_angle));
    if (buffer_.size() > 0 && buffer_[buffer_.size() - 1]->getValue() == joint_angle) {
      // do nothing, duplicate value
    }
    else {
      buffer_.push_back(j);
    }
  }

  void TiltLaserListener::processInfiniteSpindle(
    const ros::Time& stamp, const double& joint_angle, const double& velocity,
    const double& threshold)
  {
    if (velocity == 0) {
      return;
    }
    if (buffer_.size() > 3) {
      bool jumped = false;
      for (size_t i = buffer_.size() - 1; i > 0; i--) {
        // find jump first
        if (!jumped) {
          double direction = buffer_[i-1]->getValue() - buffer_[i]->getValue();
          if (direction * velocity > 0) {
            jumped = true;
          }
        }
        else {                  // already jumped
          if (velocity > 0) {
            if (buffer_[i-1]->getValue() < fmod(threshold - overwrap_angle_, 2.0 * M_PI)) {
              publishTimeRange(stamp, buffer_[i-1]->header.stamp, stamp);
              buffer_.removeBefore(buffer_[i-1]->header.stamp);
              break;
            }
          }
          else if (velocity < 0) {
            if (buffer_[i-1]->getValue() > fmod(threshold + overwrap_angle_, 2.0 * M_PI)) {
              publishTimeRange(stamp, buffer_[i-1]->header.stamp, stamp);
              buffer_.removeBefore(buffer_[i-1]->header.stamp);
              break;
            }
          }
        }
        //if (buffer_[i]->getValue()
      }
    }
    std_msgs::Header header;
    header.stamp = stamp;
    StampedJointAngle::Ptr j(new StampedJointAngle(header, joint_angle));
    buffer_.push_back(j);
  }
  
  void TiltLaserListener::jointCallback(
    const sensor_msgs::JointState::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    for (size_t i = 0; i < msg->name.size(); i++) {
      std::string name = msg->name[i];
      if (name == joint_name_) {
        if (laser_type_ == TILT_HALF_UP) {
          processTiltHalfUp(msg->header.stamp, msg->position[i]);
        }
        else if (laser_type_ == TILT_HALF_DOWN) {
          processTiltHalfDown(msg->header.stamp, msg->position[i]);
        }
        else if (laser_type_ == TILT) {
          processTilt(msg->header.stamp, msg->position[i]);
        }
        else if (laser_type_ == INFINITE_SPINDLE) {
          processInfiniteSpindle(msg->header.stamp,
                                 msg->position[i],
                                 msg->velocity[i],
                                 msg->position[i]);
        }
        else if (laser_type_ == INFINITE_SPINDLE_HALF) {
          processInfiniteSpindle(msg->header.stamp,
                                 fmod(msg->position[i], M_PI),
                                 msg->velocity[i],
                                 fmod(msg->position[i], M_PI));
        }
        break;
      }
    }
  }
}  

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::TiltLaserListener, nodelet::Nodelet);
