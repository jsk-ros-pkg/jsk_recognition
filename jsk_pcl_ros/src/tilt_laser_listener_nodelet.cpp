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
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_topic_tools/time_accumulator.h>

namespace jsk_pcl_ros
{
  StampedJointAngle::StampedJointAngle(const std_msgs::Header& header_arg, const double& value):
    header(header_arg), value_(value)
  {

  }
  
  void TiltLaserListener::onInit()
  {
    DiagnosticNodelet::onInit();
    skip_counter_ = 0;
    if (pnh_->hasParam("joint_name")) {
      pnh_->getParam("joint_name", joint_name_);
    }
    else {
      NODELET_ERROR("no ~joint_state is specified");
      return;
    }
    pnh_->getParam("twist_frame_id", twist_frame_id_);
    pnh_->param("overwrap_angle", overwrap_angle_, 0.0);
    std::string laser_type;
    bool subscribe_joint = true;
    pnh_->param("clear_assembled_scans", clear_assembled_scans_, false);
    pnh_->param("skip_number", skip_number_, 1);
    pnh_->param("laser_type", laser_type, std::string("tilt_half_down"));
    pnh_->param("max_queue_size", max_queue_size_, 100);
    pnh_->param("publish_rate", publish_rate_, 1.0);
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
    else if (laser_type == "periodic") {
      laser_type_ = PERIODIC;
      periodic_timer_ = pnh_->createTimer(ros::Duration(1/publish_rate_), &TiltLaserListener::timerCallback, this);
      subscribe_joint = false;
    }
    else {
      NODELET_ERROR("unknown ~laser_type: %s", laser_type.c_str());
      return;
    }
    pnh_->param("not_use_laser_assembler_service", not_use_laser_assembler_service_, false);
    pnh_->param("use_laser_assembler", use_laser_assembler_, false);
    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    cloud_vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    if (use_laser_assembler_) {
      if (not_use_laser_assembler_service_) {
        sub_cloud_
          = pnh_->subscribe("input/cloud", max_queue_size_, &TiltLaserListener::cloudCallback, this);
      }
      else {
        assemble_cloud_srv_
          = pnh_->serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
      }
      cloud_pub_
        = pnh_->advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
    }
    twist_pub_ = pnh_->advertise<geometry_msgs::TwistStamped>("output_velocity", 1);
    prev_angle_ = 0;
    prev_velocity_ = 0;
    start_time_ = ros::Time::now();
    clear_cache_service_ = pnh_->advertiseService(
      "clear_cache", &TiltLaserListener::clearCacheCallback,
      this);
    trigger_pub_ = advertise<jsk_recognition_msgs::TimeRange>(*pnh_, "output", 1);
    if(subscribe_joint) {
      sub_ = pnh_->subscribe("input", max_queue_size_, &TiltLaserListener::jointCallback, this);
    }

    onInitPostProcess();
  }

  void TiltLaserListener::timerCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    publishTimeRange(e.current_real, e.last_real, e.current_real);
    start_time_ = e.current_real; // not used??
  }

  void TiltLaserListener::subscribe()
  {
    
  }

  void TiltLaserListener::unsubscribe()
  {

  }

  void TiltLaserListener::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(cloud_mutex_);
    if (vital_checker_->isAlive()) {
      if (not_use_laser_assembler_service_ && 
          use_laser_assembler_) {
        if (cloud_vital_checker_->isAlive()) {
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                       getName() + " running");
        }
        else {
          stat.summary(diagnostic_error_level_,
                       "~input/cloud is not activate");
        }
        stat.add("scan queue", cloud_buffer_.size());
      }
      else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     getName() + " running");
      }
      
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        name_, vital_checker_, stat, diagnostic_error_level_);
    }
  }

  void TiltLaserListener::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_vital_checker_->poke();
    cloud_buffer_.push_back(msg);
  }

  bool TiltLaserListener::clearCacheCallback(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    buffer_.clear();
    return true;
  }

  void TiltLaserListener::getPointCloudFromLocalBuffer(
    const std::vector<sensor_msgs::PointCloud2::ConstPtr>& target_clouds,
    sensor_msgs::PointCloud2& output_cloud)
  {
    if (target_clouds.size() > 0) {
      output_cloud.fields = target_clouds[0]->fields;
      output_cloud.is_bigendian = target_clouds[0]->is_bigendian;
      output_cloud.is_dense = true;
      output_cloud.point_step = target_clouds[0]->point_step;
      size_t point_num = 0;
      size_t data_num = 0;
      for (size_t i = 0; i < target_clouds.size(); i++) {
        data_num += target_clouds[i]->row_step;
        point_num += target_clouds[i]->width * target_clouds[i]->height;
      }
      output_cloud.data.reserve(data_num);
      for (size_t i = 0; i < target_clouds.size(); i++) {
        std::copy(target_clouds[i]->data.begin(),
                  target_clouds[i]->data.end(),
                  std::back_inserter(output_cloud.data));
      }
      output_cloud.header.frame_id = target_clouds[0]->header.frame_id;
      output_cloud.width = point_num;
      output_cloud.height = 1;
      output_cloud.row_step = data_num;
    }
    else {
      NODELET_WARN("target_clouds size is 0");
    }
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

    // publish velocity
    // only publish if twist_frame_id is not empty
    if (!twist_frame_id_.empty()) {
      // simply compute from the latest velocity
      if (buffer_.size() >= 2) {
        // at least, we need two joint angles to
        // compute velocity
        StampedJointAngle::Ptr latest = buffer_[buffer_.size() - 1];
        StampedJointAngle::Ptr last_second = buffer_[buffer_.size() - 2];
        double value_diff = latest->getValue() - last_second->getValue();
        double time_diff = (latest->header.stamp - last_second->header.stamp).toSec();
        double velocity = value_diff / time_diff;
        geometry_msgs::TwistStamped twist;
        twist.header.frame_id = twist_frame_id_;
        twist.header.stamp = latest->header.stamp;
        if (laser_type_ == INFINITE_SPINDLE_HALF || // roll laser
            laser_type_ == INFINITE_SPINDLE) {
          twist.twist.angular.x = velocity;
        }
        else {                  // tilt laser
          twist.twist.angular.y = velocity;
        }
        twist_pub_.publish(twist);
      }
    }

    if (use_laser_assembler_) {
      if (skip_counter_++ % skip_number_ == 0) {
        laser_assembler::AssembleScans2 srv;
        srv.request.begin = start;
        srv.request.end = end;
        try {
          if (!not_use_laser_assembler_service_) {
            if (assemble_cloud_srv_.call(srv)) {
              sensor_msgs::PointCloud2 output_cloud = srv.response.cloud;
              output_cloud.header.stamp = stamp;
              cloud_pub_.publish(output_cloud);
            }
            else {
              NODELET_ERROR("Failed to call assemble cloud service");
            }
          }
          else {
            // Assemble cloud from local buffer
            std::vector<sensor_msgs::PointCloud2::ConstPtr> target_clouds;
            {
              boost::mutex::scoped_lock lock(cloud_mutex_);
              if (cloud_buffer_.size() == 0) {
                return;
              }
              for (size_t i = 0; i < cloud_buffer_.size(); i++) {
                ros::Time the_stamp = cloud_buffer_[i]->header.stamp;
                if (the_stamp > start && the_stamp < end) {
                  target_clouds.push_back(cloud_buffer_[i]);
                }
              }
              if (clear_assembled_scans_) {
                cloud_buffer_.removeBefore(end);
              }
              else {
                cloud_buffer_.removeBefore(start);
              }
            }
            sensor_msgs::PointCloud2 output_cloud;
            getPointCloudFromLocalBuffer(target_clouds, output_cloud);
            output_cloud.header.stamp = stamp;
            cloud_pub_.publish(output_cloud);
          }
        }
        catch (...) {
          NODELET_ERROR("Exception in calling assemble cloud service");
        }
      }
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
          if (clear_assembled_scans_) {
            buffer_.removeBefore(stamp);
          }
          else {
            buffer_.removeBefore(buffer_[i-1]->header.stamp);
          }
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
          double direction = fmod(buffer_[i-1]->getValue(), 2.0 * M_PI) - fmod(buffer_[i]->getValue(), 2.0 * M_PI);
          if (direction * velocity > 0) {
            jumped = true;
          }
        }
        else {                  // already jumped
          if (velocity > 0) {
            if (fmod(buffer_[i-1]->getValue(), 2.0 * M_PI) < fmod(threshold - overwrap_angle_, 2.0 * M_PI)) {
              publishTimeRange(stamp, buffer_[i-1]->header.stamp, stamp);
              if (clear_assembled_scans_) {
                buffer_.removeBefore(stamp);
              }
              else {
                buffer_.removeBefore(buffer_[i-1]->header.stamp);
              }
              break;
            }
          }
          else if (velocity < 0) {
            if (fmod(buffer_[i-1]->getValue(), 2.0 * M_PI) > fmod(threshold + overwrap_angle_, 2.0 * M_PI)) {
              publishTimeRange(stamp, buffer_[i-1]->header.stamp, stamp);
              if (clear_assembled_scans_) {
                buffer_.removeBefore(stamp);
              }
              else {
                buffer_.removeBefore(buffer_[i-1]->header.stamp);
              }
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
    for (size_t i = 0; i < msg->name.size(); i++) {
      std::string name = msg->name[i];
      if (name == joint_name_) {
        vital_checker_->poke();
        if(msg->position.size() <= i) {
          ROS_WARN("size of position (%zu) is smaller than joint(%s) position(%zu)",
                   msg->position.size(), name.c_str(), i);
          return;
        }
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
          if(msg->velocity.size() <= i) {
            ROS_WARN("size of velocity (%zu) is smaller than joint(%s) position(%zu)",
                     msg->velocity.size(), name.c_str(), i);
            return;
          }
          processInfiniteSpindle(msg->header.stamp,
                                 msg->position[i],
                                 msg->velocity[i],
                                 msg->position[i]);
        }
        else if (laser_type_ == INFINITE_SPINDLE_HALF) {
          if(msg->velocity.size() <= i) {
            ROS_WARN("size of velocity (%zu) is smaller than joint(%s) position(%zu)",
                     msg->velocity.size(), name.c_str(), i);
            return;
          }
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
