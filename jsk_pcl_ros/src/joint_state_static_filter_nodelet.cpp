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

#include "jsk_pcl_ros/joint_state_static_filter.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <jsk_recognition_utils/pcl_util.h>

namespace jsk_pcl_ros
{
  void JointStateStaticFilter::onInit()
  {
    DiagnosticNodelet::onInit();

    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    joint_vital_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    if (!jsk_topic_tools::readVectorParameter(*pnh_,
                                              "joint_names",
                                              joint_names_) ||
        joint_names_.size() == 0) {
      NODELET_FATAL("NO ~joint_names is specified");
      return;
    }
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void JointStateStaticFilter::subscribe()
  {
    sub_joint_ = pnh_->subscribe("input_joint_state", 1,
                                 &JointStateStaticFilter::jointStateCallback,
                                 this);
    sub_input_ = pnh_->subscribe("input", 1,
                                 &JointStateStaticFilter::filter,
                                 this);
  }

  void JointStateStaticFilter::unsubscribe()
  {
    sub_joint_.shutdown();
    sub_input_.shutdown();
  }

  void JointStateStaticFilter::filter(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("Pointcloud Callback");
    vital_checker_->poke();
    if (isStatic(msg->header.stamp)) {
      ROS_DEBUG("static");
      pub_.publish(msg);
    }
    else {
      ROS_DEBUG("not static");
    }
    diagnostic_updater_->update();
  }

  std::vector<double>
  JointStateStaticFilter::filterJointState(
    const sensor_msgs::JointState::ConstPtr& msg)
  {
    std::vector<double> ret;
    for (size_t i = 0; i < joint_names_.size(); i++) {
      std::string target_joint = joint_names_[i];
      // lookup target_joint from msg
      bool find_joint = false;
      for (size_t j = 0; j < msg->name.size(); j++) {
        if (target_joint == msg->name[j]) {
          ret.push_back(msg->position[j]);
          find_joint = true;
          break;
        }
      }
      if (!find_joint) {
        return std::vector<double>();
      }
    }
    return ret;
  }

  bool JointStateStaticFilter::isStatic(
    const ros::Time& stamp)
  {
    double min_diff = DBL_MAX;
    bool min_value = false;
    for (boost::circular_buffer<StampedBool>::iterator it = buf_.begin();
         it != buf_.end();
         ++it) {
      double diff = fabs((it->get<0>() - stamp).toSec());
      if (diff < min_diff) {
        min_value = it->get<1>();
        min_diff = diff;
      }
    }
    NODELET_DEBUG("min_diff: %f", min_diff);
    return min_value;
  }
  
  void JointStateStaticFilter::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("jointCallback");
    // filter out joints based on joint names
    std::vector<double> joints = filterJointState(msg);
    if (joints.size() == 0) {
      NODELET_DEBUG("cannot find the joints from the input message");
      return;
    }
    joint_vital_->poke();

    // check the previous state...
    if (previous_joints_.size() > 0) {
      // compute velocity
      for (size_t i = 0; i < previous_joints_.size(); i++) {
        // NODELET_INFO("[%s] diff: %f", joint_names_[i].c_str(),
        //              fabs(previous_joints_[i] - joints[i]));
        if (fabs(previous_joints_[i] - joints[i]) > eps_) {
          buf_.push_front(boost::make_tuple<ros::Time, bool>(
                            msg->header.stamp, false));
          previous_joints_ = joints;
          return;
        }
      }
      buf_.push_front(boost::make_tuple<ros::Time, bool>(
                        msg->header.stamp, true));
    }
    previous_joints_ = joints;
  }
  
  void JointStateStaticFilter::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      if (joint_vital_->isAlive()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     name_ + " running");
      }
      else {
        jsk_topic_tools::addDiagnosticErrorSummary(
          name_, joint_vital_, stat, diagnostic_error_level_);
      }
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        name_, vital_checker_, stat, diagnostic_error_level_);
    }

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::JointStateStaticFilter, nodelet::Nodelet);
