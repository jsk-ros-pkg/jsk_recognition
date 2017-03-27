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

#include "jsk_pcl_ros_utils/polygon_array_foot_angle_likelihood.h"
#include "jsk_recognition_utils/tf_listener_singleton.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>

namespace jsk_pcl_ros_utils
{
  void PolygonArrayFootAngleLikelihood::onInit()
  {
    DiagnosticNodelet::onInit();
    if (!pnh_->getParam("target_frame_id", target_frame_id_)) {
      ROS_ERROR("You need to specify ~target_frame_id");
      return;
    }
    pnh_->param("tf_queue_size", tf_queue_size_, 10);
    std::vector<double> axis(3);
    if (!jsk_topic_tools::readVectorParameter(*pnh_, "axis", axis)) {
      axis[0] = 1;
      axis[1] = 0;
      axis[2] = 0;
    }
    axis_[0] = axis[0];
    axis_[1] = axis[1];
    axis_[2] = axis[2];
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pub_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void PolygonArrayFootAngleLikelihood::subscribe()
  {
    sub_.subscribe(*pnh_, "input", 10);
    tf_filter_.reset(new tf::MessageFilter<jsk_recognition_msgs::PolygonArray>(
                       sub_,
                       *tf_listener_,
                       target_frame_id_,
                       tf_queue_size_));
    tf_filter_->registerCallback(
      boost::bind(&PolygonArrayFootAngleLikelihood::likelihood, this, _1));
  }

  void PolygonArrayFootAngleLikelihood::unsubscribe()
  {
    sub_.unsubscribe();
  }

  void PolygonArrayFootAngleLikelihood::likelihood(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    jsk_recognition_msgs::PolygonArray new_msg(*msg);

    try
    {
      // Resolve tf
      // ConstPtrmpute position of target_frame_id
      // respected from msg->header.frame_id
      tf::StampedTransform transform;
      tf_listener_->lookupTransform(
        target_frame_id_, msg->header.frame_id, msg->header.stamp, transform);
      Eigen::Affine3f pose;
      tf::transformTFToEigen(transform, pose);

      Eigen::Vector3f reference_axis = pose.rotation() * axis_;

      std::vector<double> distances; 
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        jsk_recognition_utils::Polygon::Ptr polygon
          = jsk_recognition_utils::Polygon::fromROSMsgPtr(msg->polygons[i].polygon);
        double distance;
        Eigen::Vector3f foot = polygon->nearestPoint(pose.translation(), distance);
        Eigen::Vector3f foot_dir = (foot - pose.translation()).normalized();
        double ang_distance = std::abs(reference_axis.dot(foot_dir));
        distances.push_back(ang_distance);
      }

      // Normalization
      for (size_t i = 0; i < distances.size(); i++) {
        // double normalized_distance
        //   = (distances[i] - min_distance) / (max_distance - min_distance);
        double likelihood = 1 / (1 + (distances[i] - 1) * (distances[i] - 1));
        
        if (msg->likelihood.size() == 0) {
          new_msg.likelihood.push_back(likelihood);
        }
        else {
          new_msg.likelihood[i] = new_msg.likelihood[i] * likelihood;
        }
      }
      pub_.publish(new_msg);
    }
    catch (...)
    {
      NODELET_ERROR("Unknown transform error");
    }
    
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonArrayFootAngleLikelihood,
                        nodelet::Nodelet);
