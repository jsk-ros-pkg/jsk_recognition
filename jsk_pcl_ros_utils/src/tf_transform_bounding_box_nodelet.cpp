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

#include "jsk_pcl_ros_utils/tf_transform_bounding_box.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros_utils
{

  void TfTransformBoundingBox::onInit()
  {
    DiagnosticNodelet::onInit();
    if (!pnh_->getParam("target_frame_id", target_frame_id_)) {
      ROS_FATAL("~target_frame_id is not specified");
      return;
    }

    pnh_->param("use_latest_tf", use_latest_tf_, false);
    pnh_->param("tf_queue_size", tf_queue_size_, 10);
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pub_ = advertise<jsk_recognition_msgs::BoundingBox>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void TfTransformBoundingBox::subscribe()
  {
    if (use_latest_tf_) {
      sub_ = pnh_->subscribe("input", 1, &TfTransformBoundingBox::transform, this);
    }
    else {
      sub_filter_.subscribe(*pnh_, "input", 10);
      tf_filter_.reset(new tf::MessageFilter<jsk_recognition_msgs::BoundingBox>(
                         sub_filter_,
                         *tf_listener_,
                         target_frame_id_,
                         tf_queue_size_));
      tf_filter_->registerCallback(boost::bind(&TfTransformBoundingBox::transform, this, _1));
    }
  }

  void TfTransformBoundingBox::unsubscribe()
  {
    if (use_latest_tf_) {
      sub_.shutdown();
    }
    else {
      sub_filter_.unsubscribe();
    }
  }

  void TfTransformBoundingBox::transform(
    const jsk_recognition_msgs::BoundingBox::ConstPtr& msg)
  {
    vital_checker_->poke();
    try
    {
      jsk_recognition_msgs::BoundingBox transformed_box;
      transformed_box.header.stamp = msg->header.stamp;
      transformed_box.header.frame_id = target_frame_id_;
      transformed_box.dimensions = msg->dimensions;
      tf::StampedTransform tf_transform;
      if (use_latest_tf_) {
        tf_listener_->lookupTransform(target_frame_id_, msg->header.frame_id,
                                      ros::Time(0), tf_transform);
      }
      else {
        tf_listener_->lookupTransform(target_frame_id_, msg->header.frame_id,
                                      msg->header.stamp, tf_transform);
      }
      Eigen::Affine3f pose;
      tf::poseMsgToEigen(msg->pose, pose);
      Eigen::Affine3f transform;
      tf::transformTFToEigen(tf_transform, transform);
      Eigen::Affine3f new_pose = transform * pose;
      tf::poseEigenToMsg(new_pose, transformed_box.pose);
      pub_.publish(transformed_box);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (...)
    {
      NODELET_ERROR("Unknown transform error");
    }
  }

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::TfTransformBoundingBox, nodelet::Nodelet);
