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

#include "jsk_perception/project_image_point.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>

namespace jsk_perception
{
  void ProjectImagePoint::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ProjectImagePoint::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<geometry_msgs::PointStamped>(*pnh_, "output", 1);
    pub_vector_ = advertise<geometry_msgs::Vector3Stamped>(
      *pnh_, "output/ray", 1);
    onInitPostProcess();
 }

  void ProjectImagePoint::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ProjectImagePoint::project, this);
    sub_camera_info_ = pnh_->subscribe("input/camera_info", 1,
                                       &ProjectImagePoint::cameraInfoCallback,
                                       this);
    ros::V_string names = boost::assign::list_of("~input")("~input/camera_info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ProjectImagePoint::unsubscribe()
  {
    sub_.shutdown();
    sub_camera_info_.shutdown();
  }

  void ProjectImagePoint::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    z_ = config.z;
  }

  void ProjectImagePoint::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_ = msg;
  }

  void ProjectImagePoint::project(
    const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    if (!camera_info_) {
      NODELET_WARN(
        "[ProjectImagePoint::project] camera info is not yet available");
      return;
    }
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_);
    cv::Point3d ray = model.projectPixelTo3dRay(
      cv::Point2d(msg->point.x, msg->point.y));
    geometry_msgs::Vector3Stamped vector;
    vector.header.frame_id = camera_info_->header.frame_id;
    vector.header = msg->header;
    vector.vector.x = ray.x;
    vector.vector.y = ray.y;
    vector.vector.z = ray.z;
    pub_vector_.publish(vector);
    if (ray.z == 0.0) {
      NODELET_ERROR("Z value of projected ray is 0");
      return;
    }
    double alpha = z_ / ray.z;
    geometry_msgs::PointStamped point;
    point.header = msg->header;
    point.header.frame_id = camera_info_->header.frame_id;
    point.point.x = ray.x * alpha;
    point.point.y = ray.y * alpha;
    point.point.z = ray.z * alpha;
    pub_.publish(point);
    
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::ProjectImagePoint, nodelet::Nodelet);
