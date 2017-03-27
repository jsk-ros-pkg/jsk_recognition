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
#include "jsk_pcl_ros_utils/planar_pointcloud_simulator.h"
#include <image_geometry/pinhole_camera_model.h>

namespace jsk_pcl_ros_utils
{
  PlanarPointCloudSimulator::PlanarPointCloudSimulator()
  {

  }

  PlanarPointCloudSimulator::~PlanarPointCloudSimulator()
  {

  }

  void PlanarPointCloudSimulator::generate(
    const sensor_msgs::CameraInfo& info, double distance,
    pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info);
    cloud.points.resize(info.width * info.height);
    cloud.is_dense = true;
    for (size_t j = 0; j < info.height; j++) {
      for (size_t i = 0; i < info.width; i++) {
        cv::Point3d ray = model.projectPixelTo3dRay(cv::Point2d(i, j));
        pcl::PointXYZ p;
        p.x = ray.x * distance;
        p.y = ray.y * distance;
        p.z = ray.z * distance;
        cloud.points[j * info.width + i] = p;
      }
    }
    cloud.width = info.width;
    cloud.height = info.height;
  }

  void PlanarPointCloudSimulatorNodelet::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PlanarPointCloudSimulatorNodelet::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output", 1);

    onInitPostProcess();
  }

  void PlanarPointCloudSimulatorNodelet::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", 1, &PlanarPointCloudSimulatorNodelet::generate, this);
  }

  void PlanarPointCloudSimulatorNodelet::unsubscribe()
  {
    sub_.shutdown();
  }

  void PlanarPointCloudSimulatorNodelet::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    distance_ = config.distance;
  }
  
  void PlanarPointCloudSimulatorNodelet::generate(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      cloud(new pcl::PointCloud<pcl::PointXYZ>);
    impl_.generate(*info_msg, distance_, *cloud);
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = info_msg->header;
    pub_.publish(ros_cloud);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PlanarPointCloudSimulatorNodelet,
                        nodelet::Nodelet);
