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
#include "jsk_pcl_ros_utils/spherical_pointcloud_simulator.h"

namespace jsk_pcl_ros_utils
{
  void SphericalPointCloudSimulator::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->getParam("frame_id", frame_id_);
    rotate_velocity_ = 0.5;
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&SphericalPointCloudSimulator::configCallback, this, _1, _2);
    srv_->setCallback (f);

    double rate;
    if (pnh_->getParam("rate", rate)) {
      timer_ = pnh_->createTimer(
        ros::Duration(1 / rate), boost::bind(
          &SphericalPointCloudSimulator::timerCallback,
          this,
          _1));
    }
    pub_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void SphericalPointCloudSimulator::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", 1, &SphericalPointCloudSimulator::generate, this);
  }

  void SphericalPointCloudSimulator::unsubscribe()
  {
    sub_.shutdown();
  }

  void SphericalPointCloudSimulator::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    r_ = config.r;
    min_phi_ = config.min_phi;
    max_phi_ = config.max_phi;
    scan_range_ = config.scan_range;
    scan_num_ = config.scan_num;
    fps_ = config.fps;
  }

  void SphericalPointCloudSimulator::timerCallback(
    const ros::TimerEvent& event)
  {
    // make up pointcloud and call generate
    sensor_msgs::PointCloud2 dummy_cloud;
    dummy_cloud.header.stamp = ros::Time::now();
    dummy_cloud.header.frame_id = "map"; // default is map
    generate(boost::make_shared<sensor_msgs::PointCloud2>(dummy_cloud));
  }
  
  void SphericalPointCloudSimulator::generate(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // 40fps 
    int phi_num = 2 * M_PI / rotate_velocity_ * fps_;
    int point_num = phi_num * scan_num_;
    cloud->points.resize(point_num);
    int i = 0;
    for (int phi_i = 0; phi_i < phi_num; phi_i++) {
      double phi = (float)phi_i / phi_num * (max_phi_ - min_phi_) + min_phi_;
      Eigen::Affine3f trans = getPlane(phi);
      for (int theta_i = 0; theta_i < scan_num_; theta_i++) {
        double theta = theta_i * scan_range_ / scan_num_ - scan_range_ / 2.0;
        pcl::PointXYZ p = getPoint(r_, theta, trans);
        cloud->points[i] = p;
        i++;
      }
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header.stamp = msg->header.stamp;
    if (!frame_id_.empty()) {
      ros_cloud.header.frame_id = frame_id_;
    }
    else {
      ros_cloud.header.frame_id = cloud->header.frame_id;
    }
    pub_.publish(ros_cloud);
  }

  Eigen::Affine3f SphericalPointCloudSimulator::getPlane(double phi)
  {
    Eigen::Vector3f norm(0, sin(phi), cos(phi));
    Eigen::Quaternionf rot;
    rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), norm);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans = trans * rot;
    return trans;
  }

  pcl::PointXYZ SphericalPointCloudSimulator::getPoint(
    double r, double theta, const Eigen::Affine3f& trans)
  {
    Eigen::Vector3f local_p(r * cos(theta), r * sin(theta), 0);
    Eigen::Vector3f world_p = trans * local_p;
    pcl::PointXYZ p;
    p.getVector3fMap() = world_p;
    return p;
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (
  jsk_pcl_ros_utils::SphericalPointCloudSimulator, nodelet::Nodelet);
