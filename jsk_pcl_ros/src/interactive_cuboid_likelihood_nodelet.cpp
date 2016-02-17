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
#include "jsk_pcl_ros/interactive_cuboid_likelihood.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>

namespace jsk_pcl_ros
{
  void InteractiveCuboidLikelihood::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_ = TfListenerSingleton::getInstance();
    pub_ = pnh_->advertise<std_msgs::Float32>("output", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    pnh_->param("frame_id", frame_id_, std::string("odom"));
    pnh_->param("sensor_frame", sensor_frame_, std::string("odom"));

    std::vector<double> initial_pos;
    std::vector<double> initial_rot;
    if (!jsk_topic_tools::readVectorParameter(*pnh_, "initial_pos", initial_pos)) {
      initial_pos.push_back(0);
      initial_pos.push_back(0);
      initial_pos.push_back(0);
    }
    if (!jsk_topic_tools::readVectorParameter(*pnh_, "initial_rot", initial_rot)) {
      initial_rot.push_back(0);
      initial_rot.push_back(0);
      initial_rot.push_back(0);
    }
    particle_.x = initial_pos[0];
    particle_.y = initial_pos[1];
    particle_.z = initial_pos[2];
    particle_.roll = initial_rot[0];
    particle_.pitch = initial_rot[1];
    particle_.yaw = initial_rot[2];
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&InteractiveCuboidLikelihood::configCallback, this, _1, _2);
    srv_->setCallback (f);
    sub_ = pnh_->subscribe("input", 1, &InteractiveCuboidLikelihood::likelihood, this);
    // Cuboid
    server_.reset(new interactive_markers::InteractiveMarkerServer(getName()));
    visualization_msgs::InteractiveMarker int_marker = particleToInteractiveMarker(particle_);
    server_->insert(int_marker, boost::bind(&InteractiveCuboidLikelihood::processFeedback, this, _1));
    server_->applyChanges();

    // SupportPlane
    plane_server_.reset(new interactive_markers::InteractiveMarkerServer(getName() + "_plane"));
    plane_pose_ = Eigen::Affine3f::Identity();
    visualization_msgs::InteractiveMarker plane_int_marker = planeInteractiveMarker();
    plane_server_->insert(plane_int_marker, boost::bind(&InteractiveCuboidLikelihood::processPlaneFeedback, this, _1));
    plane_server_->applyChanges();
    onInitPostProcess();
  }

  void InteractiveCuboidLikelihood::subscribe()
  {
    
  }

  void InteractiveCuboidLikelihood::unsubscribe()
  {

  }

  void InteractiveCuboidLikelihood::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
  {
    boost::mutex::scoped_lock lock(mutex_);
    Eigen::Affine3f pose;
    tf::poseMsgToEigen(feedback->pose, pose);
    particle_.fromEigen(pose);
  }
  
  void InteractiveCuboidLikelihood::processPlaneFeedback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
  {
    boost::mutex::scoped_lock lock(mutex_);
    tf::poseMsgToEigen(feedback->pose, plane_pose_);
  }

  void InteractiveCuboidLikelihood::likelihood(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    tf::StampedTransform transform
      = lookupTransformWithDuration(tf_, sensor_frame_, msg->header.frame_id,
                                    ros::Time(0.0),
                                    ros::Duration(0.0));
    Eigen::Vector3f vp;
    tf::vectorTFToEigen(transform.getOrigin(), vp);
    jsk_recognition_utils::Vertices vertices;
    vertices.push_back(Eigen::Vector3f(plane_pose_ * (Eigen::Vector3f::UnitX() + Eigen::Vector3f::UnitY())));
    vertices.push_back(Eigen::Vector3f(plane_pose_ * (- Eigen::Vector3f::UnitX() + Eigen::Vector3f::UnitY())));
    vertices.push_back(Eigen::Vector3f(plane_pose_ * (- Eigen::Vector3f::UnitX() - Eigen::Vector3f::UnitY())));
    vertices.push_back(Eigen::Vector3f(plane_pose_ * (Eigen::Vector3f::UnitX() - Eigen::Vector3f::UnitY())));
    Polygon::Ptr plane(new Polygon(vertices));
    //particle_.plane = plane;
    particle_.plane_index = 0;
    std::vector<Polygon::Ptr> polygons;
    polygons.push_back(plane);
    // for (size_t i = 0; i < vertices.size(); i++) {
    //   ROS_INFO("v: [%f, %f, %f]", vertices[i][0], vertices[i][1], vertices[i][2]);
    // }
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    std::vector<float> polygon_likelihood(1, 1.0);
    double l = computeLikelihood(particle_, cloud, tree, vp, polygons, polygon_likelihood, config_);
    NODELET_INFO("likelihood: %f", l);
    std_msgs::Float32 float_msg;
    float_msg.data = l;
    pub_.publish(float_msg);
  }

  void InteractiveCuboidLikelihood::configCallback(
    Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    particle_.dx = config_.dx;
    particle_.dy = config_.dy;
    particle_.dz = config_.dz;
    if (server_) {
      visualization_msgs::InteractiveMarker int_marker = particleToInteractiveMarker(particle_);
      server_->insert(int_marker, boost::bind(&InteractiveCuboidLikelihood::processFeedback, this, _1));
      server_->applyChanges();
    }
  }

  visualization_msgs::InteractiveMarker
  InteractiveCuboidLikelihood::planeInteractiveMarker()
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = getName() + "_plane";
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    visualization_msgs::Marker plane_marker;
    plane_marker.type = visualization_msgs::Marker::CUBE;
    plane_marker.scale.x = 1.0;
    plane_marker.scale.y = 1.0;
    plane_marker.scale.z = 0.01;
    plane_marker.color.r = 1.0;
    plane_marker.color.a = 1.0;
    plane_marker.pose.orientation.w = 1.0;
    box_control.markers.push_back(plane_marker);
    int_marker.controls.push_back(box_control);
    return int_marker;
  }
  
  visualization_msgs::Marker
  InteractiveCuboidLikelihood::particleToMarker(
    const Particle& p)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = p.dx;
    marker.scale.y = p.dy;
    marker.scale.z = p.dz;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    return marker;
  }
  
  visualization_msgs::InteractiveMarker InteractiveCuboidLikelihood::particleToInteractiveMarker(const Particle& p)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = getName();
    Eigen::Affine3f pose = particle_.toEigenMatrix();
    tf::poseEigenToMsg(pose, int_marker.pose);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(particleToMarker(particle_));
    int_marker.controls.push_back(box_control);
    return int_marker;
  }

  
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::InteractiveCuboidLikelihood, nodelet::Nodelet);
