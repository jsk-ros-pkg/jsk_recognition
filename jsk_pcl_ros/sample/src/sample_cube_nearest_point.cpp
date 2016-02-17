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
#include <ros/ros.h>
#include "jsk_pcl_ros/geo_util.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>

using namespace jsk_pcl_ros;

Cube cube(Eigen::Vector3f(1, 0, 0),
          Eigen::Quaternionf(0.108755, 0.088921, 0.108755, 0.984092),
          Eigen::Vector3f(0.3, 0.3, 0.3));
  
ros::Publisher pub_nearest_point;
void processFeedbackCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  Eigen::Vector3f p(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  double distance;
  Eigen::Vector3f q = cube.nearestPoint(p, distance);
  ROS_INFO("distance: %f", distance);
  geometry_msgs::PointStamped ps;
  ps.header = feedback->header;
  ps.point.x = q[0];
  ps.point.y = q[1];
  ps.point.z = q[2];
  pub_nearest_point.publish(ps);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube_nearest_point");
  ros::NodeHandle nh("~");
  pub_nearest_point = nh.advertise<geometry_msgs::PointStamped>("nearest_point", 1);
  interactive_markers::InteractiveMarkerServer server("sample_cube_nearest_point");
  
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.name = "marker";
  int_marker.pose.orientation.w = 1.0;
  int_marker.pose.position.x = -2.0;
  visualization_msgs::Marker object_marker;
  object_marker.type = visualization_msgs::Marker::SPHERE;
  object_marker.scale.x = 0.1;
  object_marker.scale.y = 0.1;
  object_marker.scale.z = 0.1;
  object_marker.color.r = 1.0;
  object_marker.color.g = 1.0;
  object_marker.color.b = 1.0;
  object_marker.color.a = 1.0;
  visualization_msgs::InteractiveMarkerControl object_marker_control;
  object_marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  object_marker_control.always_visible = true;
  object_marker_control.markers.push_back(object_marker);
  int_marker.controls.push_back(object_marker_control);
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
    
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
  server.insert(int_marker, &processFeedbackCB);
  server.applyChanges();
  
  ros::Publisher pub_box_array = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bbox_array", 1, true);
  jsk_recognition_msgs::BoundingBox box = cube.toROSMsg();
  box.header.frame_id = "world";
  box.header.stamp = ros::Time::now();
  jsk_recognition_msgs::BoundingBoxArray box_array;
  box_array.boxes.push_back(box);
  box_array.header = box.header;
  pub_box_array.publish(box_array);
  ros::spin();
}
