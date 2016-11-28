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
#include "jsk_pcl_ros/collision_detector.h"
#include <cmath>

namespace jsk_pcl_ros
{
  void CollisionDetector::onInit()
  {
    DiagnosticNodelet::onInit();
    initSelfMask();
    pnh_->param<std::string>("world_frame_id", world_frame_id_, "map");
    sub_ = pnh_->subscribe("input", 1, &CollisionDetector::pointcloudCallback, this);
    service_ = pnh_->advertiseService("check_collision", &CollisionDetector::serviceCallback, this);
    onInitPostProcess();
 }

  void CollisionDetector::subscribe()
  {
  }

  void CollisionDetector::unsubscribe()
  {
  }

  void CollisionDetector::initSelfMask()
  {
    // genearte urdf model
    std::string content;
    urdf::Model urdf_model;
    if (nh_->getParam("robot_description", content)) {
      if (!urdf_model.initString(content)) {
        NODELET_ERROR("Unable to parse URDF description!");
      }
    }

    std::string root_link_id;
    std::string world_frame_id;
    pnh_->param<std::string>("root_link_id", root_link_id, "BODY");
    pnh_->param<std::string>("world_frame_id", world_frame_id, "map");

    double default_padding, default_scale;
    pnh_->param("self_see_default_padding", default_padding, 0.01);
    pnh_->param("self_see_default_scale", default_scale, 1.0);
    std::vector<robot_self_filter::LinkInfo> links;
    std::string link_names;

    if (!pnh_->hasParam("self_see_links")) {
      NODELET_WARN("No links specified for self filtering.");
    } else {
      XmlRpc::XmlRpcValue ssl_vals;;
      pnh_->getParam("self_see_links", ssl_vals);
      if (ssl_vals.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        NODELET_WARN("Self see links need to be an array");
      } else {
        if (ssl_vals.size() == 0) {
          NODELET_WARN("No values in self see links array");
        } else {
          for (int i = 0; i < ssl_vals.size(); i++) {
            robot_self_filter::LinkInfo li;
            if (ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
              NODELET_WARN("Self see links entry %d is not a structure.  Stopping processing of self see links",i);
              break;
            }
            if (!ssl_vals[i].hasMember("name")) {
              NODELET_WARN("Self see links entry %d has no name.  Stopping processing of self see links",i);
              break;
            }
            li.name = std::string(ssl_vals[i]["name"]);
            if (!ssl_vals[i].hasMember("padding")) {
              NODELET_DEBUG("Self see links entry %d has no padding.  Assuming default padding of %g",i,default_padding);
              li.padding = default_padding;
            } else {
              li.padding = ssl_vals[i]["padding"];
            }
            if (!ssl_vals[i].hasMember("scale")) {
              NODELET_DEBUG("Self see links entry %d has no scale.  Assuming default scale of %g",i,default_scale);
              li.scale = default_scale;
            } else {
              li.scale = ssl_vals[i]["scale"];
            }
            links.push_back(li);
          }
        }
      }
    }
    self_mask_ = boost::shared_ptr<robot_self_filter::SelfMaskUrdfRobot>(new robot_self_filter::SelfMaskUrdfRobot(tf_listener_, tf_broadcaster_, links, urdf_model, root_link_id, world_frame_id));
  }

  void CollisionDetector::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("update pointcloud.");

    pcl::fromROSMsg(*msg, cloud_);
    cloud_frame_id_ = msg->header.frame_id;
    cloud_stamp_ = msg->header.stamp;
  }

  bool CollisionDetector::serviceCallback(jsk_recognition_msgs::CheckCollision::Request &req,
                                          jsk_recognition_msgs::CheckCollision::Response &res)
  {
    sensor_msgs::JointState joint = req.joint;
    geometry_msgs::PoseStamped pose = req.pose;
    res.result = checkCollision(joint, pose);
    return true;
  }

  bool CollisionDetector::checkCollision(const sensor_msgs::JointState& joint,
                                         const geometry_msgs::PoseStamped& pose)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("checkCollision is called.");

    // calculate the sensor transformation
    tf::StampedTransform sensor_to_world_tf;
    try {
      tf_listener_.lookupTransform(world_frame_id_, cloud_frame_id_, cloud_stamp_, sensor_to_world_tf);
    } catch (tf::TransformException& ex) {
      NODELET_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
      return false;
    }

    // transform point cloud
    Eigen::Matrix4f sensor_to_world;
    pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
    pcl::transformPointCloud(cloud_, cloud_, sensor_to_world);

    self_mask_->assumeFrameFromJointAngle(joint, pose);

    // check containment for all point cloud
    bool contain_flag = false;
    pcl::PointXYZ p;
    for (size_t i = 0; i < cloud_.size(); i++) {
      p = cloud_.at(i);
      if (finite(p.x) && finite(p.y) && finite(p.z) &&
         self_mask_->getMaskContainment(p.x, p.y, p.z) == robot_self_filter::INSIDE) {
        contain_flag = true;
        break;
      }
    }

    if (contain_flag) {
      NODELET_INFO("collision!");
    } else {
      NODELET_INFO("no collision!");
    }
    return contain_flag;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::CollisionDetector, nodelet::Nodelet);
