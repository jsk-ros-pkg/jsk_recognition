/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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
#include <jsk_pcl_ros_utils/centroid_publisher.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <jsk_recognition_utils/geo_util.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>

namespace jsk_pcl_ros_utils
{
  void CentroidPublisher::extract(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg(*input, cloud_xyz);
    Eigen::Vector4f center;
    bool is_valid = pcl::compute3DCentroid(cloud_xyz, center);
    if (publish_tf_ && is_valid) {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
      transform.setRotation(tf::createIdentityQuaternion());
      br_.sendTransform(tf::StampedTransform(transform, input->header.stamp,
                                             input->header.frame_id, frame_));
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = center[0];
    pose.pose.position.y = center[1];
    pose.pose.position.z = center[2];
    pose.header = input->header;
    pub_pose_.publish(pose);
    geometry_msgs::PointStamped point;
    point.point.x = center[0];
    point.point.y = center[1];
    point.point.z = center[2];
    point.header = input->header;
    pub_point_.publish(point);
  }

  void CentroidPublisher::extractPolygons(const jsk_recognition_msgs::PolygonArray::ConstPtr& input)
  {
    vital_checker_->poke();

    int size = std::min(100, (int)input->polygons.size());

    geometry_msgs::PoseArray pose_array;
    pose_array.header = input->header;
    pose_array.poses.resize(size);

    for (size_t i = 0; i < size; ++i) {
      jsk_recognition_utils::Polygon::Ptr polygon
        = jsk_recognition_utils::Polygon::fromROSMsgPtr(input->polygons[i].polygon);
      Eigen::Vector3f c = polygon->centroid(), n = polygon->getNormal();
      Eigen::Vector3f ez;
      ez << 0.0f, 0.0f, 1.0f;
      Eigen::Quaternionf q;
      q.setFromTwoVectors(ez, n);

      if (publish_tf_) {
        std::stringstream ss;
        ss << frame_ << std::setfill('0') << std::setw(2) << std::right << i;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(c.x(), c.y(), c.z()));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        br_.sendTransform(tf::StampedTransform(transform, input->header.stamp,
                                               input->header.frame_id, ss.str()));
      }

      pose_array.poses[i].position.x = c.x();
      pose_array.poses[i].position.y = c.y();
      pose_array.poses[i].position.z = c.z();
      pose_array.poses[i].orientation.x = q.x();
      pose_array.poses[i].orientation.y = q.y();
      pose_array.poses[i].orientation.z = q.z();
      pose_array.poses[i].orientation.w = q.w();
    }

    pub_pose_array_.publish(pose_array);
  }

  void CentroidPublisher::subscribe()
  {
    sub_cloud_ = pnh_->subscribe("input", 1, &CentroidPublisher::extract, this);
    sub_polygons_ = pnh_->subscribe("input/polygons", 1, &CentroidPublisher::extractPolygons, this);
  }

  void CentroidPublisher::unsubscribe()
  {
    sub_cloud_.shutdown();
    sub_polygons_.shutdown();
  }
  
  void CentroidPublisher::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("publish_tf", publish_tf_, false);
    if (publish_tf_) {
      if (!pnh_->getParam("frame", frame_))
      {
        ROS_WARN("~frame is not specified, using %s", getName().c_str());
        frame_ = getName();
      }
      // do not use DiagnosticNodelet functionality when ~publish_tf is false
      pub_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output/pose", 1);
      pub_point_ = pnh_->advertise<geometry_msgs::PointStamped>(
        "output/point", 1);
      pub_pose_array_ = pnh_->advertise<geometry_msgs::PoseArray>("output/pose_array", 1);
      subscribe();
    }
    else {
      pub_pose_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose", 1);
      pub_point_ = advertise<geometry_msgs::PointStamped>(
        *pnh_, "output/point", 1);
      pub_pose_array_ = advertise<geometry_msgs::PoseArray>(*pnh_, "output/pose_array", 1);
    }

    onInitPostProcess();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::CentroidPublisher, nodelet::Nodelet);
