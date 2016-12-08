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
#include "jsk_pcl_ros/octree_voxel_grid.h"
#include <std_msgs/Float32.h>
#include <jsk_topic_tools/color_utils.h>
#include <pcl/common/common.h>
#include <jsk_recognition_utils/pcl_ros_util.h>

namespace jsk_pcl_ros
{
  template <class PointT>
  void OctreeVoxelGrid::generateVoxelCloudImpl(const sensor_msgs::PointCloud2ConstPtr& input_msg)
  {

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloud_voxeled (new pcl::PointCloud<PointT> ());
    pcl::fromROSMsg(*input_msg, *cloud);

    // generate octree
    typename pcl::octree::OctreePointCloud<PointT> octree(resolution_);
    // add point cloud to octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    // get points where grid is occupied
    typename pcl::octree::OctreePointCloud<PointT>::AlignedPointTVector point_vec;
    octree.getOccupiedVoxelCenters(point_vec);
    // put points into point cloud
    cloud_voxeled->width = point_vec.size();
    cloud_voxeled->height = 1;
    for (int i = 0; i < point_vec.size(); i++) {
      cloud_voxeled->push_back(point_vec[i]);
    }

    // publish point cloud
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*cloud_voxeled, output_msg);
    output_msg.header = input_msg->header;
    pub_cloud_.publish(output_msg);

    // publish marker
    if (publish_marker_flag_) {
      visualization_msgs::Marker marker_msg;
      marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
      marker_msg.scale.x = resolution_;
      marker_msg.scale.y = resolution_;
      marker_msg.scale.z = resolution_;
      marker_msg.header = input_msg->header;
      marker_msg.pose.orientation.w = 1.0;
      if (marker_color_ == "flat") {
        marker_msg.color = jsk_topic_tools::colorCategory20(0);
        marker_msg.color.a = marker_color_alpha_;
      }
      
      // compute min and max
      Eigen::Vector4f minpt, maxpt;
      pcl::getMinMax3D<PointT>(*cloud_voxeled, minpt, maxpt);
      PointT p;
      for (size_t i = 0; i < cloud_voxeled->size(); i++) {
        p = cloud_voxeled->at(i);
        geometry_msgs::Point point_ros;
        point_ros.x = p.x;
        point_ros.y = p.y;
        point_ros.z = p.z;
        marker_msg.points.push_back(point_ros);
        if (marker_color_ == "flat") {
          marker_msg.colors.push_back(jsk_topic_tools::colorCategory20(0));
        }
        else if (marker_color_ == "z") {
          marker_msg.colors.push_back(jsk_topic_tools::heatColor((p.z - minpt[2]) / (maxpt[2] - minpt[2])));
        }
        else if (marker_color_ == "x") {
          marker_msg.colors.push_back(jsk_topic_tools::heatColor((p.x - minpt[0]) / (maxpt[0] - minpt[0])));
        }
        else if (marker_color_ == "y") {
          marker_msg.colors.push_back(jsk_topic_tools::heatColor((p.y - minpt[1]) / (maxpt[1] - minpt[1])));
        }
        marker_msg.colors[marker_msg.colors.size() - 1].a = marker_color_alpha_;
      }
      pub_marker_.publish(marker_msg);

      // publish marker_array also for convenience
      visualization_msgs::MarkerArray marker_array_msg;
      marker_array_msg.markers.push_back(marker_msg);
      pub_marker_array_.publish(marker_array_msg);
    }
  }

  void OctreeVoxelGrid::generateVoxelCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (resolution_ == 0.0) {
      pub_cloud_.publish(input_msg);
      
    }
    else {
      if (jsk_recognition_utils::hasField("rgb", *input_msg) &&
          jsk_recognition_utils::hasField("normal_x", *input_msg) &&
          jsk_recognition_utils::hasField("normal_y", *input_msg) &&
          jsk_recognition_utils::hasField("normal_z", *input_msg)) {
        generateVoxelCloudImpl<pcl::PointXYZRGBNormal>(input_msg);
      }
      else if (jsk_recognition_utils::hasField("rgb", *input_msg)) {
        generateVoxelCloudImpl<pcl::PointXYZRGB>(input_msg);
      }
      else if (jsk_recognition_utils::hasField("normal_x", *input_msg) &&
               jsk_recognition_utils::hasField("normal_y", *input_msg) &&
               jsk_recognition_utils::hasField("normal_z", *input_msg)) {
        generateVoxelCloudImpl<pcl::PointNormal>(input_msg);
      }
      else {
        generateVoxelCloudImpl<pcl::PointXYZ>(input_msg);
      }
    }
    std_msgs::Float32 resolution;
    resolution.data = resolution_;
    pub_octree_resolution_.publish(resolution);
  }

  void OctreeVoxelGrid::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &OctreeVoxelGrid::generateVoxelCloud, this);
  }

  void OctreeVoxelGrid::unsubscribe()
  {
    sub_input_.shutdown();
  }

  void OctreeVoxelGrid::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    resolution_ = config.resolution;
    publish_marker_flag_ = config.publish_marker;
    marker_color_ = config.marker_color;
    marker_color_alpha_ = config.marker_color_alpha;
  }

  void OctreeVoxelGrid::onInit(void)
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OctreeVoxelGrid::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_marker_ = advertise<visualization_msgs::Marker>(*pnh_, "output_marker", 1);
    pub_marker_array_ = advertise<visualization_msgs::MarkerArray>(*pnh_, "output_marker_array", 1);
    pub_octree_resolution_ = advertise<std_msgs::Float32>(*pnh_, "output_resolution", 1);

    onInitPostProcess();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OctreeVoxelGrid, nodelet::Nodelet);
