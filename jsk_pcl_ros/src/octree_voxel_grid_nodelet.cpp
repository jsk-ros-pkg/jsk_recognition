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
#include <jsk_topic_tools/color_utils.h>

namespace jsk_pcl_ros
{
  void OctreeVoxelGrid::generateVoxelCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_voxeled (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg(*input_msg, *cloud_xyz);

    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;
    bool down_sampling = true;
    // instantiate point cloud compression for encoding and decoding
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudEncoder(compressionProfile, show_statistics_,  point_resolution_,  octree_resolution_, down_sampling);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudDecoder;

    std::stringstream compressed_data;
    // compress point cloud
    PointCloudEncoder.encodePointCloud(cloud_xyz, compressed_data);
    // decompress point cloud
    PointCloudDecoder.decodePointCloud(compressed_data, cloud_xyz_voxeled);

    // filter with voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud (cloud_xyz_voxeled);
    filter.setLeafSize (octree_resolution_, octree_resolution_, octree_resolution_);
    filter.filter (*cloud_xyz_filtered);

    // publish point cloud
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*cloud_xyz_filtered, output_msg);
    output_msg.header = input_msg->header;
    pub_cloud_.publish(output_msg);

    // publish marker
    if (publish_marker_flag_) {
      visualization_msgs::Marker marker_msg;
      marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
      marker_msg.scale.x = octree_resolution_;
      marker_msg.scale.y = octree_resolution_;
      marker_msg.scale.z = octree_resolution_;
      marker_msg.header = input_msg->header;
      marker_msg.pose.orientation.w = 1.0;
      marker_msg.color = jsk_topic_tools::colorCategory20(0);

      pcl::PointXYZ p;
      for (size_t i = 0; i < cloud_xyz_filtered->size(); i++) {
        p = cloud_xyz_filtered->at(i);
        geometry_msgs::Point point_ros;
        point_ros.x = p.x;
        point_ros.y = p.y;
        point_ros.z = p.z;
        marker_msg.points.push_back(point_ros);
        marker_msg.colors.push_back(jsk_topic_tools::colorCategory20(0));
      }
      pub_marker_.publish(marker_msg);
    }
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
    octree_resolution_ = config.octree_resolution;
    point_resolution_ = config.point_resolution;
    show_statistics_ = config.show_statistics;
  }

  void OctreeVoxelGrid::onInit(void)
  {
    DiagnosticNodelet::onInit();

    pnh_->param("publish_marker", publish_marker_flag_, true);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OctreeVoxelGrid::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_marker_ = advertise<visualization_msgs::Marker>(*pnh_, "output_marker", 1);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OctreeVoxelGrid, nodelet::Nodelet);
