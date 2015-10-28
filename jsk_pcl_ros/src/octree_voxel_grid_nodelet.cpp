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
#include <iostream>
#include <sstream>

namespace jsk_pcl_ros
{
  void OctreeVoxelGrid::generateVoxelCloud(const sensor_msgs::PointCloud2ConstPtr& input_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::fromROSMsg(*input_msg, *cloud_xyz);

    bool show_statistics = false;
    double point_resolution = 0.1;
    double octree_resolution = 0.1;
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;

    // instantiate point cloud compression for encoding and decoding
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudEncoder(compressionProfile, show_statistics, point_resolution, octree_resolution);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> PointCloudDecoder;

    std::stringstream compressed_data;
    // compress point cloud
    PointCloudEncoder.encodePointCloud(cloud_xyz, compressed_data);
    // decompress point cloud
    PointCloudDecoder.decodePointCloud(compressed_data, cloud_xyz_filtered);

    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*cloud_xyz_filtered, output_msg);
    output_msg.header = input_msg->header;
    pub_cloud_.publish(output_msg);
  }

  void OctreeVoxelGrid::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &OctreeVoxelGrid::generateVoxelCloud, this);
  }

  void OctreeVoxelGrid::unsubscribe()
  {
    sub_input_.shutdown();
  }

  void OctreeVoxelGrid::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OctreeVoxelGrid, nodelet::Nodelet);
