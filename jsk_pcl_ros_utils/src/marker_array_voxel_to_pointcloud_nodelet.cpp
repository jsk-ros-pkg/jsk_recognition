/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <pcl_conversions/pcl_conversions.h>
#include "jsk_pcl_ros_utils/marker_array_voxel_to_pointcloud.h"

namespace jsk_pcl_ros_utils
{

  void MarkerArrayVoxelToPointCloud::onInit()
  {
    DiagnosticNodelet::onInit();

    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void MarkerArrayVoxelToPointCloud::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &MarkerArrayVoxelToPointCloud::convert, this);
  }

  void MarkerArrayVoxelToPointCloud::unsubscribe()
  {
    sub_.shutdown();
  }

  void MarkerArrayVoxelToPointCloud::convert(
    const visualization_msgs::MarkerArray::ConstPtr& marker_array_msg)
  {
    vital_checker_->poke();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->is_dense = true;
    for (size_t i=0; i<marker_array_msg->markers.size(); i++) {
      visualization_msgs::Marker marker = marker_array_msg->markers[i];
      for (size_t j=0; j<marker.points.size(); j++) {
        pcl::PointXYZRGB pt;
        pt.x = marker.points[j].x;
        pt.y = marker.points[j].y;
        pt.z = marker.points[j].z;
        pt.r = marker.color.r;
        pt.g = marker.color.g;
        pt.b = marker.color.b;
        cloud->points.push_back(pt);
      }
    }

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = marker_array_msg->markers[0].header;
    pub_.publish(ros_cloud);
  }

}  // namespace jsk_pcl_ros_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::MarkerArrayVoxelToPointCloud, nodelet::Nodelet);
