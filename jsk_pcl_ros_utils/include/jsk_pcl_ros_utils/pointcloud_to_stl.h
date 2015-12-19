// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_UTILS_POINTCLOUD_TO_STL_H_
#define JSK_PCL_ROS_UTILS_POINTCLOUD_TO_STL_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <jsk_recognition_msgs/SetPointCloud2.h>
#include <visualization_msgs/Marker.h>

// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/surface/organized_fast_mesh.h>

namespace jsk_pcl_ros_utils
{
  class PointCloudToSTL: public pcl_ros::PCLNodelet
  {
  public:
    PointCloudToSTL(){}
  protected:
    virtual void onInit();
    virtual void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    virtual void exportSTL(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr  &cloud);
    virtual bool createSTL(jsk_recognition_msgs::SetPointCloud2::Request &req,
                           jsk_recognition_msgs::SetPointCloud2::Response &res);

    ros::Publisher pub_mesh_;
    ros::Subscriber sub_input_;
    ros::ServiceServer create_stl_srv_;

    std::string frame_;
    double search_radius_;
    double mu_;
    int maximum_nearest_neighbors_;
    double maximum_surface_angle_;
    double minimum_angle_;
    double maximum_angle_;
    bool normal_consistency_;
    bool store_shadow_faces_;

    double triangle_pixel_size_;
    double max_edge_length_;
    std::string file_name_;
    std::string latest_output_path_;
    pcl::OrganizedFastMesh<pcl::PointXYZRGB> ofm;

  private:
  };
}

#endif
