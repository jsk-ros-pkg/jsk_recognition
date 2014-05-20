// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto_Inagaki and JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "jsk_pcl_ros/normal_marker_array_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
namespace jsk_pcl_ros
{
  void NormalMarkerArray::configure_marker_array(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& normals,visualization_msgs::MarkerArray& marker_array)
  {
    for(int i = 0; i < normals->points.size();i++){
      if(i % skip_times_ == 0 ){
        pcl::PointXYZRGBNormal cloud_normal = normals->points[i];
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = name_space_;
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point start_point;
        start_point.x = cloud_normal.x;
        start_point.y = cloud_normal.y;
        start_point.z = cloud_normal.z;

        //add Start point
        marker.points.push_back(start_point);

        geometry_msgs::Point end_point;
        end_point.x = cloud_normal.x + cloud_normal.normal_x/length_rate_;
        end_point.y = cloud_normal.y + cloud_normal.normal_y/length_rate_;
        end_point.z = cloud_normal.z + cloud_normal.normal_z/length_rate_;

        //add Start point
        marker.points.push_back(end_point);

        marker.scale.x = 0.03/(length_rate_/2);
        marker.scale.y = 0.07/(length_rate_/2);
        //          marker.scale.z = 0.01;


        Eigen::Vector3f cloud_normal_vec(cloud_normal.normal_x, cloud_normal.normal_y, cloud_normal.normal_z);
        cloud_normal_vec = cloud_normal_vec.normalized();

        marker.color.a = 1.0;
        //1 /2 is for setting value 0 ~ 1
        marker.color.r = (cloud_normal_vec.dot(Eigen::Vector3f(1,0,0))+ 1) / 2;
        marker.color.g = (cloud_normal_vec.dot(Eigen::Vector3f(0,1,0)) + 1) / 2 ;
        marker.color.b = (cloud_normal_vec.dot(Eigen::Vector3f(0,0,1)) + 1) / 2;

        marker.lifetime = ros::Duration(1.0);
        marker_array.markers.push_back( marker );
      }
    }
  }


  bool NormalMarkerArray::check_include_normal_feature(const sensor_msgs::PointCloud2& pc)
  {
    for(int index= 0; index < pc.fields.size(); index++){
      if(pc.fields[index].name.find("normal",0) != std::string::npos){
        return false;
      }
    }
    return true;
  }

  void NormalMarkerArray::convert_normal_to_marker_array(const sensor_msgs::PointCloud2& pc)
  {
    //the cloud should have the feature of normals
    if(check_include_normal_feature(pc)){
      ROS_ERROR("@%s:%d the msh doesn't have normal features", __func__, __LINE__);
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PCLPointCloud2 pcl_pc;
    std::vector<int> indices;
    pcl_conversions::toPCL(pc, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *normals);

    frame_id_ = pc.header.frame_id;

    if(normals->points.size() < 1){
      ROS_INFO("@%s:%d normals is zero", __func__, __LINE__);
      return;
    }

    visualization_msgs::MarkerArray marker_array;
    configure_marker_array(normals, marker_array);
    pub_.publish(marker_array);
  }

  void NormalMarkerArray::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();

    if (!pnh_->getParam("skip_times",skip_times_))
      {
        ROS_WARN("~skip_times is not specified");
        skip_times_ = 100;
      }

    if (!pnh_->getParam("name_space",name_space_))
      {
        ROS_WARN("~name_space is not specified");
        name_space_ = "normal";
      }

    if (!pnh_->getParam("length_rate",length_rate_))
      {
        ROS_WARN("~length_rate is not specified");
        length_rate_ = 5.0;
      }

    sub_ = pnh_->subscribe("input", 1, &NormalMarkerArray::convert_normal_to_marker_array, this);
    pub_ = pnh_->advertise<visualization_msgs::MarkerArray>("normal_marker_array", 1);

  }
}

typedef jsk_pcl_ros::NormalMarkerArray NormalMarkerArray;
PLUGINLIB_DECLARE_CLASS (jsk_pcl_ros, NormalMarkerArray, NormalMarkerArray, nodelet::Nodelet);
