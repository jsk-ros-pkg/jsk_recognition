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
#include "jsk_pcl_ros/extract_field_of_view.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/common/centroid.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void ExtractFieldOfView::extract(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    if (camera_info_.width > 0){
      sensor_msgs::PointCloud2 output(*input);
      if ( frame_ != input->header.frame_id){
        sensor_msgs::PointCloud2 latest_pointcloud(*input);
        if(tf_->waitForTransform(frame_, input->header.frame_id, ros::Time(0), ros::Duration(2.0) )){
          pcl_ros::transformPointCloud(frame_, latest_pointcloud, output, *tf_);
        }else{
          ROS_ERROR("Wait For Transform Fail");
          return;
        }
      }

      pcl_msgs::PointIndices ros_indices;
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromROSMsg(output, cloud);

      pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
      for (int i = 0; i < cloud.points.size(); i++){
        pcl::PointXYZRGB p = cloud.points[i];
        cv::Point2d uy_end = model_.project3dToPixel(cv::Point3d(p.x,
                                                                 p.y,
                                                                 p.z));
        if ((uy_end.x > 0) && (uy_end.y > 0)&&
            (uy_end.x < camera_info_.width) && (uy_end.y < camera_info_.height) &&
            (p.z > 0)){
          new_cloud.points.push_back(p);
          ros_indices.indices.push_back(i);
        }
      }
      sensor_msgs::PointCloud2 new_cloud_ros;
      pcl::toROSMsg(new_cloud, new_cloud_ros);
      new_cloud_ros.header.frame_id = frame_;
      new_cloud_ros.header.stamp = input->header.stamp;
      pub_point_.publish(new_cloud_ros);

      pub_camera_info_.publish(camera_info_);

      ros_indices.header.frame_id = frame_;
      ros_indices.header.stamp = input->header.stamp;
      pub_indices_.publish(ros_indices);

    }else{
      ROS_ERROR("Witdh is 0");
    }
  }

  void ExtractFieldOfView::camera_info(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    sensor_msgs::CameraInfo new_camera_info(*info_msg);
    new_camera_info.header.frame_id = frame_;
    camera_info_ = new_camera_info;
    model_.fromCameraInfo(new_camera_info);
  }

  void ExtractFieldOfView::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &ExtractFieldOfView::extract, this);
    sub_input_camera_info_ = pnh_->subscribe("input_camera_info", 1, &ExtractFieldOfView::camera_info, this);
  }

  void ExtractFieldOfView::unsubscribe()
  {
    sub_input_.shutdown();
  }

  void ExtractFieldOfView::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("frame", frame_, std::string("/head_mount_kinect_rgb_link"));

    tf_ = TfListenerSingleton::getInstance();
    pub_point_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_camera_info_ = pnh_->advertise<sensor_msgs::CameraInfo>("output_camera_info", 1);
    pub_indices_ = pnh_->advertise<pcl_msgs::PointIndices>("output_indices", 1);
    subscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ExtractFieldOfView, nodelet::Nodelet);
