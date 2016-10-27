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

#include "jsk_pcl_ros/voxel_grid_downsample_manager.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl_ros/transforms.h>
#include <boost/format.hpp>
#include "jsk_recognition_msgs/SlicedPointCloud.h"

namespace jsk_pcl_ros
{

  void VoxelGridDownsampleManager::clearAll()
  {
    grid_.clear();
  }
  
  
  void VoxelGridDownsampleManager::pointCB(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    try {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if (grid_.size() == 0) {
      NODELET_DEBUG("the number of registered grids is 0, skipping");
      return;
    }
    fromROSMsg(*input, *cloud);
    for (size_t i = 0; i < grid_.size(); i++)
    {
      visualization_msgs::Marker::ConstPtr target_grid = grid_[i];
      // not yet tf is supported
      int id = target_grid->id;
      // solve tf with ros::Time 0.0

      // transform pointcloud to the frame_id of target_grid
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_ros::transformPointCloud(target_grid->header.frame_id,
                                   *cloud,
                                   *transformed_cloud,
                                   *tf_listener);
      double center_x = target_grid->pose.position.x;
      double center_y = target_grid->pose.position.y;
      double center_z = target_grid->pose.position.z;
      double range_x = target_grid->scale.x * 1.0; // for debug
      double range_y = target_grid->scale.y * 1.0;
      double range_z = target_grid->scale.z * 1.0;
      double min_x = center_x - range_x / 2.0;
      double max_x = center_x + range_x / 2.0;
      double min_y = center_y - range_y / 2.0;
      double max_y = center_y + range_y / 2.0;
      double min_z = center_z - range_z / 2.0;
      double max_z = center_z + range_z / 2.0;
      double resolution = target_grid->color.r;
      // filter order: x -> y -> z -> downsample
      pcl::PassThrough<pcl::PointXYZRGB> pass_x;
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(min_x, max_x);
      
      pcl::PassThrough<pcl::PointXYZRGB> pass_y;
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(min_y, max_y);
      pcl::PassThrough<pcl::PointXYZRGB> pass_z;
      pass_z.setFilterFieldName("z");
      pass_z.setFilterLimits(min_z, max_z);

      NODELET_DEBUG_STREAM(id << " filter x: " << min_x << " - " << max_x);
      NODELET_DEBUG_STREAM(id << " filter y: " << min_y << " - " << max_y);
      NODELET_DEBUG_STREAM(id << " filter z: " << min_z << " - " << max_z);
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_x (new pcl::PointCloud<pcl::PointXYZRGB>);
      pass_x.setInputCloud (transformed_cloud);
      pass_x.filter(*cloud_after_x);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_y (new pcl::PointCloud<pcl::PointXYZRGB>);
      pass_y.setInputCloud (cloud_after_x);
      pass_y.filter(*cloud_after_y);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_after_z (new pcl::PointCloud<pcl::PointXYZRGB>);
      pass_z.setInputCloud (cloud_after_y);
      pass_z.filter(*cloud_after_z);

      // downsample
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud (cloud_after_z);
      sor.setLeafSize (resolution, resolution, resolution);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
      sor.filter (*cloud_filtered);

      // reverse transform
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr reverse_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl_ros::transformPointCloud(input->header.frame_id,
                                   *cloud_filtered,
                                   *reverse_transformed_cloud,
                                   *tf_listener);
      
      // adding the output into *output_cloud
      // tmp <- cloud_filtered + output_cloud
      // output_cloud <- tmp
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
      //pcl::concatenatePointCloud (*cloud_filtered, *output_cloud, tmp);
      //output_cloud = tmp;
      NODELET_DEBUG_STREAM(id << " includes " << reverse_transformed_cloud->points.size() << " points");
      for (size_t i = 0; i < reverse_transformed_cloud->points.size(); i++) {
        output_cloud->points.push_back(reverse_transformed_cloud->points[i]);
      }
    }
    sensor_msgs::PointCloud2 out;
    toROSMsg(*output_cloud, out);
    out.header = input->header;
    pub_.publish(out);          // for debugging

    // for concatenater
    size_t cluster_num = output_cloud->points.size() / max_points_ + 1;
    NODELET_DEBUG_STREAM("encoding into " << cluster_num << " clusters");
    for (size_t i = 0; i < cluster_num; i++) {
      size_t start_index = max_points_ * i;
      size_t end_index = max_points_ * (i + 1) > output_cloud->points.size() ?
        output_cloud->points.size(): max_points_ * (i + 1);
      sensor_msgs::PointCloud2 cluster_out_ros;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        cluster_out_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
      cluster_out_pcl->points.resize(end_index - start_index);
      // build cluster_out_pcl
      NODELET_DEBUG_STREAM("make cluster from " << start_index << " to " << end_index);
      for (size_t j = start_index; j < end_index; j++) {
        cluster_out_pcl->points[j - start_index] = output_cloud->points[j];
      }
      // conevrt cluster_out_pcl into ros msg
      toROSMsg(*cluster_out_pcl, cluster_out_ros);
      jsk_recognition_msgs::SlicedPointCloud publish_point_cloud;
      cluster_out_ros.header = input->header;
      publish_point_cloud.point_cloud = cluster_out_ros;
      publish_point_cloud.slice_index = i;
      publish_point_cloud.sequence_id = sequence_id_;
      pub_encoded_.publish(publish_point_cloud);
      ros::Duration(1.0 / rate_).sleep();
    }
    }
    catch (std::runtime_error e) { // catch any error
      NODELET_WARN_STREAM("error has occured in VoxelGridDownsampleManager but ignore it: " << e.what());
      ros::Duration(1.0 / rate_).sleep();
    }
  }

  void VoxelGridDownsampleManager::addGrid(const visualization_msgs::Marker::ConstPtr &new_box)
  {
    ++sequence_id_;
    // check we have new_box->id in our bounding_boxes_
    if (new_box->id == -1) {
      // cancel all
      NODELET_DEBUG("clear all pointclouds");
      clearAll();
    }
    else {
      for (size_t i = 0; i < grid_.size(); i++) {
        if (grid_[i]->id == new_box->id) {
          NODELET_DEBUG_STREAM("updating " << new_box->id << " grid");
          grid_[i] = new_box;
        }
      }
      NODELET_DEBUG_STREAM("adding new grid: " << new_box->id);
      grid_.push_back(new_box);
    }
  }

  void VoxelGridDownsampleManager::initializeGrid(void) {
    visualization_msgs::Marker::Ptr box (new visualization_msgs::Marker);
    box->header.stamp = ros::Time(0.0);
    box->header.frame_id = base_frame_;
    box->pose.position.x = 2.0;
    box->pose.position.y = 0.0;
    box->pose.position.z = -0.5;
    box->scale.x = 4.0;
    box->scale.y = 2.0;
    box->scale.z = 3.0;
    box->color.r = 0.05;
    box->color.g = 0.05;
    box->color.b = 0.05;
    box->color.a = 1.0;
    grid_.push_back(box);
  }
  
  void VoxelGridDownsampleManager::onInit(void)
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("base_frame", base_frame_, std::string("pelvis"));
    tf_listener = TfListenerSingleton::getInstance();
    initializeGrid();
    sequence_id_ = 0;

    int max_points_param;
    pnh_->param("max_points", max_points_param, 300);
    pnh_->param("rate", rate_, 1.0);
    max_points_  = max_points_param;
    
    pub_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output", 1);
    pub_encoded_ = advertise<jsk_recognition_msgs::SlicedPointCloud>(
      *pnh_, "output_encoded", 1);
    
    onInitPostProcess();
  }

  void VoxelGridDownsampleManager::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &VoxelGridDownsampleManager::pointCB,
                           this);
    bounding_box_sub_ = pnh_->subscribe("add_grid", 1, &VoxelGridDownsampleManager::addGrid,
                                        this);
  }

  void VoxelGridDownsampleManager::unsubscribe()
  {
    sub_.shutdown();
    bounding_box_sub_.shutdown();
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::VoxelGridDownsampleManager,
                        nodelet::Nodelet);
