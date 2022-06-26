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

#include "jsk_pcl_ros/normal_publisher_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
namespace jsk_pcl_ros
{
  void NormalPublisher::normal_estimate(double radius_search, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
  {
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*normals);
  }

  void NormalPublisher::calculate_normal(const sensor_msgs::PointCloud2& pc)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCLPointCloud2 pcl_pc;
    std::vector<int> indices;
    pcl_conversions::toPCL(pc, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *cloud);
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    if(cloud->points.size() < 1){
      ROS_INFO("%s cloud is zero", __func__);
      return;
    }

    //normal estimate
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimate(radius_search, cloud, normals);

    if(normals->points.size() < 1){
      ROS_INFO("cloud_normals is zero");
      return;
    }


    //publish normal result
    pcl::PCLPointCloud2 pclpc2_normal;
    sensor_msgs::PointCloud2 rospc2_normal;
    pcl::toPCLPointCloud2(*normals, pclpc2_normal);
    pcl_conversions::fromPCL(pclpc2_normal, rospc2_normal);
    pub_normal_.publish(rospc2_normal);


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_and_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_and_normals);
    std::vector<int> remove_nan_indices;
    pcl::removeNaNFromPointCloud(*cloud_and_normals, *cloud_and_normals, remove_nan_indices);


    //publish cloud_and_normal result
    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 rospc2;
    pcl::toPCLPointCloud2(*cloud_and_normals, pclpc2);
    pcl_conversions::fromPCL(pclpc2, rospc2);
    pub_.publish(rospc2);
  }

  void NormalPublisher::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();

    if (!pnh_->getParam("radius_search",radius_search))
      {
        ROS_WARN("~radius_search is not specified");
        radius_search = 0.01;
      }

    sub_input_ = pnh_->subscribe("input", 1, &NormalPublisher::calculate_normal, this);
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_normal_ = pnh_->advertise<sensor_msgs::PointCloud2>("output_normal_only", 1);

  }
}

typedef jsk_pcl_ros::NormalPublisher NormalPublisher;
PLUGINLIB_DECLARE_CLASS (jsk_pcl_ros, NormalPublisher, NormalPublisher, nodelet::Nodelet);
