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

#include "jsk_pcl_ros/fpfh_publisher.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
namespace jsk_pcl_ros
{

  bool FPFHPublisher::check_include_normal_feature(const sensor_msgs::PointCloud2& pc)
  {
    for(int index= 0; index < pc.fields.size(); index++){
      if(pc.fields[index].name.find("normal",0) != std::string::npos){
        return false;
      }
    }
    return true;
  }

  void FPFHPublisher::calculate_fpfh(const sensor_msgs::PointCloud2& pc)
  {
    //the cloud should have the feature of normals
    if(check_include_normal_feature(pc)){
      ROS_ERROR("@%s:%d the msh doesn't have normal features", __func__, __LINE__);
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
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

    pcl::FPFHEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (cloud);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

    fpfh.setSearchMethod (tree);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    fpfh.setRadiusSearch (radius_search);
    fpfh.compute (*fpfhs);

    //publish cloud_and_normal result
    pcl::PCLPointCloud2 pclpc2;
    sensor_msgs::PointCloud2 rospc2;
    pcl::toPCLPointCloud2(*fpfhs, pclpc2);
    pcl_conversions::fromPCL(pclpc2, rospc2);
    pub_.publish(rospc2);
  }

  void FPFHPublisher::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();

    if (!pnh_->getParam("radius_search",radius_search))
      {
        ROS_WARN("~radius_search is not specified");
        radius_search = 0.01;
      }

    sub_input_ = pnh_->subscribe("input", 1, &FPFHPublisher::calculate_fpfh, this);
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
  }
}

typedef jsk_pcl_ros::FPFHPublisher FPFHPublisher;
PLUGINLIB_DECLARE_CLASS (jsk_pcl_ros, FPFHPublisher, FPFHPublisher, nodelet::Nodelet);
