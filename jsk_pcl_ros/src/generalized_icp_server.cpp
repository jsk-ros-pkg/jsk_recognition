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

#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include "jsk_pcl_ros/ICPAlign.h"
#include "jsk_pcl_ros/ICPAlignReturn.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/pyramid_feature_matching.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>

std::string common_frame_id_;


bool ICPCallback(jsk_pcl_ros::ICPAlign::Request &req,
                 jsk_pcl_ros::ICPAlign::Response &res) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(req.reference_cloud, *cloud_in);
  pcl::fromROSMsg(req.target_cloud, *cloud_out);

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(cloud_in);
  gicp.setInputTarget(cloud_out);
  gicp.setMaximumIterations (50);
  gicp.setTransformationEpsilon (1e-8);
  pcl::PointCloud<pcl::PointXYZ> Final;
  gicp.align(Final);
  std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
    gicp.getFitnessScore() << std::endl;
  std::cout << gicp.getFinalTransformation() << std::endl;
  return true;
};

bool ICPCloudCallback(jsk_pcl_ros::ICPAlignReturn::Request &req,
                 jsk_pcl_ros::ICPAlignReturn::Response &res) {
  static tf::TransformListener tf_listener_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

  if(common_frame_id_ != req.reference_cloud.header.frame_id){
    sensor_msgs::PointCloud2 output;
    bool success = false;
    while (!success) {
      try {
        tf_listener_.waitForTransform(common_frame_id_, req.reference_cloud.header.frame_id, ros::Time::now(), ros::Duration(3.0));
        pcl_ros::transformPointCloud(common_frame_id_, req.reference_cloud, output, tf_listener_);
        //        listener.lookupTransform(target_frame, source_frame, time, st);
        success = true;
      }catch(tf::ExtrapolationException e){
        ROS_INFO("transform error");
      }
    }
    pcl::fromROSMsg(output, *cloud_in);
  }else{
    pcl::fromROSMsg(req.reference_cloud, *cloud_in);
  }

  if(common_frame_id_ != req.target_cloud.header.frame_id){
    sensor_msgs::PointCloud2 output;
   bool success = false;
    while (!success) {
      try {
        tf_listener_.waitForTransform(common_frame_id_, req.target_cloud.header.frame_id, ros::Time::now(), ros::Duration(3.0));
        pcl_ros::transformPointCloud(common_frame_id_, req.target_cloud, output, tf_listener_);
        //        listener.lookupTransform(target_frame, source_frame, time, st);
        success = true;
      }catch(tf::ExtrapolationException e){
        ROS_INFO("transform error");
      }
    }

    pcl::fromROSMsg(output, *cloud_out);
  }else{
    pcl::fromROSMsg(req.target_cloud, *cloud_out);
  }


  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
  gicp.setInputSource(cloud_in);
  gicp.setInputTarget(cloud_out);
  gicp.setMaximumIterations (50);
  gicp.setTransformationEpsilon (1e-8);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGB>);
  gicp.align(*Final);
  std::cout << "has converged:" << gicp.hasConverged() << " score: " <<
    gicp.getFitnessScore() << std::endl;
  std::cout << gicp.getFinalTransformation() << std::endl;

  for(int i = 0; i < cloud_in->points.size(); i++){
    pcl::PointXYZRGB point = cloud_in->points[i];
    point.z *= -1;
    cloud_in->points[i] = point;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*cloud_in, *result, gicp.getFinalTransformation());

  //  Eigen::Matrix<Scalar, 4, 4> trans = gicp.getresultTransformation();
  // for(int i = 0; i < result->points.size(); i++){
  //   pcl::PointXYZRGB point = result->points[i];
  //   point.x *= -1;
  //   result->points[i] = point;
  // }
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*result, centroid);

  // sensor_msgs::PointCloud2 result_pc;
  // pcl::toROSMsg(*result, result_pc);
  // res.result_cloud = result_pc;
  sensor_msgs::PointCloud2 result_pc;
  pcl::toROSMsg(*result, result_pc);
  res.result_cloud = result_pc;


  res.center.x = centroid[0];
  res.center.y = centroid[1];
  res.center.z = centroid[2];

  Eigen::Matrix<float, 4, 4> matrix = gicp.getFinalTransformation();
  res.matrix.push_back(matrix(0,0));
  res.matrix.push_back(matrix(0,1));
  res.matrix.push_back(matrix(0,2));
  res.matrix.push_back(matrix(0,3));

  res.matrix.push_back(matrix(1,0));
  res.matrix.push_back(matrix(1,1));
  res.matrix.push_back(matrix(1,2));
  res.matrix.push_back(matrix(1,3));

  res.matrix.push_back(matrix(2,0));
  res.matrix.push_back(matrix(2,1));
  res.matrix.push_back(matrix(2,2));
  res.matrix.push_back(matrix(2,3));

  res.matrix.push_back(matrix(3,0));
  res.matrix.push_back(matrix(3,1));
  res.matrix.push_back(matrix(3,2));
  res.matrix.push_back(matrix(3,3));


  res.result_cloud.header.stamp = ros::Time();
  //  res.result_cloud.header.frame_id = req.reference_cloud.header.frame_id;
  return true;
};


int
main (int argc, char** argv)
{
  ros::init(argc, argv, "gicp_server");
  ros::NodeHandle n("~");

  if (!n.getParam("common_frame_id", common_frame_id_))
    {
      ROS_WARN("~common_frame_id is not specified");
      common_frame_id_ = "openni_rgb_optical_frame";
    }

  ros::ServiceServer service = n.advertiseService("align", ICPCallback);
  ros::ServiceServer service2 = n.advertiseService("cloud_align", ICPCloudCallback);
  ros::spin();
  return (0);
}
