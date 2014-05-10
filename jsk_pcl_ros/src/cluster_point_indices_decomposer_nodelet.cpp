// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/cluster_point_indices_decomposer.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <boost/format.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace jsk_pcl_ros
{
  ClusterPointIndicesDecomposer::ClusterPointIndicesDecomposer() {}
  ClusterPointIndicesDecomposer::~ClusterPointIndicesDecomposer() {}
  void ClusterPointIndicesDecomposer::onInit()
  {
    PCLNodelet::onInit();
    marker_num_ = 0;
    pnh_.reset (new ros::NodeHandle (getPrivateNodeHandle ()));
    marker_pub_ = pnh_->advertise<visualization_msgs::MarkerArray>("marker", 1);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_target_.subscribe(*pnh_, "target", 1);
    if (!pnh_->getParam("tf_prefix_", tf_prefix_))
    {
      ROS_WARN("~tf_prefix_ is not specified, using %s", getName().c_str());
      tf_prefix_ = getName();
    }

    //sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, jsk_pcl_ros::ClusterPointIndices> >(input_sub, target_sub, 100);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_target_);
    sync_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2));
  }
  
  void ClusterPointIndicesDecomposer::sortIndicesOrder
  (pcl::PointCloud<pcl::PointXYZ>::Ptr input,
   std::vector<pcl::IndicesPtr> indices_array,
   std::vector<pcl::IndicesPtr> &output_array)
  {
    output_array.resize(indices_array.size());
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      output_array[i] = indices_array[i];
    }
  }
  
  void ClusterPointIndicesDecomposer::extract
  (const sensor_msgs::PointCloud2ConstPtr &input,
   const jsk_pcl_ros::ClusterPointIndicesConstPtr &indices_input)
  {
    allocatePublishers(indices_input->cluster_indices.size());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::fromROSMsg(*input, *cloud_xyz);

    std::vector<pcl::IndicesPtr> converted_indices;
    std::vector<pcl::IndicesPtr> sorted_indices;
    for (size_t i = 0; i < indices_input->cluster_indices.size(); i++)
    {
      pcl::IndicesPtr vindices;
      vindices.reset (new std::vector<int> (indices_input->cluster_indices[i].indices));
      converted_indices.push_back(vindices);
    }
    
    sortIndicesOrder(cloud_xyz, converted_indices, sorted_indices);
    extract.setInputCloud(cloud);

    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < sorted_indices.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
      extract.setIndices(sorted_indices[i]);
      extract.filter(*segmented_cloud);
      sensor_msgs::PointCloud2::Ptr out_cloud(new sensor_msgs::PointCloud2);
      pcl::toROSMsg(*segmented_cloud, *out_cloud);
      out_cloud->header = input->header;
      publishers_[i].publish(out_cloud);
      // publish tf
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*segmented_cloud, center);
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
      transform.setRotation(tf::createIdentityQuaternion());
      br_.sendTransform(tf::StampedTransform(transform, input->header.stamp,
                                             input->header.frame_id,
                                             tf_prefix_ + (boost::format("output%02u") % (i)).str()));
      
      visualization_msgs::Marker marker;
      Eigen::Vector4f minpt, maxpt;
      pcl::getMinMax3D<pcl::PointXYZRGB>(*segmented_cloud, minpt, maxpt);
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.header = input->header;
      marker.id = i;
      marker.pose.position.x = center[0];
      marker.pose.position.y = center[1];
      marker.pose.position.z = center[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = std::max(fabs(minpt[0] - center[0]),
                                fabs(maxpt[0] - center[0])) * 2.0;
      marker.scale.y = std::max(fabs(minpt[1] - center[1]),
                                fabs(maxpt[1] - center[1])) * 2.0;
      marker.scale.z = std::max(fabs(minpt[2] - center[2]),
                                fabs(maxpt[2] - center[2])) * 2.0;
      marker.color.a = 0.5;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker_array.markers.push_back(marker);
    }
    
    if (marker_num_ > sorted_indices.size()) { // delete the markers
      for (size_t i = sorted_indices.size(); i < marker_num_; i++) {
        visualization_msgs::Marker marker;
        marker.header = input->header;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.id = i;
        marker_array.markers.push_back(marker);
      }
    }
    marker_pub_.publish(marker_array);
    marker_num_ = marker_array.markers.size();
  }

  void ClusterPointIndicesDecomposer::allocatePublishers(size_t num)
  {
    if (num > publishers_.size())
    {
        for (size_t i = publishers_.size(); i < num; i++)
        {
            std::string topic_name = (boost::format("output%02u") % (i)).str();
            ROS_INFO("advertising %s", topic_name.c_str());
            ros::Publisher publisher = pnh_->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
            publishers_.push_back(publisher);
        }
    }
  }
  
}

typedef jsk_pcl_ros::ClusterPointIndicesDecomposer ClusterPointIndicesDecomposer;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ClusterPointIndicesDecomposer, ClusterPointIndicesDecomposer, nodelet::Nodelet);

