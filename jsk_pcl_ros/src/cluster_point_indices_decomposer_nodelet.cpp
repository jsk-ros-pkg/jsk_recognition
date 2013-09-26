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

#include <boost/format.hpp>
namespace jsk_pcl_ros
{
  ClusterPointIndicesDecomposer::ClusterPointIndicesDecomposer() {}
  ClusterPointIndicesDecomposer::~ClusterPointIndicesDecomposer() {}
  void ClusterPointIndicesDecomposer::onInit()
  {
    PCLNodelet::onInit();
    pnh_.reset (new ros::NodeHandle (getPrivateNodeHandle ()));
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_target_.subscribe(*pnh_, "target", 1);
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
    }
    
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

