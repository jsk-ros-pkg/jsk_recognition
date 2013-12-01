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

#include "jsk_pcl_ros/lazy_concatenater.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  void LazyConcatenater::subscribeCallback(std::string topic_name,
                                           const sensor_msgs::PointCloud2ConstPtr &input)
  {
    ROS_INFO_STREAM("callback is called " << topic_name);
    
    // lookup to find the index
    int index = -1;
    for (size_t i = 0; i < subscribers_.size(); i++)
    {
      if (subscribers_[i].getTopic() == topic_name) {
        index = i;
        break;
      }
    }
    if (index == -1) {
      ROS_ERROR_STREAM("cannot find the topic: " << topic_name);
      return;
    }
    else {
      ROS_INFO_STREAM(topic_name << " pointcloud is updated");
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      fromROSMsg(*input, *cloud);
      pointcloud_buffer_[index] = cloud;
    }
    publishConcatenatePointCloud(input->header);
  }

  void LazyConcatenater::publishConcatenatePointCloud(const std_msgs::Header &header) {
    // create concatenate pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < pointcloud_buffer_.size(); i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr target = pointcloud_buffer_[i];
      for (size_t j = 0; j < target->points.size(); j++) {
        cloud->points.push_back(target->points[j]);
      }
    }
    sensor_msgs::PointCloud2 output_cloud;
    toROSMsg(*cloud, output_cloud);
    output_cloud.header = header;
    pub_.publish(output_cloud);
  }
    

  void LazyConcatenater::resetSubscribers(std::vector<std::string> topics)
  {
    for (size_t i = 0; i < subscribers_.size(); i++) {
      ROS_INFO_STREAM("unsubscribing [" << subscribers_[i].getTopic() << "]");
      subscribers_[i].shutdown();
    }
    
    subscribers_.resize(topics.size());
    //pointcloud_buffer_.resize();
    // clear all the pointcloud buffer
    pointcloud_buffer_.resize(topics.size());
    for (size_t i = 0; i < pointcloud_buffer_.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
      pointcloud_buffer_[i] = cloud_tmp;
    }
    
    for (size_t i = 0; i < topics.size(); i++)
    {
      std::string topic_name = topics[i];
      ROS_INFO_STREAM("subscribing " << topic_name);
      ros::Subscriber sub
        = pnh_->subscribe<sensor_msgs::PointCloud2>(topic_name, 1,
                                                    boost::bind(&LazyConcatenater::subscribeCallback,
                                                                this,
                                                                topic_name,
                                                                _1));
      subscribers_[i] = sub;
    }
  }
  
  void LazyConcatenater::onInit(void)
  {
    PCLNodelet::onInit();
    // read topics to subscribe from parameter server
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    XmlRpc::XmlRpcValue input_topics;
    if (!pnh_->getParam("input_topics", input_topics))
    {
      NODELET_ERROR ("[onInit] no ~input_topics is specified");
      return;
    }
    switch (input_topics.getType ())
    {
      case XmlRpc::XmlRpcValue::TypeArray:
      {
        std::vector<std::string> topics;
        for (int i = 0; i < input_topics.size(); i++) {
          topics.push_back((std::string)input_topics[i]);
        }
        resetSubscribers(topics);
        break;
      }
    default:
    {
      NODELET_ERROR ("[onInit] Invalid 'input_topics' parameter given!");
      return;
    }
    }
    
    
  }
}

typedef jsk_pcl_ros::LazyConcatenater LazyConcatenater;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, LazyConcatenater, LazyConcatenater, nodelet::Nodelet);

