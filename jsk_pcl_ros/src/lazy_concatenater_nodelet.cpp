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

#include <boost/format.hpp>

namespace jsk_pcl_ros
{

  void LazyConcatenater::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);
    // max 20
    std::vector<std::string> topics;
#define ADD_TOPIC(x) {                                                  \
      if (config.enable_input_topic##x && config.input_topic##x.length() > 0) { \
        topics.push_back(config.input_topic##x);                        \
      }                                                                 \
    }
    ADD_TOPIC(0);
    ADD_TOPIC(1);
    ADD_TOPIC(2);
    ADD_TOPIC(3);
    ADD_TOPIC(4);
    ADD_TOPIC(5);
    ADD_TOPIC(6);
    ADD_TOPIC(7);
    ADD_TOPIC(8);
    ADD_TOPIC(9);
    ADD_TOPIC(10);
    ADD_TOPIC(11);
    ADD_TOPIC(12);
    ADD_TOPIC(13);
    ADD_TOPIC(14);
    ADD_TOPIC(15);
    ADD_TOPIC(16);
    ADD_TOPIC(17);
    ADD_TOPIC(18);
    ADD_TOPIC(19);
    resetSubscribers(topics);
  }
  
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

    std::vector<ros::Subscriber> subscribers_to_be_removed;
    std::vector<ros::Subscriber> subscribers_to_be_remained;
    std::vector<std::string> subscribers_to_be_remained_names;
    ROS_INFO("subscribers_ => %lu", subscribers_.size());
    for (size_t i = 0; i < subscribers_.size(); i++) {
      std::string topic_name = subscribers_[i].getTopic();
      // check `topic_name' is included in topics or not
      if(std::find(topics.begin(), topics.end(), topic_name) == topics.end()) { // cannot find
        ROS_INFO_STREAM(topic_name << " will be removed");
        subscribers_to_be_removed.push_back(subscribers_[i]);
        
      }
      else {                    // found
        ROS_INFO_STREAM(topic_name << " will be remained");
        subscribers_to_be_remained.push_back(subscribers_[i]);
        subscribers_to_be_remained_names.push_back(topic_name);
        
      }
    }
    
    for (size_t i = 0; i < subscribers_to_be_removed.size(); i++) {
      ROS_INFO_STREAM("unsubscribing [" << subscribers_to_be_removed[i].getTopic() << "]");
      subscribers_to_be_removed[i].shutdown();
    }
    
    subscribers_ = subscribers_to_be_remained;
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
      
      if (std::find(subscribers_to_be_remained_names.begin(),
                    subscribers_to_be_remained_names.end(), topic_name) != subscribers_to_be_remained_names.end()) {
        //ROS_INFO_STREAM("skip subscribing " << topic_name);
      }
      else {
        ROS_INFO_STREAM("subscribing " << topic_name);
        ros::Subscriber sub
          = pnh_->subscribe<sensor_msgs::PointCloud2>(topic_name, 1,
                                                      boost::bind(&LazyConcatenater::subscribeCallback,
                                                                  this,
                                                                  topic_name,
                                                                  _1));
        subscribers_.push_back(sub);
      }
    }
  }
  
  void LazyConcatenater::onInit(void)
  {
    PCLNodelet::onInit();
    // read topics to subscribe from parameter server
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    // dynamic reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&LazyConcatenater::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    // concatenater reads the parmeter input_topic0, input_topic1, ..., input_topic20
    const size_t max_input = 20;
    std::vector<std::string> input_topics;
    for (size_t i = 0; i < max_input; i++) {
      std::string input_topic;
      bool enable_input_topic;
      std::string target_param = (boost::format("input_topic%d") % (i)).str();
      std::string target_enabled_param = (boost::format("enable_input_topic%d") % (i)).str();
      if (!pnh_->getParam(target_enabled_param, enable_input_topic)) { // default False
        enable_input_topic = false;
      }
      if (enable_input_topic && pnh_->getParam(target_param, input_topic)) {
        input_topics.push_back(input_topic);
      }
      else {
        ROS_INFO_STREAM(target_param << " is not specified or not enabled, ignore it");
      }
    }
    resetSubscribers(input_topics);
  }
}

typedef jsk_pcl_ros::LazyConcatenater LazyConcatenater;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, LazyConcatenater, LazyConcatenater, nodelet::Nodelet);

