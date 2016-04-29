// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros/pointcloud_database_server.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <pcl/io/pcd_io.h>

namespace jsk_pcl_ros
{

  PointCloudData::PointCloudData(const std::string fname):
    file_name_(fname)
  {
    pcl::PCDReader reader;
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    reader.read(file_name_, *cloud_);
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudData::getPointCloud()
  {
  return cloud_;
  }

  sensor_msgs::PointCloud2
  PointCloudData::getROSPointCloud(ros::Time stamp)
  {
    sensor_msgs::PointCloud2 ros_out;
    ros_out.header.stamp = stamp;
    ros_out.header.frame_id = file_name_;
    pcl::toROSMsg(*cloud_, ros_out);
    return ros_out;
  }

  PointcloudDatabaseServer::~PointcloudDatabaseServer()
  {
    timer_.stop();
  }

  void PointcloudDatabaseServer::onInit()
  {
    PCLNodelet::onInit();
    std::vector<std::string> pcd_files;
    pub_points_ = pnh_->advertise<jsk_recognition_msgs::PointsArray>("output", 1);
    if (!jsk_topic_tools::readVectorParameter(*pnh_, "models", pcd_files)
        || pcd_files.size() == 0) {
      JSK_NODELET_FATAL("no models is specified");
      return;
    }

    for (size_t i = 0; i< pcd_files.size(); i++) {
      PointCloudData::Ptr data(new PointCloudData(pcd_files[i]));
      point_clouds_.push_back(data);
    }
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(
        &PointcloudDatabaseServer::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->getParam("duration", duration_);
  }

  void PointcloudDatabaseServer::timerCallback(const ros::TimerEvent& event)
  {
    jsk_recognition_msgs::PointsArray ros_out;
    ros_out.header.stamp = event.current_real;
    for (size_t i = 0; i < point_clouds_.size(); i++) {
      ros_out.cloud_list.push_back(point_clouds_[i]->getROSPointCloud(event.current_real));
                                                                      
    }
    pub_points_.publish(ros_out);
  }

  void PointcloudDatabaseServer::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    duration_ = config.duration;
    if (timer_) {
      timer_.stop();
    }
    timer_ = pnh_->createTimer(ros::Duration(duration_),
                               boost::bind(
                                           &PointcloudDatabaseServer::timerCallback,
                                           this,
                                           _1));
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointcloudDatabaseServer,
                        nodelet::Nodelet);
