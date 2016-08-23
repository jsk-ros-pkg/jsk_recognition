// -*- mode: C++ -*-
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

#include "jsk_pcl_ros_utils/delay_pointcloud.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros_utils
{
  void DelayPointCloud::onInit()
  {
    ConnectionBasedNodelet::onInit();

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&DelayPointCloud::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pnh_->param("delay_time", delay_time_, 0.1);
    pnh_->param("queue_size", queue_size_, 1000);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", queue_size_);
  }

  void DelayPointCloud::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    delay_time_ = config.delay_time;
    DelayPointCloud::subscribe();
  }

  void DelayPointCloud::delay(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    sensor_msgs::PointCloud2 out_cloud_msg = *msg;
    out_cloud_msg.header.stamp = ros::Time::now();
    pub_.publish(out_cloud_msg);
  }

  void DelayPointCloud::subscribe()
  {
    sub_.subscribe(*pnh_, "input", 1);
    time_sequencer_ = boost::make_shared<message_filters::TimeSequencer<sensor_msgs::PointCloud2> >(ros::Duration(delay_time_), ros::Duration(0.01), queue_size_);
    time_sequencer_->connectInput(sub_);
    time_sequencer_->registerCallback(boost::bind(&DelayPointCloud::delay, this, _1));
  }
  void DelayPointCloud::unsubscribe()
  {
    sub_.unsubscribe();
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::DelayPointCloud, nodelet::Nodelet);
