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

#ifndef JSK_PCL_ROS_UTILS_DELAY_POINTCLOUD_H_
#define JSK_PCL_ROS_UTILS_DELAY_POINTCLOUD_H_

#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud.h>
#include "jsk_topic_tools/connection_based_nodelet.h"

#include "jsk_pcl_ros_utils/DelayPointCloudConfig.h"
#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>

namespace jsk_pcl_ros_utils
{

  class DelayPointCloud: public jsk_topic_tools::ConnectionBasedNodelet
  {
    
  public:

    typedef jsk_pcl_ros_utils::DelayPointCloudConfig Config;

  protected:
    virtual void onInit();
    virtual void delay(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void subscribe();
    virtual void unsubscribe();

    boost::mutex mutex_;
    double delay_time_;
    int queue_size_;
    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
    boost::shared_ptr<message_filters::TimeSequencer<sensor_msgs::PointCloud2> > time_sequencer_;

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    void configCallback (Config &config, uint32_t level);

  private:

  };

}


#endif

