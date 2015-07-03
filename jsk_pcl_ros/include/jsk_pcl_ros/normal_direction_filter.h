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


#ifndef JSK_PCL_ROS_NORMAL_DIRECTION_FILTER_H_
#define JSK_PCL_ROS_NORMAL_DIRECTION_FILTER_H_

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_pcl_ros/NormalDirectionFilterConfig.h"
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"


namespace jsk_pcl_ros
{
  class NormalDirectionFilter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef NormalDirectionFilterConfig Config;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::Imu> SyncPolicy;
    NormalDirectionFilter(): DiagnosticNodelet("NormalDirectionFilter") {}
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void filter(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void filter(const sensor_msgs::PointCloud2::ConstPtr& msg,
                        const sensor_msgs::Imu::ConstPtr& imu_msg);
    virtual void filterIndices(
      const pcl::PointCloud<pcl::Normal>::Ptr& normal,
      const Eigen::Vector3f& direction,
      pcl::PointIndices& indices);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback (Config &config, uint32_t level);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    Eigen::Vector3f static_direction_;
    double eps_angle_;
    double angle_offset_;
    tf::TransformListener* tf_listener_;
    bool use_imu_;
    int queue_size_;
  private:
    
  };
}

#endif
