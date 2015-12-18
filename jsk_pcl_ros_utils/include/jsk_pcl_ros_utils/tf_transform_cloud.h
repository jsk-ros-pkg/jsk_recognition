// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
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

#ifndef JSK_PCL_ROS_UTILS_TF_TRANSFORMCLOUD_H_
#define JSK_PCL_ROS_UTILS_TF_TRANSFORMCLOUD_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>

// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_recognition_utils/tf_listener_singleton.h"

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace jsk_pcl_ros_utils
{
  class TfTransformCloud: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    TfTransformCloud(): DiagnosticNodelet("TfTransformCloud") {}
  protected:
    ros::Subscriber sub_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_message_filters_;
    ros::Publisher  pub_cloud_;
    std::string target_frame_id_;
    tf::TransformListener* tf_listener_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > tf_filter_;
    virtual void transform(const sensor_msgs::PointCloud2ConstPtr &input);
    virtual void subscribe();
    virtual void unsubscribe();
    
    double duration_;
    bool use_latest_tf_;
    int tf_queue_size_;
  private:
    virtual void onInit();
  };
}

#endif
