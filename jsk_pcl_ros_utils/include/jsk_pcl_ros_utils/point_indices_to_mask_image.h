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


#ifndef JSK_PCL_ROS_UTILS_POINT_INDICES_TO_MASK_IMAGE_H_
#define JSK_PCL_ROS_UTILS_POINT_INDICES_TO_MASK_IMAGE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace jsk_pcl_ros_utils
{
  class PointIndicesToMaskImage: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    PCLIndicesMsg,
    sensor_msgs::Image > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      PCLIndicesMsg,
      sensor_msgs::Image > SyncPolicy;

    PointIndicesToMaskImage(): DiagnosticNodelet("PointIndicesToMaskImage") { }
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void convertAndPublish(
      const PCLIndicesMsg::ConstPtr& indices_msg,
      const int width,
      const int height);
    virtual void mask(
      const PCLIndicesMsg::ConstPtr& indices_msg);
    virtual void mask(
      const PCLIndicesMsg::ConstPtr& indices_msg,
      const sensor_msgs::Image::ConstPtr& image_msg);
  
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    bool approximate_sync_;
    int queue_size_;
    bool static_image_size_;
    int width_;
    int height_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >async_; 
    ros::Subscriber sub_input_static_;
    message_filters::Subscriber<PCLIndicesMsg> sub_input_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    ros::Publisher pub_;
  private:
  
  };
}

#endif
