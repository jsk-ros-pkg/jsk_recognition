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


#ifndef JSK_PCL_ROS_UTILS_MASK_IMAGE_TO_DEPTH_CONSIDERED_MASK_IMAGE_H_
#define JSK_PCL_ROS_UTILS_MASK_IMAGE_TO_DEPTH_CONSIDERED_MASK_IMAGE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_pcl_ros_utils/MaskImageToDepthConsideredMaskImageConfig.h>

namespace jsk_pcl_ros_utils
{
  class MaskImageToDepthConsideredMaskImage: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::Image > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::Image > SyncPolicy;
    typedef jsk_pcl_ros_utils::MaskImageToDepthConsideredMaskImageConfig Config;

    MaskImageToDepthConsideredMaskImage(): DiagnosticNodelet("MaskImageToDepthConsideredMaskImage") { }
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void extractmask
    (
     const sensor_msgs::PointCloud2::ConstPtr& point_cloud2_msg,
     const sensor_msgs::Image::ConstPtr& image_msg);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void mask_region_callback(const sensor_msgs::Image::ConstPtr& msg);
  
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    bool approximate_sync_;
    int queue_size_;
    int extract_num_;
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >async_; 
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    ros::Publisher pub_;
    ros::Publisher applypub_;
    ros::Subscriber sub_;
    int region_width_;
    int region_height_;
    int region_x_off_;
    int region_y_off_;
    double region_width_ratio_;
    double region_height_ratio_;
    double region_x_off_ratio_;
    double region_y_off_ratio_;
    bool use_region_ratio_;
    bool use_mask_region_;
    bool in_the_order_of_depth_;

  private:
  
  };
}

#endif
