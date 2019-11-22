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



#ifndef JSK_PCL_ROS_ROI_CLIPPER_H_
#define JSK_PCL_ROS_ROI_CLIPPER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
namespace jsk_pcl_ros
{
  class ROIClipper: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::CameraInfo > SyncPolicy;
    ROIClipper(): DiagnosticNodelet("ROIClipper") {}
    
  protected:
    virtual void onInit();
    virtual void clip(const sensor_msgs::Image::ConstPtr& image_msg,
                      const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void imageCallback(
      const sensor_msgs::Image::ConstPtr& image_msg);
    virtual void infoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    
    boost::mutex mutex_;
    bool not_sync_;
    bool keep_organized_;
    ros::Publisher pub_image_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_cloud_indices_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Subscriber sub_image_no_sync_;
    ros::Subscriber sub_info_no_sync_;
    ros::Subscriber sub_cloud_no_sync_;
    sensor_msgs::CameraInfo::ConstPtr latest_camera_info_;
  private:
  };
}

#endif
