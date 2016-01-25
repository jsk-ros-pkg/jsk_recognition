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


#ifndef JSK_PCL_ROS_BORDER_ESTIMATOR_H_
#define JSK_PCL_ROS_BORDER_ESTIMATOR_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_topic_tools/connection_based_nodelet.h"

#include <jsk_pcl_ros/BorderEstimatorConfig.h>
#include <dynamic_reconfigure/server.h>

namespace jsk_pcl_ros
{
  class BorderEstimator: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> SyncPolicy;
    typedef BorderEstimatorConfig Config;
  protected:
    virtual void onInit();
    virtual void estimate(const sensor_msgs::PointCloud2::ConstPtr& msg,
                          const sensor_msgs::CameraInfo::ConstPtr& caminfo);
    virtual void estimate(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void computeBorder(
      const pcl::RangeImage& image,
      const std_msgs::Header& header);
    virtual void publishCloud(ros::Publisher& pub,
                              const pcl::PointIndices& inlier,
                              const std_msgs::Header& header);
    
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_border_, pub_veil_, pub_shadow_;
    ros::Publisher pub_range_image_;
    ros::Publisher pub_cloud_;
    ros::Subscriber sub_;
    std::string model_type_;
    boost::mutex mutex_;
    double noise_level_;
    double min_range_;
    int border_size_;
    double angular_resolution_;
    double max_angle_height_;
    double max_angle_width_;

  private:
    
  };
}

#endif
