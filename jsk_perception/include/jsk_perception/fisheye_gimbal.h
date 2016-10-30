// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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


#ifndef JSK_PERCEPTION_FISHEYE_GIMBAL_H_
#define JSK_PERCEPTION_FISHEYE_GIMBAL_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/pass_through.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/FisheyeConfig.h>
#include <tf/transform_listener.h> /* for vector3 */
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <boost/version.hpp>
#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
 #include <boost/thread/lock_guard.hpp>
#endif

namespace jsk_perception
{
  class FisheyeGimbal: public nodelet::Nodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicy;
    typedef jsk_perception::FisheyeConfig Config;

    FisheyeGimbal(){}
  protected:
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    void configCallback(Config &new_config, uint32_t level);
    virtual void onInit();
    void rectifycallback(const sensor_msgs::ImageConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg);

    boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;

    ros::Publisher pub_undistorted_image_;
    ros::Publisher pub_undistorted_center_image_;
    std::string odom_topic_name_, image_topic_name_;
    float max_degree_;
    float scale_;
    double offset_degree_;
    double  k_;
    double roll_,pitch_;
    float circle_x_, circle_y_, circle_r_;
    bool gimbal_;
    tf::Matrix3x3 gimbal_orientation_;
    tf::Matrix3x3 basis_;

    double absolute_max_degree_, absolute_max_radian_;

    boost::mutex param_mutex_;

    bool calib_;
    bool debug_;

  private:
    ros::NodeHandle pnh_;
    void setBasis(tf::Matrix3x3 basis)
    {
      boost::lock_guard<boost::mutex> lock(param_mutex_);
      basis_ = basis;
    }

    const tf::Matrix3x3 getBasis()
    {
      boost::lock_guard<boost::mutex> lock(param_mutex_);
      return basis_;
    }
  };
}

#endif
