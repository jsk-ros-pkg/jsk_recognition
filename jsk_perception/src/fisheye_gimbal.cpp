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

#include "jsk_perception/fisheye_gimbal.h"
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <math.h> 
#include <boost/assign.hpp>


#define PI 3.141592

namespace jsk_perception
{
  void FisheyeGimbal::onInit()
  {
    pnh_ = this->getPrivateNodeHandle();

    /* ros param */
    pnh_.param("debug",debug_, false);
    pnh_.param("calib",calib_, false);
    pnh_.param("k", k_, 300.0);
    pnh_.param("absolute_max_degree",absolute_max_degree_, 110.0);
    pnh_.param("odom_topic_name", odom_topic_name_, std::string("odom"));
    pnh_.param("image_topic_name", image_topic_name_, std::string("camera/image_raw"));

    /* ros pub and sub */
    pub_undistorted_image_ = pnh_.advertise<sensor_msgs::Image>("output", 1);
    if(debug_)
      pub_undistorted_center_image_ = pnh_.advertise<sensor_msgs::Image>("debug", 1);
    sub_image_.subscribe(pnh_, image_topic_name_, 1, ros::TransportHints().tcpNoDelay());
    sub_odom_.subscribe(pnh_, odom_topic_name_, 1, ros::TransportHints().tcpNoDelay());

    sync_ = boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10)));
    sync_->connectInput(sub_image_, sub_odom_);
    sync_->registerCallback(&FisheyeGimbal::rectifycallback, this);

    /* ros service */
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FisheyeGimbal::configCallback, this, _1, _2);
    srv_->setCallback (f);

    /* base variables init */
    absolute_max_radian_ = absolute_max_degree_ * M_PI /180.0;
    scale_ = 0.5;
    roll_ = 0;
    pitch_ = 0;
    gimbal_ = false;

  }

  void FisheyeGimbal::configCallback(Config &new_config, uint32_t level)
  {
    max_degree_ = new_config.degree;
    scale_ = new_config.scale;
    if(!gimbal_)
      {
        roll_ = new_config.roll;
        pitch_ = new_config.pitch;
      }
    if(calib_)
      {
        circle_x_ = new_config.circle_x;
        circle_y_ = new_config.circle_y;
        circle_r_ = new_config.circle_r;
      }
  }

  void FisheyeGimbal::rectifycallback(const sensor_msgs::Image::ConstPtr& image_msg, const nav_msgs::OdometryConstPtr& odom_msg)
  {
    cv::Mat distorted = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;
    gimbal_ = true;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    gimbal_orientation_.setRotation(q);
    double yaw = 0;
    gimbal_orientation_.getRPY(roll_, pitch_, yaw);
    tf::Matrix3x3 basis1, basis2, basis;
    basis1.setRPY(pitch_, 0 , 0);
    basis2.setRPY(0, -roll_, 0);
    basis_ = (basis1 * basis2).transpose();

    //setBasis(basis);

    if(!calib_)
      {
        int l = distorted.rows / 2;
        float max_degree = max_degree_;
        float max_radian = max_degree * M_PI /180.0;
        float tan_max_radian = tan(max_radian);

        float max_radius = max_radian * k_;

        cv::Mat undistorted(int(l * tan_max_radian * 2 * scale_), int(l * tan_max_radian * 2 * scale_), distorted.depth());
        cv::Mat center_undistorted(int(l * tan_max_radian * 2 * scale_), int(l * tan_max_radian * 2 * scale_), distorted.depth());

        int center_x = distorted.cols/2, center_y = distorted.rows/2;
        int un_center_x = undistorted.cols/2, un_center_y = undistorted.rows/2;

        if(!gimbal_)  basis_.setRPY(roll_, pitch_, 0);
        //tf::Matrix3x3 basis = getBasis();

        for(int i = 0; i < undistorted.cols; ++i){
          for(int j = 0; j < undistorted.rows; ++j){

            tf::Vector3 p = basis_ * tf::Vector3((i - un_center_x) / scale_, (j - un_center_y) /scale_, l);

            float radius = sqrt(pow(p.x(), 2) + pow(p.y(), 2));
            float radian = atan2(radius, p.z());
            if( radian < absolute_max_radian_ ){
              float multi = 0, new_x = center_x, new_y = center_y;
              if(radius){
                multi = radian * k_ / radius;
                new_x += (p.x() * multi);
                new_y += (p.y() * multi);
              }

              for(int c = 0; c < undistorted.channels(); ++c)
                undistorted.data[  j * undistorted.step + i * undistorted.elemSize() + c ]
                  = distorted.data[ int(new_y) * distorted.step + int(new_x) * distorted.elemSize() + c];
            }
            else
              {
                for(int c = 0; c < undistorted.channels(); ++c)
                  undistorted.data[  j * undistorted.step + i * undistorted.elemSize() + c ] = 255;
              }

            if(debug_)
              {
                p = tf::Vector3((i - un_center_x) / scale_, (j - un_center_y) /scale_, l);

                radius = sqrt(pow(p.x(), 2) + pow(p.y(), 2));
                radian = atan2(radius, p.z());
                if( radian < absolute_max_radian_ )
                  {
                    float multi = 0, new_x = center_x, new_y = center_y;
                    if(radius){
                      multi = radian * k_ / radius;
                      new_x += (p.x() * multi);
                      new_y += (p.y() * multi);
                    }

                    for(int c = 0; c < undistorted.channels(); ++c)
                      center_undistorted.data[  j * undistorted.step + i * undistorted.elemSize() + c ]
                        = distorted.data[ int(new_y) * distorted.step + int(new_x) * distorted.elemSize() + c];
                  }
                else
                  {
                    for(int c = 0; c < undistorted.channels(); ++c)
                      undistorted.data[  j * undistorted.step + i * undistorted.elemSize() + c ] = 255;
                  }
              }
          }
        }

        pub_undistorted_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                          image_msg->encoding,
                                                          undistorted).toImageMsg());
        if(debug_)
          pub_undistorted_center_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                                   image_msg->encoding,
                                                                   center_undistorted).toImageMsg());
      }
    else
      {
        cv::circle(distorted, cv::Point(circle_x_, circle_y_), circle_r_, CV_RGB(255,255,255), 10);
        pub_undistorted_image_.publish(cv_bridge::CvImage(image_msg->header, image_msg->encoding, distorted).toImageMsg());
      }
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::FisheyeGimbal, nodelet::Nodelet);
