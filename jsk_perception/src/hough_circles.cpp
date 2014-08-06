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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <jsk_perception/Circle2DArray.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_view2/ImageMarker2.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_perception/HoughCirclesConfig.h"

namespace jsk_perception
{
  class HoughCircleDetector: public nodelet::Nodelet
  {
  public:
    typedef jsk_perception::HoughCirclesConfig Config;
    Config config_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    ros::NodeHandle nh_, pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::Publisher circle_pub_, marker_pub_;
    image_transport::Subscriber image_sub_;
    int subscriber_count_;
    int previous_marker_num_;

    int gaussian_blur_size_;
    double gaussian_sigma_x_;
    double gaussian_sigma_y_;
    double edge_threshold_;
    double voting_threshold_;
    double dp_;
    int min_circle_radius_;
    int max_circle_radius_;

    boost::mutex mutex_;
    
    void imageCallback(const sensor_msgs::Image::ConstPtr& image)
    {
      boost::mutex::scoped_lock lock(mutex_);
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv::Mat bgr_image = cv_ptr->image;
      cv::Mat gray_image;
      cv::cvtColor(bgr_image, gray_image, CV_BGR2GRAY);
      cv::GaussianBlur(gray_image, gray_image,
                       cv::Size(gaussian_blur_size_, gaussian_blur_size_),
                       gaussian_sigma_x_, gaussian_sigma_y_);
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(gray_image, circles, CV_HOUGH_GRADIENT,
                       dp_, gray_image.rows / 4, edge_threshold_, voting_threshold_,
                       min_circle_radius_, max_circle_radius_);
      jsk_perception::Circle2DArray circles_msg;
      circles_msg.header = image->header;
      for (size_t i = 0; i < circles.size(); i++) {
        jsk_perception::Circle2D circle;
        circle.header = image->header;
        circle.radius = circles[i][2];
        circle.x = circles[i][0];
        circle.y = circles[i][1];
        circles_msg.circles.push_back(circle);
        // marker
        image_view2::ImageMarker2 marker;
        marker.header = image->header;
        marker.type = image_view2::ImageMarker2::CIRCLE;
        marker.id = i;
        marker.action = image_view2::ImageMarker2::ADD;
        marker.position.x = circles[i][0];
        marker.position.y = circles[i][1];
        marker.scale = circles[i][2];
        marker_pub_.publish(marker);
      }
      circle_pub_.publish(circles_msg);
      if (circles.size() < previous_marker_num_) {
        for (size_t i = circles.size(); i < previous_marker_num_; i++) {
          image_view2::ImageMarker2 marker;
          marker.header = image->header;
          marker.id = i;
          marker.action = image_view2::ImageMarker2::REMOVE;
          marker_pub_.publish(marker);
        }
      }
      previous_marker_num_ = circles.size();
    }

    void configCallback(Config &new_config, uint32_t level)
    {
      boost::mutex::scoped_lock lock(mutex_);
      gaussian_blur_size_ = new_config.gaussian_blur_size;
      gaussian_sigma_x_ = new_config.gaussian_sigma_x;
      gaussian_sigma_y_ = new_config.gaussian_sigma_y;
      edge_threshold_ = new_config.edge_threshold;
      voting_threshold_ = new_config.voting_threshold;
      dp_ = new_config.dp;
      min_circle_radius_ = new_config.min_circle_radius;
      max_circle_radius_ = new_config.max_circle_radius;
    }
    
    virtual void onInit()
    {
      subscriber_count_ = 0;
      previous_marker_num_ = 0;
      nh_ = ros::NodeHandle(getNodeHandle(), "image");
      pnh_ = getPrivateNodeHandle();
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&HoughCircleDetector::configCallback, this, _1, _2);
      srv_->setCallback (f);
      it_.reset(new image_transport::ImageTransport(nh_));
      circle_pub_ = pnh_.advertise<jsk_perception::Circle2DArray>("output", 1);
      marker_pub_ = pnh_.advertise<image_view2::ImageMarker2>("image_marker", 1);
      image_sub_ = it_->subscribe("", 1, &HoughCircleDetector::imageCallback, this);
    }
  };
}

#include <pluginlib/class_list_macros.h>
typedef jsk_perception::HoughCircleDetector HoughCircleDetector;
PLUGINLIB_DECLARE_CLASS (jsk_perception, HoughCircleDetector, HoughCircleDetector, nodelet::Nodelet);
