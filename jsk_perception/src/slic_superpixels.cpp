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

#include "jsk_perception/slic_superpixels.h"
#include "slic.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
      
  void SLICSuperPixels::onInit()
  {
    ROS_WARN("Maybe this node does not work for large size images with segfault.");
    nh_ = ros::NodeHandle(getNodeHandle(), "image");
    pnh_ = getPrivateNodeHandle();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &SLICSuperPixels::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pnh_.param("publish_debug_images", debug_image_, false);

    it_.reset(new image_transport::ImageTransport(nh_));
    pub_ = pnh_.advertise<sensor_msgs::Image>("output", 1);
    if (debug_image_) {
      pub_debug_ = pnh_.advertise<sensor_msgs::Image>("debug", 1);
      pub_debug_mean_color_ = pnh_.advertise<sensor_msgs::Image>("debug/mean_color", 1);
      pub_debug_center_grid_ = pnh_.advertise<sensor_msgs::Image>("debug/center_grid", 1);
    }
    image_sub_ = it_->subscribe("", 1, &SLICSuperPixels::imageCallback, this);

    ros::V_string names = boost::assign::list_of("image");
    jsk_topic_tools::warnNoRemap(names);
  }
  
  void SLICSuperPixels::imageCallback(const sensor_msgs::Image::ConstPtr& image)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv::Mat in_image = cv_bridge::toCvShare(image, image->encoding)->image;
    cv::Mat bgr_image;
    if (in_image.channels() == 1) {
      // gray image
      cv::cvtColor(in_image, bgr_image, CV_GRAY2BGR);
    }
    else if (image->encoding == sensor_msgs::image_encodings::RGB8) {
      // convert to BGR8
      cv::cvtColor(in_image, bgr_image, CV_RGB2BGR);
    }
    else {
      bgr_image = in_image;
    }
    //cv::Mat bgr_image = cv_ptr->image;
    cv::Mat lab_image, out_image, mean_color_image, center_grid_image;
    // slic
    if (debug_image_) {
      bgr_image.copyTo(out_image);
      bgr_image.copyTo(mean_color_image);
      bgr_image.copyTo(center_grid_image);
    }
    cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);
    int w = image->width, h = image->height;
    double step = sqrt((w * h) / (double) number_of_super_pixels_);
    Slic slic;
    slic.generate_superpixels(lab_image, step, weight_);
    slic.create_connectivity(lab_image);

    if (debug_image_) {
      // creating debug image may occur seg fault.
      // So, publish_debug_images was added, in order to create debug image explicitly
      // See https://github.com/jsk-ros-pkg/jsk_recognition/pull/2181
      slic.colour_with_cluster_means(mean_color_image);
      slic.display_center_grid(center_grid_image, cv::Scalar(0, 0, 255));
      slic.display_contours(out_image, cv::Vec3b(0,0,255));

      pub_debug_.publish(cv_bridge::CvImage(
                                            image->header,
                                            sensor_msgs::image_encodings::BGR8,
                                            out_image).toImageMsg());
      pub_debug_mean_color_.publish(cv_bridge::CvImage(
                                                       image->header,
                                                       sensor_msgs::image_encodings::BGR8,
                                                       mean_color_image).toImageMsg());
      pub_debug_center_grid_.publish(cv_bridge::CvImage(
                                                        image->header,
                                                        sensor_msgs::image_encodings::BGR8,
                                                        center_grid_image).toImageMsg());
    }
    // publish clusters
    cv::Mat clusters;
    cv::transpose(slic.clusters, clusters);
    clusters = clusters + cv::Scalar(1);
    pub_.publish(cv_bridge::CvImage(
                   image->header,
                   sensor_msgs::image_encodings::TYPE_32SC1,
                   clusters).toImageMsg());
  }

  void SLICSuperPixels::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    weight_ = config.weight;
    number_of_super_pixels_ = config.number_of_super_pixels;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::SLICSuperPixels, nodelet::Nodelet);
