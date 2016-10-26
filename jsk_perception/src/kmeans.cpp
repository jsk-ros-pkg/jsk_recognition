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


#include "jsk_perception/kmeans.h"
#include <jsk_recognition_utils/cv_utils.h>
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  void KMeans::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&KMeans::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void KMeans::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &KMeans::apply, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void KMeans::unsubscribe()
  {
    sub_.shutdown();
  }

  void KMeans::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    n_clusters_ = config.n_clusters;
  }

  void KMeans::apply(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    if ((image_msg->width == 0) && (image_msg->height == 0)) {
        ROS_WARN("invalid image input");
        return;
    }
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, image_msg->encoding);
    cv::Mat image = cv_ptr->image;

    cv::Mat reshaped_img = image.reshape(1, image.cols * image.rows);
    cv::Mat reshaped_img32f;
    reshaped_img.convertTo(reshaped_img32f, CV_32FC1, 1.0 / 255.0);
    cv::Mat labels;
    cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0);
    cv::Mat centers;
    cv::kmeans(reshaped_img32f, n_clusters_, labels, criteria, /*attempts=*/1, /*flags=*/cv::KMEANS_PP_CENTERS, centers);

    cv::Mat rgb_image(image.rows, image.cols, CV_8UC3);
    cv::MatIterator_<cv::Vec3b> rgb_first = rgb_image.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> rgb_last = rgb_image.end<cv::Vec3b>();
    cv::MatConstIterator_<int> label_first = labels.begin<int>();

    cv::Mat centers_u8;
    centers.convertTo(centers_u8, CV_8UC1, 255.0);
    cv::Mat centers_u8c3 = centers_u8.reshape(3);

    while ( rgb_first != rgb_last ) {
      const cv::Vec3b& rgb = centers_u8c3.ptr<cv::Vec3b>(*label_first)[0];
      *rgb_first = rgb;
      ++rgb_first;
      ++label_first;
    }

    pub_.publish(cv_bridge::CvImage(
                  image_msg->header,
                  image_msg->encoding,
                  rgb_image).toImageMsg());
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::KMeans, nodelet::Nodelet);
