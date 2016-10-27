// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include "jsk_perception/bing.h"
#include <ros/ros.h>
#include <rospack/rospack.h>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>
#include <sensor_msgs/image_encodings.h>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <vector>
#include <algorithm>

namespace jsk_perception
{
  void Bing::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_rects_ = advertise<jsk_recognition_msgs::RectArray>(*pnh_, "output", 1);
    pub_objectness_ = advertise<sensor_msgs::Image>(*pnh_, "output/objectness", 1);
    // find trained data
    std::string training_path;
#ifdef ROSPACK_EXPORT
    rospack::ROSPack rp;
    rospack::Package *p = rp.get_pkg("jsk_perception");
    if (p == NULL) {
      ROS_ERROR("Package path of 'jsk_perception' does not found");
      exit(1);
    } else {
       training_path = p->path + std::string("/trained_data/ObjectnessTrainedModel");
    }
#else
    rospack::Rospack rp;
    ros::V_string search_path;
    rp.getSearchPathFromEnv(search_path);
    rp.crawl(search_path, 1);
    std::string path;
    if (rp.find("jsk_perception", path) == true) {
      training_path = path + std::string("/trained_data/ObjectnessTrainedModel");
    } else {
      ROS_ERROR("Package path of 'jsk_perception' does not found");
      exit(1);
    }
#endif
    if (!boost::filesystem::exists(training_path)) {
      ROS_ERROR("Training data path '%s' does not exist", training_path.c_str());
      exit(1);
    }
    // setup bing
    binger_ = new cv::saliency::ObjectnessBING();
    binger_->setTrainingPath(training_path);
    onInitPostProcess();
  }

  void Bing::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &Bing::apply, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void Bing::unsubscribe()
  {
    sub_.shutdown();
  }

  void Bing::apply(
    const sensor_msgs::Image::ConstPtr& img_msg)
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      img_msg, img_msg->encoding);
    cv::Mat img = cv_ptr->image;

    // Resize too large image for fast processing
    double scale = 1.0;
    if (img.rows * img.cols > 250000) {
      scale = static_cast<double>(std::min(500. / img.rows, 500. / img.cols));
      cv::resize(img, img, cv::Size(static_cast<int>(scale * img.cols),
                                    static_cast<int>(scale * img.rows)));
    }

    std::vector<cv::Vec4i> saliency_map;
    binger_->computeSaliency(img, saliency_map);
    std::vector<float> objectness_values = binger_->getobjectnessValues();

    jsk_recognition_msgs::RectArray rects_msg;
    cv::Mat objectness_img = cv::Mat(img.rows, img.cols, CV_32FC1);
    for (size_t k=0; k < saliency_map.size(); k++) {
      int min_x = static_cast<int>(saliency_map[k][0] / scale);
      int min_y = static_cast<int>(saliency_map[k][1] / scale);
      int max_x = static_cast<int>(saliency_map[k][2] / scale);
      int max_y = static_cast<int>(saliency_map[k][3] / scale);
      // set a proposal
      jsk_recognition_msgs::Rect rect;
      rect.x = min_x;
      rect.y = min_y;
      rect.width = max_x - min_x;
      rect.height = max_y - min_y;
      rects_msg.rects.push_back(rect);
      // set objectness
      for (size_t j=std::max(0, min_y); j < std::min(max_y, img.rows); j++) {
        for (size_t i=std::max(0, min_x); i < std::min(max_x, img.cols); i++) {
          objectness_img.at<float>(j, i) += objectness_values[k];
        }
      }
    }
    // publish proposals
    rects_msg.header = img_msg->header;
    pub_rects_.publish(rects_msg);
    // publish objectness
    pub_objectness_.publish(
      cv_bridge::CvImage(
        img_msg->header,
        sensor_msgs::image_encodings::TYPE_32FC1,
        objectness_img).toImageMsg());
  }

}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::Bing, nodelet::Nodelet);
