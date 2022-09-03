// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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

#include "jsk_perception/split_image.h"
#include <cv_bridge/cv_bridge.h>


namespace jsk_perception
{
  void SplitImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("vertical_parts", vertical_parts_, 1);
    pnh_->param("horizontal_parts", horizontal_parts_, 1);
    pub_imgs_.resize(vertical_parts_ * horizontal_parts_);
    camera_info_managers_.resize(vertical_parts_ * horizontal_parts_);
    frame_ids_.resize(vertical_parts_ * horizontal_parts_);
    for (int h = 0; h < horizontal_parts_; ++h) {
      for (int v = 0; v < vertical_parts_; ++v) {
        const int index = h * vertical_parts_ + v;
        camera_info_managers_[index].reset(new camera_info_manager::CameraInfoManager(*pnh_));
        std::string camera_info_url;
        if (pnh_->getParam((boost::format("vertical%02d/horizontal%02d/camera_info_url") % v % h).str(), camera_info_url)) {
          camera_info_managers_[index]->loadCameraInfo(camera_info_url);
        }
        pnh_->getParam((boost::format("vertical%02d/horizontal%02d/frame_id") % v % h).str(), frame_ids_[index]);
        pub_imgs_[index] = advertiseCamera(*pnh_, (boost::format("output/vertical%02d/horizontal%02d") % v % h).str(), 1);
      }
    }
    onInitPostProcess();
  }

  SplitImage::~SplitImage() {
    for (size_t i = 0; i < camera_info_managers_.size(); ++i) {
      camera_info_managers_[i].reset();
    }
  }

  void SplitImage::subscribe()
  {
    pnh_->param("queue_size", queue_size_, 100);
    sub_image_ = pnh_->subscribe("input", queue_size_, &SplitImage::split, this);
  }

  void SplitImage::unsubscribe()
  {
    sub_image_.shutdown();
  }

  void SplitImage::split(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    cv::Mat image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;

    int height = image_msg->height / vertical_parts_;
    int width = image_msg->width / horizontal_parts_;
    for (int h = 0; h < horizontal_parts_; ++h) {
      int x = h * width;
      for (int v = 0; v < vertical_parts_; ++v) {
        if (pub_imgs_[h * vertical_parts_ + v].getNumSubscribers() <= 0) {
          continue;
        }
        const int index = h * vertical_parts_ + v;
        int y = v * height;
        // std::cout << x << " " << y << " " << width << " " << height << std::endl;
        cv::Mat roi = image(cv::Rect(x, y, width, height));
        sensor_msgs::CameraInfo camera_info = camera_info_managers_[index]->getCameraInfo();
        camera_info.header = image_msg->header;
        sensor_msgs::Image out_image_msg = *cv_bridge::CvImage(image_msg->header,
                                                               image_msg->encoding,
                                                               roi).toImageMsg();
        if (frame_ids_[index].length() > 0) {
          out_image_msg.header.frame_id = frame_ids_[index];
          camera_info.header.frame_id = frame_ids_[index];
        }
        pub_imgs_[index].publish(out_image_msg, camera_info);
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::SplitImage, nodelet::Nodelet);
