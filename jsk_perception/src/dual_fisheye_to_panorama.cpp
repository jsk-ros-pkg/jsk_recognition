// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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

#include "jsk_perception/dual_fisheye_to_panorama.h"
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/PanoramaInfo.h>
#include <algorithm>
#include <math.h>
#include <boost/assign.hpp>

// #define PI 3.141592

namespace jsk_perception
{
  void DualFisheyeToPanorama::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("light_compen", enb_lc_, false);
    pnh_->param("refine_align", enb_ra_, false);
    pnh_->param("fovd", fovd_, 195.0f);
    pnh_->param("save_unwarped", save_unwarped_, false);
    pnh_->param("mls_map_path", mls_map_path_, std::string(""));
    ROS_INFO("light_compen : %s", enb_lc_?"true":"false");
    ROS_INFO("refine_align : %s", enb_ra_?"true":"false");
    ROS_INFO("fovd         : %7.3f", fovd_);
    ROS_INFO("save_unwarped: %7.3f", save_unwarped_?"true":"false");
    ROS_INFO("mls_map_path : %s", mls_map_path_.c_str());
    pub_panorama_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pub_panorama_info_ = advertise<jsk_recognition_msgs::PanoramaInfo>(*pnh_, "panorama_info", 1);

    sticher_initialized_ = false;
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&DualFisheyeToPanorama::configCallback, this, _1, _2);
    srv_->setCallback (f);

    msg_panorama_info_.projection_model = "equirectangular";
    msg_panorama_info_.theta_min = 0;
    msg_panorama_info_.theta_max = M_PI;
    msg_panorama_info_.phi_min = -M_PI;
    msg_panorama_info_.phi_max = M_PI;

    onInitPostProcess();
  }

  void DualFisheyeToPanorama::configCallback(Config &new_config, uint32_t level)
  {
  }


  void DualFisheyeToPanorama::subscribe()
  {
    sub_image_ = pnh_->subscribe("input", 1, &DualFisheyeToPanorama::rectify, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void DualFisheyeToPanorama::unsubscribe()
  {
    sub_image_.shutdown();
  }


  void DualFisheyeToPanorama::rectify(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    cv::Mat img;
    cv::resize(cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image, img, cv::Size(3840,1920));

    if ( ! sticher_initialized_ )  {
      ROS_INFO("initialize stitcher w:%d h:%d", img.size().width, img.size().height);
      stitcher_.reset(new stitcher::FisheyeStitcher(img.size().width,
                                                    img.size().height,
                                                    fovd_,
                                                    enb_lc_,
                                                    enb_ra_,
                                                    save_unwarped_,
                                                    mls_map_path_));
      sticher_initialized_ = true;
    }
    cv::Mat img_l, img_r;
    // Left fisheye
    img_l = img(cv::Rect(0, 0,
                         static_cast<int>(img.size().width / 2), img.size().height));
    // Right fisheye
    img_r = img(cv::Rect(static_cast<int>(img.size().width / 2), 0,
                         static_cast<int>(img.size().width / 2), img.size().height));
    // Stitch video frames
    cv::Mat pano;
    pano = stitcher_->stitch(img_l, img_r);

    pub_panorama_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                   image_msg->encoding,
                                                   pano).toImageMsg());
    msg_panorama_info_.header = image_msg->header;
    pub_panorama_info_.publish(msg_panorama_info_);
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::DualFisheyeToPanorama, nodelet::Nodelet);
