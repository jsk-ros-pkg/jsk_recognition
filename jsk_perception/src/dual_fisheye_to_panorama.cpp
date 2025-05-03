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

#include "jsk_perception/dual_fisheye_to_panorama.h"
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
    pnh_->param("blend_image_height", blend_image_height_, 1920);
    pnh_->param("blend_image_width", blend_image_width_, 3840);
    pnh_->param("blend_param_p_wid", blend_param_p_wid_, 55);
    pnh_->param("blend_param_p_x1", blend_param_p_x1_, 90 - 15);
    pnh_->param("blend_param_p_x2", blend_param_p_x2_, 1780 - 5);
    pnh_->param("blend_param_row_start", blend_param_row_start_, 590);
    pnh_->param("blend_param_row_end", blend_param_row_end_, 1320);
    pnh_->param("output_image_height", output_image_height_, 2000);
    pnh_->param("output_image_width", output_image_width_, 4000);
    ROS_INFO("light_compen : %s", enb_lc_?"true":"false");
    ROS_INFO("refine_align : %s", enb_ra_?"true":"false");
    ROS_INFO("fovd         : %7.3f", fovd_);
    ROS_INFO("save_unwarped: %s", save_unwarped_?"true":"false");
    ROS_INFO("mls_map_path : %s", mls_map_path_.c_str());
    ROS_INFO("blend_image_height : %d", blend_image_height_);
    ROS_INFO("blend_image_width  : %d", blend_image_width_);
    ROS_INFO("blend_param_p_wid : %d", blend_param_p_wid_);
    ROS_INFO("blend_param_p_x1 : %d", blend_param_p_x1_);
    ROS_INFO("blend_param_p_x2 : %d", blend_param_p_x2_);
    ROS_INFO("blend_param_row_start : %d", blend_param_row_start_);
    ROS_INFO("blend_param_row_end : %d", blend_param_row_end_);
    ROS_INFO("output_image_height : %d", output_image_height_);
    ROS_INFO("output_image_width  : %d", output_image_width_);
    pub_panorama_image_ = advertiseImage(*pnh_, "output", 1);
    pub_panorama_info_ = advertise<jsk_recognition_msgs::PanoramaInfo>(*pnh_, "panorama_info", 1);

    sticher_initialized_ = false;
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&DualFisheyeToPanorama::configCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    srv_->setCallback (f);

    msg_panorama_info_.projection_model = "equirectangular";
    msg_panorama_info_.theta_min = 0;
    msg_panorama_info_.theta_max = M_PI;
    msg_panorama_info_.phi_min = -M_PI;
    msg_panorama_info_.phi_max = M_PI;
    msg_panorama_info_.image_height = output_image_height_;
    msg_panorama_info_.image_width = output_image_width_;

    onInitPostProcess();
  }

  void DualFisheyeToPanorama::configCallback(Config &new_config, uint32_t level)
  {
  }


  void DualFisheyeToPanorama::subscribe()
  {
    image_transport::ImageTransport it(*pnh_);
    image_transport::TransportHints hints("raw", ros::TransportHints(), *pnh_, "image_transport");
    sub_image_ = it.subscribe(pnh_->resolveName("input"), 1, &DualFisheyeToPanorama::rectify, this, hints);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void DualFisheyeToPanorama::unsubscribe()
  {
    sub_image_.shutdown();
  }


  void DualFisheyeToPanorama::rectify(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    cv::Mat img;
    cv::resize(cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image, img, cv::Size(blend_image_width_,blend_image_height_));

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
    pano = stitcher_->stitch(img_l, img_r,
                             blend_param_p_wid_,
                             blend_param_p_x1_,
                             blend_param_p_x2_,
                             blend_param_row_start_,
                             blend_param_row_end_
                             );
    // resize panorama image;
    cv::Mat pano_resized;
    cv::resize(pano, pano_resized, cv::Size(output_image_width_,output_image_height_));

    pub_panorama_image_.publish(cv_bridge::CvImage(image_msg->header,
                                                   image_msg->encoding,
                                                   pano_resized).toImageMsg());
    msg_panorama_info_.header = image_msg->header;
    pub_panorama_info_.publish(msg_panorama_info_);
  }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS (jsk_perception::DualFisheyeToPanorama, nodelet::Nodelet);
