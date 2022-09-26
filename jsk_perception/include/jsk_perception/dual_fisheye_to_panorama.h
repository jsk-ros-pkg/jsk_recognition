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


#ifndef JSK_PERCEPTION_DUAL_FISHEYE_TO_PANORAMA_H_
#define JSK_PERCEPTION_DUAL_FISHEYE_TO_PANORAMA_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/PanoramaInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/pass_through.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/DualFisheyeConfig.h>

#include <opencv2/opencv.hpp>

#include "jsk_perception/fisheye_stitcher.hpp"


namespace jsk_perception
{
  class DualFisheyeToPanorama: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,         // image
      sensor_msgs::CameraInfo        // camera info
      > SyncPolicy;
    typedef jsk_perception::DualFisheyeConfig Config;

    DualFisheyeToPanorama(): DiagnosticNodelet("DualFisheyeToPanorama") {}
  protected:
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    void configCallback(Config &new_config, uint32_t level);
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void rectify(const sensor_msgs::Image::ConstPtr& image_msg);

    bool sticher_initialized_;
    boost::shared_ptr<stitcher::FisheyeStitcher> stitcher_;
    ros::Subscriber sub_image_;
    ros::Publisher pub_panorama_image_;
    ros::Publisher pub_panorama_info_;

    jsk_recognition_msgs::PanoramaInfo msg_panorama_info_;

    bool   enb_lc_;
    bool   enb_ra_;
    bool  save_unwarped_;
    float  fovd_;
    int blend_image_height_;
    int blend_image_width_;
    int blend_param_p_wid_;
    int blend_param_p_x1_;
    int blend_param_p_x2_;
    int blend_param_row_start_;
    int blend_param_row_end_;
    int output_image_height_;
    int output_image_width_;
    std::string mls_map_path_;

  private:
  };
}

#endif
