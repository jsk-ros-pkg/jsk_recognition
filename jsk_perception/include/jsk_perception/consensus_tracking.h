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


#ifndef JSK_PERCEPTION_FISHEYE_TO_PANORAMA_H_
#define JSK_PERCEPTION_FISHEYE_TO_PANORAMA_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <jsk_perception/FisheyeConfig.h>
#include <jsk_recognition_msgs/Rect.h>
#include <geometry_msgs/PolygonStamped.h>
#include <libcmt/CMT.h>

#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  class ConsensusTracking: public jsk_topic_tools::DiagnosticNodelet
  {
  public:

    ConsensusTracking(): DiagnosticNodelet("ConsensusTracking") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    inline double interpolate(double rate, double first, double second){return (1.0 - rate) * first + rate * second;};
    virtual void tracking(const sensor_msgs::Image::ConstPtr& image_msg);
    void reset_rect(jsk_recognition_msgs::Rect rc);
    void reset_rect_with_poly(geometry_msgs::PolygonStamped poly);
 
    ros::Subscriber sub_image_;
    ros::Subscriber sub_rect_;
    ros::Subscriber sub_rect_poly_;
    ros::Publisher pub_image_;
    ros::Publisher pub_mask_image_;
    bool first_initialize_;
    bool show_window_;

    cv::Point2f init_top_left_;
    cv::Point2f init_bottom_right_;


    CMT cmt;
 private:
    
  };
}

#endif
