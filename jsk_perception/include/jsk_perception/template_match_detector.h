// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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


#ifndef JSK_PERCEPTION_TEMPLATE_MATCH_DETECTOR_H_
#define JSK_PERCEPTION_TEMPLATE_MATCH_DETECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/TemplateMatchDetectorConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#ifdef USE_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>
#endif

namespace jsk_perception
{
  template <class CLASS_T>
  bool rect_horizontal(const CLASS_T& left, const CLASS_T& right)
  {
    return left.x + left.width / 2.0 < right.x + right.width / 2.0;
  }

  template <class CLASS_T>
  bool rect_vertical(const CLASS_T& left, const CLASS_T& right)
  {
    return left.y + left.height / 2.0 < right.y + right.height / 2.0;
  }

  class TemplateMatchDetector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef jsk_perception::TemplateMatchDetectorConfig Config;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      sensor_msgs::CameraInfo > SyncPolicy;

    TemplateMatchDetector(): DiagnosticNodelet("TemplateMatchDetector") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual bool is_overlaped(cv::Rect rect1, cv::Rect rect2);
    virtual void rectangleIntegration(std::vector<cv::Rect> rects,
                                      std::vector<double> score,
                                      std::vector<cv::Rect>& result);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void apply(const sensor_msgs::Image::ConstPtr& image_msg,
                       const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    boost::mutex mutex_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    ros::Publisher img_pub_;
    ros::Publisher rects_pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;

    std::string template_name_;
    bool screen_debug_, specify_target_, flip_template_, use_cuda;
    double min_scale, max_scale, matching_thre, stored_thre;
    int resize_num, target_num, position_op, max_detect_num, sort_op;
    cv::Mat tmpl_img;

#ifdef USE_CUDA
    cv::cuda::GpuMat cuda_tmpl_img;
#endif

  private:

  };
}

#endif
