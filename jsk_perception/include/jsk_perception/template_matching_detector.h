// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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


#ifndef JSK_PERCEPTION_TEMPLATE_MATCHING_DETECTOR_H_
#define JSK_PERCEPTION_TEMPLATE_MATCHING_DETECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/TemplateMatchingDetectorConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/dnn.hpp>

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

  class TemplateMatchingDetector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef jsk_perception::TemplateMatchingDetectorConfig Config;
    TemplateMatchingDetector(): DiagnosticNodelet("TemplateMatchingDetector") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void sortRects(std::vector<cv::Rect> rects);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void apply(const sensor_msgs::Image::ConstPtr& image_msg);

    Config config_;
    boost::mutex mutex_;
    ros::Subscriber sub_;
    ros::Publisher img_pub_;
    ros::Publisher rects_pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    std::string template_filename_;
    bool update_matching_threshold_, check_flipped_image_, use_cuda;
    static bool need_config_update_;
    double min_scale, max_scale, matching_threshold;
    int resize_template_num, target_num;
    int queue_size_;
    cv::Mat template_image;

#ifdef USE_CUDA
    cv::cuda::GpuMat cuda_template_image;
#endif

  private:

  };
}

#endif
