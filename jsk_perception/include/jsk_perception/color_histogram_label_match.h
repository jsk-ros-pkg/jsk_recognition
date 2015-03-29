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


#ifndef JSK_PERCEPTION_COLOR_HISTOGRAM_LABEL_MATCH_H_
#define JSK_PERCEPTION_COLOR_HISTOGRAM_LABEL_MATCH_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/ColorHistogram.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/pass_through.h>
#include <jsk_perception/ColorHistogramLabelMatchConfig.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  class ColorHistogramLabelMatch: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef ColorHistogramLabelMatchConfig Config;
    //typedef message_filters::sync_policies::ExactTime<
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,         // image
      sensor_msgs::Image,         // label
      sensor_msgs::Image          // mask
      > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,         // image
      sensor_msgs::Image         // label
      > SyncPolicyWithoutMask;

    ColorHistogramLabelMatch(): DiagnosticNodelet("ColorHistogramLabelMatch") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void match(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::Image::ConstPtr& label_msg,
      const sensor_msgs::Image::ConstPtr& mask_msg);
    virtual void match(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::Image::ConstPtr& label_msg);
    virtual void histogramCallback(
      const jsk_recognition_msgs::ColorHistogram::ConstPtr& histogram_msg);
    virtual bool isMasked(const cv::Mat& original_image,
                          const cv::Mat& masked_image);
    virtual void getLabels(const cv::Mat& label, std::vector<int>& labels);
    virtual void getMaskImage(const cv::Mat& label_image,
                                 const int label,
                                 cv::Mat& mask);
    virtual double coefficients(const cv::Mat& ref_hist,
                                const cv::Mat& target_hist);
    virtual void configCallback(Config &config, uint32_t level);
    
    float max_value_;
    float min_value_;
    float coef_threshold_;
    float masked_coefficient_;
    int threshold_method_;
    bool use_mask_;
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyWithoutMask> > sync_wo_mask_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_label_;
    message_filters::Subscriber<sensor_msgs::Image> sub_mask_;
    int coefficient_method_;
    ros::Subscriber sub_histogram_;
    cv::Mat histogram_;
    ros::Publisher pub_debug_;
    ros::Publisher pub_coefficient_image_;
    ros::Publisher pub_mask_;
    ros::Publisher pub_result_;
  private:
    
  };
}

#endif
