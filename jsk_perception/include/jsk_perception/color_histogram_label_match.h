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
#include <jsk_pcl_ros/ColorHistogram.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_perception/ColorHistogramLabelMatchConfig.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  class ColorHistogramLabelMatch: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef ColorHistogramLabelMatchConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::Image> SyncPolicy;

    ColorHistogramLabelMatch(): DiagnosticNodelet("ColorHistogramLabelMatch") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void match(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::Image::ConstPtr& label_msg);
    virtual void histogramCallback(
      const jsk_pcl_ros::ColorHistogram::ConstPtr& histogram_msg);
    virtual void getLabels(const cv::Mat& label, std::vector<int>& labels);
    virtual void getMaskImage(const cv::Mat& label_image,
                                 const int label,
                                 cv::Mat& mask);
    virtual double coefficients(const cv::Mat& ref_hist,
                                const cv::Mat& target_hist);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void normalizeHistogram(cv::Mat& hist);
    
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_label_;
    int coefficient_method_;
    ros::Subscriber sub_histogram_;
    cv::Mat histogram_;
    ros::Publisher pub_debug_;
  private:
    
  };
}

#endif
