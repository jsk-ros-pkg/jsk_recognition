// -*- mode: C++ -*-
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
/*
 * draw_rects.h
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef DRAW_RECTS_H__
#define DRAW_RECTS_H__

#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>

#include <jsk_perception/DrawRectsConfig.h>
#include <jsk_recognition_msgs/ClassificationResult.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <sensor_msgs/Image.h>


namespace jsk_perception
{
  class DrawRects : public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef DrawRectsConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    jsk_recognition_msgs::RectArray,
    jsk_recognition_msgs::ClassificationResult> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    jsk_recognition_msgs::RectArray,
    jsk_recognition_msgs::ClassificationResult> AsyncPolicy;

    DrawRects(){}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config& config, uint32_t level);
    virtual void onMessage(
      const sensor_msgs::Image::ConstPtr& image,
      const jsk_recognition_msgs::RectArray::ConstPtr& rects,
      const jsk_recognition_msgs::ClassificationResult::ConstPtr& classes);
    virtual void fillEmptyClasses(
      const jsk_recognition_msgs::RectArray::ConstPtr& rects);
    virtual void drawRect(
      cv::Mat& img, const jsk_recognition_msgs::Rect& rect, const cv::Scalar& color);
    virtual void drawLabel(
      cv::Mat& img, const jsk_recognition_msgs::Rect& rect,
      const cv::Scalar& color, const std::string& label);
    virtual void randomColor(const int& label_num, const int& index, cv::Scalar& color);
    virtual bool isDarkColor(const cv::Scalar& color) {
      return color[2] * 0.299 + color[1] * 0.587 + color[0] * 0.114 <= 186.0;
    }

    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<AsyncPolicy> > async_;
    ros::Publisher pub_image_;
    message_filters::PassThrough<jsk_recognition_msgs::ClassificationResult> null_class_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<jsk_recognition_msgs::RectArray> sub_rects_;
    message_filters::Subscriber<jsk_recognition_msgs::ClassificationResult> sub_class_;

    bool use_async_;
    bool use_classification_result_;
    bool show_proba_;
    int queue_size_;
    int rect_boldness_;
    double label_size_;
    int label_boldness_;
    int label_font_;
    double label_margin_factor_;
    double resolution_factor_;
    int interpolation_method_;
  };
}


#endif // DRAW_RECTS_H__
