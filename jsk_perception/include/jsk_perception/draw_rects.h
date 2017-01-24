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
/*
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef JSK_PERCEPTION_DRAW_RECTS_H__
#define JSK_PERCEPTION_DRAW_RECTS_H__

#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <jsk_topic_tools/connection_based_nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/LabeledRectArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace jsk_perception
{
  class DrawRects : public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, jsk_recognition_msgs::RectArray> ApproximateSyncPolicyRects;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, jsk_recognition_msgs::RectArray> ExactSyncPolicyRects;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, jsk_recognition_msgs::LabeledRectArray> ApproximateSyncPolicyLabeledRects;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, jsk_recognition_msgs::LabeledRectArray> ExactSyncPolicyLabeledRects;

    DrawRects(){}
  protected:
    bool approximate_sync_;
    int queue_size_;
    std::vector<std::string> labels_;
    int label_num_;
    ros::Publisher pub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<jsk_recognition_msgs::RectArray> sub_rects_;
    message_filters::Subscriber<jsk_recognition_msgs::LabeledRectArray> sub_labeled_rects_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicyRects> > async_rects_;
    boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicyRects> > sync_rects_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicyLabeledRects> > async_labeled_rects_;
    boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicyLabeledRects> > sync_labeled_rects_;

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    void getRandomColor(const int index, cv::Scalar &color);
    void getLabelColor(const jsk_recognition_msgs::Label &label, cv::Scalar& color);
    void drawRect(cv::Mat& mat,
                  const jsk_recognition_msgs::Rect &msg, const cv::Scalar& color);
    void drawLabel(cv::Mat& mat,
                   const jsk_recognition_msgs::LabeledRect &msg, const cv::Scalar& color);
    void callbackRects(
      const sensor_msgs::Image::ConstPtr &img,
      const jsk_recognition_msgs::RectArray::ConstPtr &rects);
    void callbackLabeledRects(
      const sensor_msgs::Image::ConstPtr &img,
      const jsk_recognition_msgs::LabeledRectArray::ConstPtr &rects);
  }; // class DrawRects
} // namespace jsk_perception


#endif // JSK_PERCEPTION_DRAW_RECTS_H__
