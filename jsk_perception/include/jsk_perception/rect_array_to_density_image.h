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


#ifndef JSK_PERCEPTION_RECT_ARRAY_TO_DENSITY_IMAGE_H_
#define JSK_PERCEPTION_RECT_ARRAY_TO_DENSITY_IMAGE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace jsk_perception
{
  class RectArrayToDensityImage: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, jsk_recognition_msgs::RectArray > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, jsk_recognition_msgs::RectArray > SyncPolicy;
    RectArrayToDensityImage(): DiagnosticNodelet("RectArrayToDensityImage") {}
    virtual ~RectArrayToDensityImage();
  protected:

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void convert(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const jsk_recognition_msgs::RectArray::ConstPtr& rects_msg);

    bool approximate_sync_;
    int queue_size_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<jsk_recognition_msgs::RectArray> sub_rects_;
    ros::Publisher pub_;

  private:

  };
}

#endif
