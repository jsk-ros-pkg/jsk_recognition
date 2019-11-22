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


#ifndef JSK_PERCEPTION_BOUNDING_BOX_TO_RECT_H_
#define JSK_PERCEPTION_BOUNDING_BOX_TO_RECT_H_

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArrayWithCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>
#include <tf/message_filter.h>

namespace jsk_perception
{
  class BoundingBoxToRect: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<BoundingBoxToRect> Ptr;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::CameraInfo,
      jsk_recognition_msgs::BoundingBoxArray > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::CameraInfo,
      jsk_recognition_msgs::BoundingBoxArray > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::CameraInfo,
      jsk_recognition_msgs::BoundingBox > SyncPolicyBox;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::CameraInfo,
      jsk_recognition_msgs::BoundingBox > ApproximateSyncPolicyBox;

    BoundingBoxToRect(): DiagnosticNodelet("BoundingBoxToRect") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void inputCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                               const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& boxes_msg);
    virtual void inputBoxCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                                  const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg);

    virtual void internalCallback(const jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo::ConstPtr& msg);

    boost::mutex mutex_;
    std::string frame_id_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> sub_box_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_boxes_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo> sub_box_with_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyBox> > sync_box_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicyBox> > async_box_;
    tf::TransformListener* tf_listener_;
    bool approximate_sync_;
    int tf_queue_size_;
    int queue_size_;
    boost::shared_ptr<tf::MessageFilter<jsk_recognition_msgs::BoundingBoxArrayWithCameraInfo> > tf_filter_;
    ros::Publisher pub_;
    ros::Publisher pub_internal_;
  private:
    
  };
}

#endif
