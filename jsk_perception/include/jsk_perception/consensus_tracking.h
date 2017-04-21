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

#ifndef JSK_PERCEPTION_CONSENSUS_TRACKING_H_
#define JSK_PERCEPTION_CONSENSUS_TRACKING_H_

#include <geometry_msgs/PolygonStamped.h>
#include <libcmt/CMT.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>

namespace jsk_perception
{
  class ConsensusTracking : public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ConsensusTracking() :
      DiagnosticNodelet("ConsensusTracking"),
      window_initialized_(false) {}
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      geometry_msgs::PolygonStamped> ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      geometry_msgs::PolygonStamped> ExactSyncPolicy;
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void getTrackingResult(const sensor_msgs::Image::ConstPtr& image_msg);
    void setInitialWindow(const sensor_msgs::Image::ConstPtr& img_msg,
                          const geometry_msgs::PolygonStamped::ConstPtr& poly_msg);

    ros::Publisher pub_mask_image_;
    ros::Publisher pub_debug_image_;

    ros::Subscriber sub_image_;
    boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_to_init_;
    message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_polygon_to_init_;

    CMT cmt;

    boost::mutex mutex_;
    bool window_initialized_;
    bool approximate_sync_;
    int queue_size_;
  private:
  };
}  // namespace jsk_perception

#endif // JSK_PERCEPTION_CONSENSUS_TRACKING_H_
