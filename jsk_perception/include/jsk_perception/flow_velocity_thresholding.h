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


#ifndef JSK_PERCEPTION_FLOW_VELOCITY_THRESHOLDING_H_
#define JSK_PERCEPTION_FLOW_VELOCITY_THRESHOLDING_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv_apps/FlowArrayStamped.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_perception/FlowVelocityThresholdingConfig.h"

namespace jsk_perception
{
  class FlowVelocityThresholding: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    FlowVelocityThresholding(): DiagnosticNodelet("FlowVelocityThresholding") {}
    typedef jsk_perception::FlowVelocityThresholdingConfig Config;
    typedef message_filters::sync_policies::ApproximateTime<
      opencv_apps::FlowArrayStamped,
      sensor_msgs::CameraInfo > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      opencv_apps::FlowArrayStamped,
      sensor_msgs::CameraInfo > SyncPolicy;
  protected:
    virtual void onInit();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void callback(
      const opencv_apps::FlowArrayStamped::ConstPtr& flows_msg);
    virtual void callback(
      const opencv_apps::FlowArrayStamped::ConstPtr& flows_msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual void process(
      const opencv_apps::FlowArrayStamped::ConstPtr& flows_msg,
      const int image_height,
      const int image_width);

    bool use_camera_info_;
    bool approximate_sync_;
    int queue_size_;
    int image_height_;
    int image_width_;
    int window_size_;
    double threshold_;

    boost::mutex mutex_;
    ros::Publisher pub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<opencv_apps::FlowArrayStamped> sub_flow_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  private:
  };
}  // namespace jsk_perception

#endif  // JSK_PERCEPTION_FLOW_VELOCITY_THRESHOLDING_H_
