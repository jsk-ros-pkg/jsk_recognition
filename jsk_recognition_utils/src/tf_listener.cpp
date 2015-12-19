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

#include "jsk_recognition_utils/tf_listener.h"
#include "jsk_recognition_utils/tf_listener_singleton.h"

namespace jsk_recognition_utils
{

  TfListener::TfListener(const bool use_tf2, double check_frequenccy, ros::Duration timeout_padding):
    use_tf2_(use_tf2)
  {
    if (use_tf2_) {
       // tf_action??
      tf2_buffer_client_.reset(new tf2_ros::BufferClient("tf_action", check_frequenccy, timeout_padding));
    }
    else {
      tf_listener_ = TfListenerSingleton::getInstance();
    }
  }

  bool TfListener::waitForTransform(const std::string& target_frame,
                                    const std::string& source_frame,
                                    const ros::Time& time,
                                    const ros::Duration& timeout) const
  {
    if (use_tf2_) {
      return true;
    }
    else {
      return tf_listener_->waitForTransform(target_frame, source_frame,
                                            time, timeout);
    }
  }

  void TfListener::lookupTransform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const ros::Time& time,
                                   tf::StampedTransform& transform) const
  {
    if (use_tf2_) {
      geometry_msgs::TransformStamped transform_msg
        = tf2_buffer_client_->lookupTransform(target_frame, source_frame, time);
      tf::transformStampedMsgToTF(transform_msg, transform);
    }
    else {
      tf_listener_->lookupTransform(target_frame, source_frame,
                                    time, transform);
    }
  }
}
