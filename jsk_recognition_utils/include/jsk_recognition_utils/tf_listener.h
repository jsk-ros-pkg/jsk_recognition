// -*- Mode: c++ -*-
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

#include <tf/transform_listener.h>
#include <tf2_ros/buffer_client.h>

namespace jsk_recognition_utils
{
  /**
   * @brief
   * Class to wrap tf and tf2 to provide
   * same API.
   */
  class TfListener
  {
  public:
    typedef boost::shared_ptr<TfListener> Ptr;
    TfListener(const bool use_tf2,
               double check_frequenccy = 10.0,
               ros::Duration timeout_padding = ros::Duration(2.0));
    bool waitForTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const ros::Time& time,
                          const ros::Duration& timeout) const;
    void lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         tf::StampedTransform& transform) const;
    
  protected:
    bool use_tf2_;
    tf::TransformListener* tf_listener_;
    boost::shared_ptr<tf2_ros::BufferClient> tf2_buffer_client_;
  private:
    
  };
}
