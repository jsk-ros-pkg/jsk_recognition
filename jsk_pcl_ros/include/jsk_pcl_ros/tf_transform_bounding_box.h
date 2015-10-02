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

#ifndef JSK_PCL_ROS_TF_TRANFORM_BOUNDING_BOX_H_
#define JSK_PCL_ROS_TF_TRANFORM_BOUNDING_BOX_H_


#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace jsk_pcl_ros
{

  class TfTransformBoundingBox: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    TfTransformBoundingBox(): DiagnosticNodelet("TfTransformBoundingBox") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void transform(const jsk_recognition_msgs::BoundingBox::ConstPtr& msg);
    
    ros::Subscriber sub_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> sub_filter_;
    ros::Publisher pub_;
    std::string target_frame_id_;
    tf::TransformListener* tf_listener_;
    boost::shared_ptr<tf::MessageFilter<jsk_recognition_msgs::BoundingBox> > tf_filter_;
    bool use_latest_tf_;
    int tf_queue_size_;
  };
}


#endif
