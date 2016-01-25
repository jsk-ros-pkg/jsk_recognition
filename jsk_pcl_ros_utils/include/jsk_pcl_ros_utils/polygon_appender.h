// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#ifndef JSK_PCL_ROS_UTILS_POLYGON_APPENDER_H_
#define JSK_PCL_ROS_UTILS_POLYGON_APPENDER_H_

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <pcl_ros/pcl_nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros_utils
{
  class PolygonAppender: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    jsk_recognition_msgs::PolygonArray, jsk_recognition_msgs::ModelCoefficientsArray,
    jsk_recognition_msgs::PolygonArray, jsk_recognition_msgs::ModelCoefficientsArray> SyncPolicy2;
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void appendAndPublish(
      const std::vector<jsk_recognition_msgs::PolygonArray::ConstPtr>& arrays,
      const std::vector<jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr>& coefficients_array);
    virtual void callback2(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg0,
                           const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients0,
                           const jsk_recognition_msgs::PolygonArray::ConstPtr& msg1,
                           const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients1);
    
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon0_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon1_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients0_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients1_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy2> >sync_;
    ros::Publisher pub_polygon_, pub_coefficients_;
  private:
  };
}

#endif 
