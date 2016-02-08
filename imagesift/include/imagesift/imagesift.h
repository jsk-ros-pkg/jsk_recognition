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


#ifndef IMAGESIFT_SIFT_NODE_H_
#define IMAGESIFT_SIFT_NODE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <posedetection_msgs/ImageFeature0D.h>
#include <posedetection_msgs/Feature0DDetect.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <cv_bridge/cv_bridge.h>
#include <siftfast/siftfast.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

namespace imagesift
{
  class SiftNode: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      sensor_msgs::Image > SyncPolicy;
    ros::WallTime lasttime;
    SiftNode(): DiagnosticNodelet("SiftNode") {}
  protected:
    bool _bInfoInitialized;
    bool _useMask;
    boost::mutex _mutex;
    boost::shared_ptr<image_transport::ImageTransport> _it;
    image_transport::Subscriber _subImage;
    // for useMask
    message_filters::Subscriber<sensor_msgs::Image> _subImageWithMask;
    message_filters::Subscriber<sensor_msgs::Image> _subMask;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > _sync;
    ros::ServiceServer _srvDetect;
    ros::Subscriber _subInfo;
    ros::Publisher _pubFeatures;
    ros::Publisher _pubSift;
    posedetection_msgs::ImageFeature0D _sift_msg;

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    void infoCb(const sensor_msgs::CameraInfoConstPtr& msg_ptr);
    bool detectCb(posedetection_msgs::Feature0DDetect::Request& req,
                  posedetection_msgs::Feature0DDetect::Response& res);
    bool detect(posedetection_msgs::Feature0D& features,
                const sensor_msgs::Image& imagemsg,
                const sensor_msgs::Image::ConstPtr& mask_ptr);
    void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr,
                 const sensor_msgs::ImageConstPtr& mask_ptr);
    void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr);
  private:
  };
}

#endif
