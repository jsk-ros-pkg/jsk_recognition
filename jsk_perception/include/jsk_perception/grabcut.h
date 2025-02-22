// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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


#ifndef JSK_PERCEPTION_GRABCUT_H_
#define JSK_PERCEPTION_GRABCUT_H_

#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_perception/GrabCutConfig.h"

namespace jsk_perception
{
  class GrabCut: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef GrabCutConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::Image> SyncPolicy;
    
    typedef boost::shared_ptr<GrabCut> Ptr;
    GrabCut() : DiagnosticNodelet("GrabCut") {}
    virtual ~GrabCut();

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void segment(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::Image::ConstPtr& foreground_msg,
      const sensor_msgs::Image::ConstPtr& background_msg);
    virtual void configCallback (Config &config, uint32_t level);    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    ros::Publisher pub_foreground_;
    ros::Publisher pub_background_;
    ros::Publisher pub_foreground_mask_;
    ros::Publisher pub_background_mask_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Image> sub_foreground_;
    message_filters::Subscriber<sensor_msgs::Image> sub_background_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    bool use_probable_pixel_seed_;    
  private:
    
  };
}

#endif
