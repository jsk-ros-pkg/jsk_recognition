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


#ifndef JSK_PERCEPTION_SLIC_SUPERPIXELS_H_
#define JSK_PERCEPTION_SLIC_SUPERPIXELS_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#if ( CV_MAJOR_VERSION >= 4)
#else
#include <opencv/cv.h>
#endif

#include <jsk_perception/SLICSuperPixelsConfig.h>
#include <dynamic_reconfigure/server.h>

namespace jsk_perception
{
  class SLICSuperPixels: public nodelet::Nodelet
  {
  public:
    typedef SLICSuperPixelsConfig Config;
    
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr& image);
    virtual void configCallback(Config &config, uint32_t level);

    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::NodeHandle nh_, pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    boost::mutex mutex_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_debug_;
    ros::Publisher pub_debug_mean_color_;
    ros::Publisher pub_debug_center_grid_;
    ros::Publisher pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    int number_of_super_pixels_;
    int weight_;
    bool debug_image_;
  private:
  };
}

#endif
