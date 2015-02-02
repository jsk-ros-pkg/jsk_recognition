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


#ifndef JSK_LIBFREENECT2_DRIVER_H_
#define JSK_LIBFREENECT2_DRIVER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <libfreenect2/opengl.h>
#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread.hpp>

namespace jsk_libfreenect2
{
  class Driver: public nodelet::Nodelet
  {
  public:
    typedef boost::shared_ptr<camera_info_manager::CameraInfoManager>
    CameraInfoManagerPtr;
  protected:
    virtual void onInit();
    virtual void run(const ros::TimerEvent&);
    ros::Timer timer_;
    libfreenect2::Freenect2* freenect2_;
    libfreenect2::Freenect2Device* dev_;
    libfreenect2::SyncMultiFrameListener* listener_;
    boost::mutex mutex_;
    sensor_msgs::CameraInfo::Ptr getRGBCameraInfo(
      libfreenect2::Frame* frame, ros::Time stamp, std::string frame_id);
    sensor_msgs::CameraInfo::Ptr getIRCameraInfo(
      libfreenect2::Frame* frame, ros::Time stamp, std::string frame_id);
    CameraInfoManagerPtr ir_caminfo_manager_, rgb_caminfo_manager_;
    libfreenect2::Freenect2Device::ColorCameraParams default_rgb_params_;
    libfreenect2::Freenect2Device::IrCameraParams default_ir_params_;
  private:
    
  };
}

#endif
