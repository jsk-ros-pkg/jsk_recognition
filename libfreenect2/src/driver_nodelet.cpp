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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "libfreenect2/driver.h"

namespace libfreenect2
{
  void Driver::onInit()
  {
    freenect2_ = new Freenect2();
    dev_ = freenect2_->openDefaultDevice();

    if (dev_ == 0)
    {
      NODELET_FATAL("[%s] no device connected", __FUNCTION__);
      return;
    }
    listener_ = new SyncMultiFrameListener(
      Frame::Color | Frame::Ir | Frame::Depth);
    dev_->setColorFrameListener(listener_);
    dev_->setIrAndDepthFrameListener(listener_);
    dev_->start();
    NODELET_INFO_STREAM(__FUNCTION__ <<
                        "device serial: " << dev_->getSerialNumber());
    NODELET_INFO_STREAM(__FUNCTION__ <<
                        "device firmware: " << dev_->getFirmwareVersion());
    libfreenect2::FrameMap frames;
    while(ros::ok())
    {
      listener_->waitForNewFrame(frames);
      {
        boost::mutex::scoped_lock lock(mutex_);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
        cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
        cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
        int key = cv::waitKey(1);
        listener_->release(frames);
      }
    }
    dev_->stop();
    dev_->close();
  }
}

#include <pluginlib/class_list_macros.h>
typedef libfreenect2::Driver Driver;
PLUGINLIB_DECLARE_CLASS (libfreenect2, Driver, Driver, nodelet::Nodelet);

