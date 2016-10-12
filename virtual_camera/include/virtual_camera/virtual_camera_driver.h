/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada
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
 *   * Neither the name of the Kentaro Wada nor the names of its
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

#ifndef VIRTUAL_CAMERA_VIRTUAL_CAMERA_DRIVER_H_
#define VIRTUAL_CAMERA_VIRTUAL_CAMERA_DRIVER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace virtual_camera
{

class VirtualCameraDriver: public nodelet::Nodelet
{
public:
  VirtualCameraDriver():
    publish_info_(false),
    publish_image_(false),
    publish_cloud_(false),
    time_prev_pub_info_(ros::Time(0)),
    time_prev_pub_image_(ros::Time(0)),
    time_prev_pub_cloud_(ros::Time(0)) {}
protected:
  virtual void onInit();
  void publishCb(const ros::TimerEvent& event);

  boost::mutex mutex_;

  ros::NodeHandle* nh_;
  ros::NodeHandle* pnh_;

  ros::Timer timer_pub_;

  ros::Publisher pub_info_;
  ros::Publisher pub_image_;
  ros::Publisher pub_cloud_;

  sensor_msgs::CameraInfo info_msg_;
  sensor_msgs::ImagePtr image_msg_;
  sensor_msgs::PointCloud2 cloud_msg_;

  bool publish_info_;
  bool publish_image_;
  bool publish_cloud_;
  double rate_info_;
  double rate_image_;
  double rate_cloud_;
  ros::Time time_prev_pub_info_;
  ros::Time time_prev_pub_image_;
  ros::Time time_prev_pub_cloud_;
private:
};

}  // namespace virtual_camera

#endif  // VIRTUAL_CAMERA_VIRTUAL_CAMERA_DRIVER_H_
