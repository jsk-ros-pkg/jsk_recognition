/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, JSK Lab.
*                2008, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <depth_image_publisher/DepthImagePublisherConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <boost/assign.hpp>
using namespace boost::assign;

namespace depth_image_publisher {
class DepthImagePublisherNodelet : public nodelet::Nodelet
{
  typedef dynamic_reconfigure::Server<depth_image_publisher::DepthImagePublisherConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> srv;

  image_transport::CameraPublisher pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_;

  cv::VideoCapture cap_;
  cv::Mat image_;
  int subscriber_count_;
  ros::Timer timer_;

  std::string frame_id_;
  std::string filename_;
  std::string encoding_;
  bool flip_image_;
  int flip_value_;
  sensor_msgs::CameraInfo camera_info_;
  

  void reconfigureCallback(depth_image_publisher::DepthImagePublisherConfig &new_config, uint32_t level)
  {
    frame_id_ = new_config.frame_id;

    timer_ = nh_.createTimer(ros::Duration(1.0/new_config.publish_rate), &DepthImagePublisherNodelet::do_work, this);

    camera_info_manager::CameraInfoManager c(nh_);
    if ( !new_config.camera_info_url.empty() ) {
      try {
        c.validateURL(new_config.camera_info_url);
        c.loadCameraInfo(new_config.camera_info_url);
        camera_info_ = c.getCameraInfo();
      } catch(cv::Exception &e) {
        NODELET_ERROR("camera calibration failed to load: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
      }
    }
    

  }

  void do_work(const ros::TimerEvent& event)
  {
    // Transform the image.
    try
    {
      if ( cap_.isOpened() ) {
        if ( ! cap_.read(image_) ) {
#if CV_MAJOR_VERSION >= 3
          cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
#else
          cap_.set(CV_CAP_PROP_POS_FRAMES, 0);
#endif
        }
      }
      if (flip_image_)
        cv::flip(image_, image_, flip_value_);

      sensor_msgs::ImagePtr out_img = cv_bridge::CvImage(
        std_msgs::Header(), encoding_, image_).toImageMsg();
      out_img->header.frame_id = frame_id_;
      out_img->header.stamp = ros::Time::now();
      camera_info_.header.frame_id = out_img->header.frame_id;
      camera_info_.header.stamp = out_img->header.stamp;

      pub_.publish(*out_img, camera_info_);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    subscriber_count_++;
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
  }

public:
  virtual void onInit()
  {
    subscriber_count_ = 0;
    nh_ = getPrivateNodeHandle();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
    pub_ = image_transport::ImageTransport(nh_).advertiseCamera("image_raw", 1);

    nh_.param("filename", filename_, std::string(""));
    nh_.param("encoding", encoding_, std::string("16UC1"));
    NODELET_INFO("File name for publishing image is : %s", filename_.c_str());
    try {
      image_ = cv::imread(filename_, cv::IMREAD_UNCHANGED);
      if (encoding_ == "16UC1") {
        image_.convertTo(image_, CV_16UC1);
      }
      else if (encoding_ == "32FC1") {
        image_.convertTo(image_, CV_32FC1);
        image_ = image_ / 1000.0;
      }
      if ( image_.empty() ) { // if filename is motion file or device file
        try {  // if filename is number
          int num = boost::lexical_cast<int>(filename_);//num is 1234798797
          cap_.open(num);
        } catch(boost::bad_lexical_cast &) { // if file name is string
          cap_.open(filename_);
        }
        CV_Assert(cap_.isOpened());
        cap_.read(image_);
#if CV_MAJOR_VERSION >= 3
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
#else
        cap_.set(CV_CAP_PROP_POS_FRAMES, 0);
#endif
      }
      CV_Assert(!image_.empty());
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Failed to load image (%s): %s %s %s %i", filename_.c_str(), e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    bool flip_horizontal;
    nh_.param("flip_horizontal", flip_horizontal, false);
    NODELET_INFO("Flip horizontal image is : %s",  ((flip_horizontal)?"true":"false"));

    bool flip_vertical;
    nh_.param("flip_vertical", flip_vertical, false);
    NODELET_INFO("Flip flip_vertical image is : %s", ((flip_vertical)?"true":"false"));

    // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
    // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
    flip_image_ = true;
    if (flip_horizontal && flip_vertical)
      flip_value_ = 0; // flip both, horizontal and vertical
    else if (flip_horizontal)
      flip_value_ = 1;
    else if (flip_vertical)
      flip_value_ = -1;
    else
      flip_image_ = false;

    camera_info_.width = image_.cols;
    camera_info_.height = image_.rows;
    camera_info_.distortion_model = "plumb_bob";
    camera_info_.D = list_of(0)(0)(0)(0)(0).convert_to_container<std::vector<double> >();
    camera_info_.K = list_of(1)(0)(camera_info_.width/2)(0)(1)(camera_info_.height/2)(0)(0)(1);
    camera_info_.R = list_of(1)(0)(0)(0)(1)(0)(0)(0)(1);
    camera_info_.P = list_of(1)(0)(camera_info_.width/2)(0)(0)(1)(camera_info_.height/2)(0)(0)(0)(1)(0);

    timer_ = nh_.createTimer(ros::Duration(1), &DepthImagePublisherNodelet::do_work, this);

    srv.reset(new ReconfigureServer(getPrivateNodeHandle()));
    ReconfigureServer::CallbackType f =
      boost::bind(&DepthImagePublisherNodelet::reconfigureCallback, this, _1, _2);
    srv->setCallback(f);
  }
};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(depth_image_publisher::DepthImagePublisherNodelet, nodelet::Nodelet);
