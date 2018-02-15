// -*- mode: c++ -*-
/*
 * Copyright (c) 2014 individual JSK Lab members.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include "jsk_libfreenect2/driver.h"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/distortion_models.h>
#include <limits>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>

namespace jsk_libfreenect2
{
  void Driver::onInit()
  {
    glfwInit();
    timer_ = getNodeHandle().createTimer(
      ros::Duration(5.0),
      boost::bind(&Driver::run, this, _1), true);
  }
  
  void Driver::run(const ros::TimerEvent&) {
    freenect2_ = new libfreenect2::Freenect2();
    dev_ = freenect2_->openDefaultDevice();
    if (dev_ == 0)
    {
      NODELET_FATAL("[Driver::run] no device connected");
      return;
    }
    listener_ = new libfreenect2::SyncMultiFrameListener(
      libfreenect2::Frame::Color |
      libfreenect2::Frame::Ir |
      libfreenect2::Frame::Depth);
    dev_->setColorFrameListener(listener_);
    dev_->setIrAndDepthFrameListener(listener_);
    dev_->start();
    NODELET_INFO_STREAM("[Driver::run] device serial: "
                        << dev_->getSerialNumber());
    NODELET_INFO_STREAM("[Driver::run] device firmware: "
                        << dev_->getFirmwareVersion());

    
    
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle pnh = getPrivateNodeHandle();
    image_transport::ImageTransport it(pnh);
    ros::NodeHandle rgb_nh(nh, "rgb");
    image_transport::ImageTransport rgb_it(rgb_nh);
    ros::NodeHandle ir_nh(nh, "ir");
    image_transport::ImageTransport ir_it(ir_nh);
    ros::NodeHandle depth_nh(nh, "depth");
    image_transport::ImageTransport depth_it(depth_nh);
    ros::NodeHandle depth_registered_nh(nh, "depth_registered");
    image_transport::ImageTransport depth_registered_it(depth_registered_nh);
    ros::NodeHandle projector_nh(nh, "projector");
    
    // read parameters
    // std::string rgb_camera_info_url, depth_camera_info_url;
    // pnh.param("rgb_camera_info_url", rgb_camera_info_url, std::string());
    // pnh.param("depth_camera_info_url", depth_camera_info_url, std::string());

    std::string rgb_frame_id, depth_frame_id;
    pnh.param("rgb_frame_id",   rgb_frame_id,   std::string("/openni_rgb_optical_frame"));
    pnh.param("depth_frame_id", depth_frame_id, std::string("/openni_depth_optical_frame"));

    default_rgb_params_ = dev_->getColorCameraParams();
    default_ir_params_ = dev_->getIrCameraParams();
    // rgb_caminfo_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(
    //   rgb_nh, "rgb", rgb_camera_info_url);
    // ir_caminfo_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(
    //   ir_nh,  "depth",  depth_camera_info_url);
    // if (!rgb_caminfo_manager_->isCalibrated())
    //   NODELET_WARN("Using default parameters for RGB camera calibration.");
    // if (!ir_caminfo_manager_->isCalibrated())
    //   NODELET_WARN("Using default parameters for IR/depth camera calibration.");
    
    libfreenect2::FrameMap frames;
    image_transport::CameraPublisher depth_pub
      = depth_it.advertiseCamera("image_raw", 1);
    image_transport::CameraPublisher ir_pub
      = ir_it.advertiseCamera("image_raw", 1);
    image_transport::CameraPublisher color_pub
      = rgb_it.advertiseCamera("image_raw", 1);
    
    while(ros::ok())
    {
      listener_->waitForNewFrame(frames);
      {
        boost::mutex::scoped_lock lock(mutex_);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        cv::Mat cvrgb(rgb->height, rgb->width, CV_8UC3, rgb->data);
        cv::Mat cvir(ir->height, ir->width, CV_32FC1, ir->data);
        cv::Mat cvdepth(depth->height, depth->width, CV_32FC1, depth->data);
        
        // int count = 0;
        // for (int i = 0; i < cvir.cols; i++) {
        //   for (int j = 0; j < cvir.rows; j++) {
        //     if (cvir.at<float>(j, i) == 65535) {
        //       NODELET_INFO("[%d, %d] is limit: %f", i, j, cvir.at<float>(j, i));
        //       ++count;
        //     }
        //   }
        // }
        // NODELET_INFO("%d limit", count);
        // quick hack
        // [76, 345], [82, 354], [103, 383], [212, 20], [340, 403], [479, 150]
        // is broken
        // std::vector<boost::tuple<int, int> > forbidden_pixels;
        // forbidden_pixels.push_back(boost::make_tuple(76, 345));
        // forbidden_pixels.push_back(boost::make_tuple(82, 354));
        // forbidden_pixels.push_back(boost::make_tuple(103, 383));
        // forbidden_pixels.push_back(boost::make_tuple(212, 20));
        // forbidden_pixels.push_back(boost::make_tuple(340, 403));
        // forbidden_pixels.push_back(boost::make_tuple(479, 150));
        // for (size_t i = 0; i < forbidden_pixels.size(); i++) {
        //   boost::tuple<int, int> pixel = forbidden_pixels[i];
        //   cvir.at<float>(pixel.get<1>(), pixel.get<0>()) = 0;
        // }
        
        ros::Time stamp = ros::Time::now();
        std_msgs::Header rgb_header, ir_header;
        rgb_header.stamp = stamp;
        ir_header.stamp = stamp;
        rgb_header.frame_id = rgb_frame_id;
        ir_header.frame_id = depth_frame_id;
        cv_bridge::CvImage rgb_cvimage(rgb_header, "bgr8", cvrgb);
        color_pub.publish(rgb_cvimage.toImageMsg(), getRGBCameraInfo(rgb, stamp, rgb_frame_id));

        // cv_bridge::CvImage depth_cvimage(ir_header, "16UC1", cvdepth / 1000.0);
        cv_bridge::CvImage depth_cvimage(ir_header, "32FC1", cvdepth / 1000.0);
        depth_pub.publish(depth_cvimage.toImageMsg(), getIRCameraInfo(depth, stamp, depth_frame_id));


        cv_bridge::CvImage ir_cvimage(ir_header, "32FC1", cvir / 1000.0);
        ir_pub.publish(ir_cvimage.toImageMsg(), getIRCameraInfo(ir, stamp, depth_frame_id));
        
        int key = cv::waitKey(1);
        listener_->release(frames);
      }
    }
    dev_->stop();
    dev_->close();
  }

  sensor_msgs::CameraInfo::Ptr Driver::getIRCameraInfo(
    libfreenect2::Frame* frame, ros::Time stamp, std::string frame_id)
  {
    if (ir_caminfo_manager_ && ir_caminfo_manager_->isCalibrated()) {
      sensor_msgs::CameraInfo info = ir_caminfo_manager_->getCameraInfo();
      info.header.stamp = stamp;
      info.header.frame_id = frame_id;
      return boost::make_shared<sensor_msgs::CameraInfo>(info);
    }
    else {
      sensor_msgs::CameraInfo info;
      info.width = frame->width;
      info.height = frame->height;
      info.header.stamp = stamp;
      info.header.frame_id = frame_id;
      info.D.resize(5, 0.0);
      info.K.assign(0.0);
      info.R.assign(0.0);
      info.P.assign(0.0);
      info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      info.D[0] = default_ir_params_.k1;
      info.D[1] = default_ir_params_.k2;
      info.D[2] = default_ir_params_.p1;
      info.D[3] = default_ir_params_.p2;
      info.D[4] = default_ir_params_.k3;
      info.K[0] = info.P[0] = default_ir_params_.fx;
      info.K[2] = info.P[2] = default_ir_params_.cx;
      info.K[4] = info.P[5] = default_ir_params_.fy;
      info.K[5] = info.P[6] = default_ir_params_.cy;
      info.K[8] = info.P[10] = 1.0;
      
      info.R[0] = info.R[4] = info.R[8] = 1.0;
      return boost::make_shared<sensor_msgs::CameraInfo>(info);
    }
  }
  
  sensor_msgs::CameraInfo::Ptr Driver::getRGBCameraInfo(
    libfreenect2::Frame* frame, ros::Time stamp, std::string frame_id)
  {
    if (rgb_caminfo_manager_ && rgb_caminfo_manager_->isCalibrated()) {
      sensor_msgs::CameraInfo info = rgb_caminfo_manager_->getCameraInfo();
      info.header.stamp = stamp;
      info.header.frame_id = frame_id;
      return boost::make_shared<sensor_msgs::CameraInfo>(info);
    }
    else {
      sensor_msgs::CameraInfo info;
      info.width = frame->width;
      info.height = frame->height;
      info.header.stamp = stamp;
      info.header.frame_id = frame_id;
      info.D.resize(5, 0.0);
      info.K.assign(0.0);
      info.R.assign(0.0);
      info.P.assign(0.0);
      info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      
      info.K[0] = info.P[0] = default_rgb_params_.fx;
      info.K[2] = info.P[2] = default_rgb_params_.cx;
      info.K[4] = info.P[5] = default_rgb_params_.fy;
      info.K[5] = info.P[6] = default_rgb_params_.cy;
      info.K[8] = info.P[10] = 1.0;
      
      info.R[0] = info.R[4] = info.R[8] = 1.0;
      return boost::make_shared<sensor_msgs::CameraInfo>(info);
    }
  }

}

#include <pluginlib/class_list_macros.h>
typedef jsk_libfreenect2::Driver Driver;
PLUGINLIB_EXPORT_CLASS(Driver, nodelet::Nodelet);

