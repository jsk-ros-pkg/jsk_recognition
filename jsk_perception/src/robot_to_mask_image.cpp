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

#include "jsk_perception/robot_to_mask_image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void RobotToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    initSelfMask(*pnh_);
    pnh_->param("max_robot_dist", max_robot_dist_, 10.0);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pub_camera_info_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output/info", 1);
    onInitPostProcess();
  }

  void RobotToMaskImage::subscribe()
  {
    sub_ = pnh_->subscribe("input/camera_info", 1,
                                &RobotToMaskImage::infoCallback, this);
  }

  void RobotToMaskImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void RobotToMaskImage::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    vital_checker_->poke();
    if (info_msg) {
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(info_msg);

      sensor_msgs::RegionOfInterest roi = info_msg->roi;
      int height = roi.height ? roi.height : info_msg->height;
      int width = roi.width ? roi.width : info_msg->width;
      if (info_msg->binning_y > 0) {
        height /= info_msg->binning_y;
      }
      if (info_msg->binning_x > 0) {
        width /= info_msg->binning_x;
      }

      cv::Mat mask_image = cv::Mat::zeros(height, width, CV_8UC1);
      self_mask_->assumeFrame(info_msg->header);
      for (int u = 0; u < width; u++) {
        for (int v = 0; v < height; v++) {
          // project to 3d
          cv::Point uv(u, v);
          cv::Point3d p = model.projectPixelTo3dRay(uv);
          // check intersection with robot
          // ROS_INFO_STREAM("uv( " << u << " , " << v << " )  intersection: " << self_mask_->getMaskIntersection(p.x * max_robot_dist_, p.y * max_robot_dist_, p.z * max_robot_dist_));
          if (self_mask_->getMaskIntersection(p.x * max_robot_dist_, p.y * max_robot_dist_, p.z * max_robot_dist_) == robot_self_filter::OUTSIDE) {
            mask_image.data[mask_image.step * v + mask_image.elemSize() * u] = 255;
          }
        }
      }
      sensor_msgs::CameraInfo camera_info_msg;
      camera_info_msg.header = info_msg->header;
      camera_info_msg.width = width;
      camera_info_msg.height = height;
      camera_info_msg.distortion_model = info_msg->distortion_model;
      camera_info_msg.roi.x_offset = 0;
      camera_info_msg.roi.y_offset = 0;
      camera_info_msg.roi.width = width;
      camera_info_msg.roi.height = height;
      camera_info_msg.roi.do_rectify = info_msg->roi.do_rectify;
      cv::Matx33d K = model.intrinsicMatrix();
      for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 3; ++j) camera_info_msg.K[i * 3 + j] = K(i, j);
      cv::Matx33d R = model.rotationMatrix();
      for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 3; ++j) camera_info_msg.R[i * 3 + j] = R(i, j);
      cv::Matx34d P = model.projectionMatrix();
      for (size_t i = 0; i < 3; ++i) for (size_t j = 0; j < 4; ++j) camera_info_msg.P[i * 4 + j] = P(i, j);
      cv::Mat_<double> D = model.distortionCoeffs();
      camera_info_msg.D.resize(D.rows * D.cols);
      for (size_t i = 0; i < D.rows; ++i) for (size_t j = 0; j < D.cols; ++j) camera_info_msg.D[i * D.cols + j] = D(i, j);
      pub_.publish(cv_bridge::CvImage(info_msg->header,
                                      sensor_msgs::image_encodings::MONO8,
                                      mask_image).toImageMsg());
      pub_camera_info_.publish(camera_info_msg);
    }
  }

  void RobotToMaskImage::initSelfMask(const ros::NodeHandle& pnh)
  {
    // genearte urdf model
    double default_padding, default_scale;
    pnh.param("self_see_default_padding", default_padding, 0.01);
    pnh.param("self_see_default_scale", default_scale, 1.0);
    std::vector<robot_self_filter::LinkInfo> links;

    if(!pnh.hasParam("self_see_links")) {
      ROS_WARN("No links specified for self filtering.");
    } else {
      XmlRpc::XmlRpcValue ssl_vals;;
      pnh.getParam("self_see_links", ssl_vals);
      if(ssl_vals.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("Self see links need to be an array");
      } else {
        if(ssl_vals.size() == 0) {
          ROS_WARN("No values in self see links array");
        } else {
          for(int i = 0; i < ssl_vals.size(); i++) {
            robot_self_filter::LinkInfo li;
            if(ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
              ROS_WARN("Self see links entry %d is not a structure.  Stopping processing of self see links",i);
              break;
            }
            if(!ssl_vals[i].hasMember("name")) {
              ROS_WARN("Self see links entry %d has no name.  Stopping processing of self see links",i);
              break;
            }
            li.name = std::string(ssl_vals[i]["name"]);
            if(!ssl_vals[i].hasMember("padding")) {
              ROS_DEBUG("Self see links entry %d has no padding.  Assuming default padding of %g",i,default_padding);
              li.padding = default_padding;
            } else {
              li.padding = ssl_vals[i]["padding"];
            }
            if(!ssl_vals[i].hasMember("scale")) {
              ROS_DEBUG("Self see links entry %d has no scale.  Assuming default scale of %g",i,default_scale);
              li.scale = default_scale;
            } else {
              li.scale = ssl_vals[i]["scale"];
            }
            links.push_back(li);
          }
        }
      }
    }
    self_mask_ = boost::shared_ptr<robot_self_filter::SelfMask<pcl::PointXYZ> >(new robot_self_filter::SelfMask<pcl::PointXYZ>(tf_listener_, links));
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS (jsk_perception::RobotToMaskImage, nodelet::Nodelet);
