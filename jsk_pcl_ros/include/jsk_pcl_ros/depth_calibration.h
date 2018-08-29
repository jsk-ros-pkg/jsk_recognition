// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_DEPTH_CALIBRATION_H_
#define JSK_PCL_ROS_DEPTH_CALIBRATION_H_

#include "pcl_ros/pcl_nodelet.h"
#include "jsk_topic_tools/diagnostic_nodelet.h"
#include "jsk_recognition_msgs/SetDepthCalibrationParameter.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace jsk_pcl_ros
{

  // calibration:
  // z' = C_2(u, v) z^2 + C_1(u, v) z + C_0(u, v)
  // C_i(u, v) = au + bv + c
  class DepthCalibration: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::Image,
        sensor_msgs::CameraInfo> SyncPolicy;

    DepthCalibration(): DiagnosticNodelet("DepthCalibration") { }
  protected:
    virtual void onInit();
    virtual void calibrate(
      const sensor_msgs::Image::ConstPtr& msg,
      const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void printModel();
    virtual inline double applyModel(double z, int u, int v, double cu, double cv) {
      double z2 = z * z;
      double uu, vv;
      if (use_abs_) {
        uu = uv_scale_ * std::abs(u - cu);
        vv = uv_scale_ * std::abs(v - cv);
      }
      else {
        uu = uv_scale_ * u;
        vv = uv_scale_ * v;
      }
      double c2 = coefficients2_[0] * uu * uu + coefficients2_[1] * uu +
        coefficients2_[2] * vv * vv + coefficients2_[3] * vv + 
        coefficients2_[4];
      double c1 = coefficients1_[0] * uu * uu + coefficients1_[1] * uu +
        coefficients1_[2] * vv * vv + coefficients1_[3] * vv + 
        coefficients1_[4];
      double c0 = coefficients0_[0] * uu * uu + coefficients0_[1] * uu +
        coefficients0_[2] * vv * vv + coefficients0_[3] * vv + 
        coefficients0_[4];
      return c2 * z2 + c1 * z + c0;
    }
    
    virtual bool setCalibrationParameter(
      jsk_recognition_msgs::SetDepthCalibrationParameter::Request& req,
      jsk_recognition_msgs::SetDepthCalibrationParameter::Response& res);
    message_filters::Subscriber<sensor_msgs::Image> sub_input_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    ros::Publisher pub_;
    ros::ServiceServer set_calibration_parameter_srv_;
    boost::mutex mutex_;
    
    // parameters
    bool use_abs_;
    double uv_scale_;
    std::vector<double> coefficients2_;
    std::vector<double> coefficients1_;
    std::vector<double> coefficients0_;
  private:
  };
}

#endif
