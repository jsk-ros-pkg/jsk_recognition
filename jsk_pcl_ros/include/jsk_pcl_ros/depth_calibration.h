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

#ifndef JSK_PCL_ROS_DEPTH_CALIBRATION_H_
#define JSK_PCL_ROS_DEPTH_CALIBRATION_H_

#include "jsk_pcl_ros/diagnostic_nodelet.h"
#include "jsk_pcl_ros/DepthCalibrationParameter.h"

namespace jsk_pcl_ros
{

  // calibration:
  // z' = C_2(u, v) z^2 + C_1(u, v) z + C_0(u, v)
  // C_i(u, v) = au + bv + c
  class DepthCalibration: public DiagnosticNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    DepthCalibration(): DiagnosticNodelet("DepthCalibration") { }
  protected:
    virtual void onInit();
    virtual void calibrate(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual inline double applyModel(double z, int u, int v) {
      double z2 = z * z;
      double c2 = coefficients2_[0] * u + coefficients2_[1] * v + coefficients2_[2];
      double c1 = coefficients1_[0] * u + coefficients1_[1] * v + coefficients1_[2];
      double c0 = coefficients0_[0] * u + coefficients0_[1] * v + coefficients0_[2];
      return c2 * z2 + c1 * z + c0;
    }
    virtual bool setCalibrationParameter(
      DepthCalibrationParameter::Request& req,
      DepthCalibrationParameter::Response& res);
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::ServiceServer set_calibration_parameter_srv_;
    boost::mutex mutex_;
    // parameters
    std::vector<double> coefficients2_;
    std::vector<double> coefficients1_;
    std::vector<double> coefficients0_;
  private:
  };
}

#endif
