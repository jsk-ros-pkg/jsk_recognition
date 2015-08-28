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

#ifndef JSK_PCL_ROS_CAMERA_DEPTH_SENSOR_H_
#define JSK_PCL_ROS_CAMERA_DEPTH_SENSOR_H_

#include "jsk_pcl_ros/sensor_model/pointcloud_sensor_model.h"
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

namespace jsk_pcl_ros
{
  class CameraDepthSensor: public PointCloudSensorModel
  {
  public:
    typedef boost::shared_ptr<CameraDepthSensor> Ptr;
    CameraDepthSensor() {}
    
    virtual void setCameraInfo(const sensor_msgs::CameraInfo& info)
    {
      model_.fromCameraInfo(info);
    }

    /**
     * @brief
     * Return the expected number of points according to distance and area.
     * it is calculated according to:
     * f^2\frac{s}{r^2}
     **/
    virtual double expectedPointCloudNum(double distance, double area) const
    {
      double f = model_.fx();
      return f*f / (distance*distance) * area;
    }
  protected:
    image_geometry::PinholeCameraModel model_;
    
  private:
    
  };
}

#endif
