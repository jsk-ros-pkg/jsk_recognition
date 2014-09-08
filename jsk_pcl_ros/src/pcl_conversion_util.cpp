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

#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace pcl_conversions
{
  
  void toPCL(const geometry_msgs::Point32& msg,
             pcl::PointXYZRGB& p)
  {
    p.x = msg.x;
    p.y = msg.y;
    p.z = msg.z;
  }

  void toPCL(const geometry_msgs::Point32& msg,
             pcl::PointXYZ& p)
  {
    p.x = msg.x;
    p.y = msg.y;
    p.z = msg.z;
  }

  
  void fromPCL(const pcl::PointXYZRGB& p,
               geometry_msgs::Point32& msg)
  {
    msg.x = p.x;
    msg.y = p.y;
    msg.z = p.z;
  }

  void fromPCL(const pcl::PointXYZ& p,
               geometry_msgs::Point32& msg)
  {
    msg.x = p.x;
    msg.y = p.y;
    msg.z = p.z;
  }

  void fromMSGToEigen(const geometry_msgs::Point32& msg,
                      Eigen::Vector3d& p)
  {
    p[0] = msg.x;
    p[1] = msg.y;
    p[2] = msg.z;
  }

  void fromMSGToEigen(const geometry_msgs::Point32& msg,
                      Eigen::Vector3f& p)
  {
    p[0] = msg.x;
    p[1] = msg.y;
    p[2] = msg.z;
  }

  void fromPCLToEigen(const pcl::PointXYZRGB& p,
                      Eigen::Vector3d& output)
  {
    output[0] = p.x;
    output[1] = p.y;
    output[2] = p.z;
  }

  void fromEigenToMSG(const Eigen::Vector3f& p,
                      geometry_msgs::Point& msg)
  {
    msg.x = p[0];
    msg.y = p[1];
    msg.z = p[2];
  }
  
}
