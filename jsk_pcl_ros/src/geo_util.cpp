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

#include "jsk_pcl_ros/geo_util.h"

namespace jsk_pcl_ros
{
  Plane::Plane(const std::vector<float>& coefficients)
  {
    normal_ = Eigen::Vector3d(coefficients[0], coefficients[1], coefficients[2]);
    d_ = coefficients[3] / normal_.norm();
    normal_.normalize();
  }

  Plane::Plane(Eigen::Vector3d normal, double d) :
    normal_(normal), d_(d)
  {
    
  }

  Plane::Plane(Eigen::Vector3d normal, Eigen::Vector3d p) :
    normal_(normal), d_(- normal.dot(p))
  {
    
  }
          
  
  Plane::~Plane()
  {

  }

  Plane Plane::flip()
  {
    return Plane(- normal_, - d_);
  }

  bool Plane::isSameDirection(const Plane& another)
  {
    return normal_.dot(another.normal_) > 0;
  }

  double Plane::signedDistanceToPoint(const Eigen::Vector3d p)
  {
    return (normal_.dot(p) + d_);
  }
  
  double Plane::signedDistanceToPoint(const Eigen::Vector4f p)
  {
    return signedDistanceToPoint(Eigen::Vector3d(p[0], p[1], p[2]));
  }
  
  double Plane::distanceToPoint(const Eigen::Vector4f p)
  {
    return fabs(signedDistanceToPoint(p));
  }

  double Plane::distanceToPoint(const Eigen::Vector3d p)
  {
    return fabs(signedDistanceToPoint(p));
  }
  
  double Plane::distance(const Plane& another)
  {
    return fabs(fabs(d_) - fabs(another.d_));
  }

  double Plane::angle(const Plane& another)
  {
    return acos(normal_.dot(another.normal_));
  }

  void Plane::project(const Eigen::Vector3d p, Eigen::Vector3d& output)
  {
    double alpha = - p.dot(normal_);
    output = p + alpha * normal_;
  }
  
}
