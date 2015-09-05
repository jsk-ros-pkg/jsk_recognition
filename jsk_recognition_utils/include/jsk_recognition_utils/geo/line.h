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

#ifndef JSK_RECOGNITION_UTILS_GEO_LINE_H_
#define JSK_RECOGNITION_UTILS_GEO_LINE_H_

#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include "jsk_recognition_utils/types.h"

namespace jsk_recognition_utils
{
  // (infinite) line
  class Line
  {
  public:
    typedef boost::shared_ptr<Line> Ptr;
    Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin);
    virtual void getDirection(Eigen::Vector3f& output) const;
    virtual Eigen::Vector3f getDirection() const;
    virtual void getOrigin(Eigen::Vector3f& output) const;
    virtual double distanceToPoint(const Eigen::Vector3f& from) const;
    virtual double distanceToPoint(const Eigen::Vector3f& from, Eigen::Vector3f& foot) const;
    virtual double distance(const Line& other) const;
    virtual void foot(const Eigen::Vector3f& point, Eigen::Vector3f& output) const;
    virtual double angle(const Line& other) const;
    virtual bool isParallel(const Line& other, double angle_threshold = 0.1) const;
    virtual bool isPerpendicular(const Line& other, double angle_threshold = 0.1) const;
    virtual Ptr midLine(const Line& other) const;
    virtual Ptr parallelLineOnAPoint(const Eigen::Vector3f& p) const;
    virtual PointPair findEndPoints(const Vertices& points) const;
    virtual double computeAlpha(const Point& p) const;
    virtual bool isSameDirection(const Line& other) const;
    virtual Line::Ptr flip();
    virtual void parallelLineNormal(const Line& other, Eigen::Vector3f& output) const;
    static Ptr fromCoefficients(const std::vector<float>& coefficients);
    virtual void print();
    virtual void point(double alpha, Eigen::Vector3f& ouptut);
  protected:
    Eigen::Vector3f direction_;
    Eigen::Vector3f origin_;
  private:
  };

}

#endif
