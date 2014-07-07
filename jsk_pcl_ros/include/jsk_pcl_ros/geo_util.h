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

#ifndef JSK_PCL_ROS_GEO_UTIL_H_
#define JSK_PCL_ROS_GEO_UTIL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <vector>

#include <boost/shared_ptr.hpp>

namespace jsk_pcl_ros
{
  // (infinite) line
  class Line
  {
  public:
    typedef boost::shared_ptr<Line> Ptr;
    Line(const Eigen::Vector3d& direction, const Eigen::Vector3d& origin);
    virtual void getDirection(Eigen::Vector3d& output);
    virtual double distanceToPoint(const Eigen::Vector3d& from);
    virtual double distanceToPoint(const Eigen::Vector3d& from, Eigen::Vector3d& foot);
    virtual void foot(const Eigen::Vector3d& point, Eigen::Vector3d& output);
  protected:
    Eigen::Vector3d direction_;
    Eigen::Vector3d origin_;
  private:
  };

  class Segment: public Line
  {
  public:
    typedef boost::shared_ptr<Segment> Ptr;
    Segment(const Eigen::Vector3d& from, const Eigen::Vector3d to);
    virtual void foot(const Eigen::Vector3d& point, Eigen::Vector3d& output);
    virtual double dividingRatio(const Eigen::Vector3d& point);
  protected:
    Eigen::Vector3d from_, to_;
  private:
  };

  class Plane
  {
  public:
    typedef boost::shared_ptr<Plane> Ptr;
    Plane(const std::vector<float>& coefficients);
    Plane(Eigen::Vector3d normal, double d);
    Plane(Eigen::Vector3d normal, Eigen::Vector3d p);
    virtual ~Plane();
    virtual Plane flip();
    
    virtual bool isSameDirection(const Plane& another);
    virtual bool isSameDirection(const Eigen::Vector3d& another_normal);
    virtual double signedDistanceToPoint(const Eigen::Vector4f p);
    virtual double distanceToPoint(const Eigen::Vector4f p);
    virtual double signedDistanceToPoint(const Eigen::Vector3d p);
    virtual double distanceToPoint(const Eigen::Vector3d p);
    
    virtual double distance(const Plane& another);
    virtual double angle(const Plane& another);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual Eigen::Vector3d getNormal();
    virtual Plane transform(const Eigen::Affine3d& transform);
    virtual void toCoefficients(std::vector<float>& output);
    virtual std::vector<float> toCoefficients();
  protected:
    Eigen::Vector3d normal_;
    double d_;
  private:
  };

  class ConvexPolygon: public Plane
  {
  public:
    typedef boost::shared_ptr<ConvexPolygon> Ptr;
    typedef std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> > Vertices;
    // vertices should be CW
    ConvexPolygon(const Vertices& vertices);
    ConvexPolygon(const Vertices& vertices,
                  const std::vector<float>& coefficients);
    //virtual Polygon flip();
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual void projectOnPlane(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    // p should be a point on the plane
    virtual bool isInside(const Eigen::Vector3d& p);
    virtual ConvexPolygon flipConvex();
    virtual Eigen::Vector3d getCentroid();
  protected:
    Vertices vertices_;
  private:
  };
  
}

#endif
