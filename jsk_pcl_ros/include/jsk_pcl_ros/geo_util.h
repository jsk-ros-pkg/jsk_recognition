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

#include <geometry_msgs/Polygon.h>

namespace jsk_pcl_ros
{
  void convertEigenVector(const Eigen::Vector3f& input,
                          Eigen::Vector3d& output);
  void convertEigenVector(const Eigen::Vector3d& input,
                          Eigen::Vector3f& output);
  
  // (infinite) line
  class Line
  {
  public:
    typedef boost::shared_ptr<Line> Ptr;
    Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin);
    virtual void getDirection(Eigen::Vector3f& output);
    virtual double distanceToPoint(const Eigen::Vector3f& from);
    virtual double distanceToPoint(const Eigen::Vector3f& from, Eigen::Vector3f& foot);
    virtual double distance(const Line& other);
    virtual void foot(const Eigen::Vector3f& point, Eigen::Vector3f& output);
    virtual double angle(const Line& other);
    virtual bool isParallel(const Line& other, double angle_threshold = 0.1);
  protected:
    Eigen::Vector3f direction_;
    Eigen::Vector3f origin_;
  private:
  };

  class Segment: public Line
  {
  public:
    typedef boost::shared_ptr<Segment> Ptr;
    Segment(const Eigen::Vector3f& from, const Eigen::Vector3f to);
    virtual void foot(const Eigen::Vector3f& point, Eigen::Vector3f& output);
    virtual double dividingRatio(const Eigen::Vector3f& point);
    virtual double distance(const Eigen::Vector3f& point);
    //virtual double distance(const Segment& other);
  protected:
    Eigen::Vector3f from_, to_;
  private:
  };

  class Plane
  {
  public:
    typedef boost::shared_ptr<Plane> Ptr;
    Plane(const std::vector<float>& coefficients);
    Plane(Eigen::Vector3f normal, double d);
    Plane(Eigen::Vector3f normal, Eigen::Vector3f p);
    virtual ~Plane();
    virtual Plane flip();
    
    virtual bool isSameDirection(const Plane& another);
    virtual bool isSameDirection(const Eigen::Vector3f& another_normal);
    virtual double signedDistanceToPoint(const Eigen::Vector4f p);
    virtual double distanceToPoint(const Eigen::Vector4f p);
    virtual double signedDistanceToPoint(const Eigen::Vector3f p);
    virtual double distanceToPoint(const Eigen::Vector3f p);
    
    virtual double distance(const Plane& another);
    virtual double angle(const Plane& another);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3d& output);
    virtual Eigen::Vector3f getNormal();
    virtual Plane transform(const Eigen::Affine3d& transform);
    virtual void toCoefficients(std::vector<float>& output);
    virtual std::vector<float> toCoefficients();
    virtual double getD();
  protected:
    Eigen::Vector3f normal_;
    double d_;
  private:
  };

  class ConvexPolygon: public Plane
  {
  public:
    typedef boost::shared_ptr<ConvexPolygon> Ptr;
    typedef Eigen::Vector3f Vertex;
            
    typedef std::vector<Eigen::Vector3f,
                        Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;
    // vertices should be CW
    ConvexPolygon(const Vertices& vertices);
    ConvexPolygon(const Vertices& vertices,
                  const std::vector<float>& coefficients);
    
    //virtual Polygon flip();
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3d& output);
    virtual void projectOnPlane(const Eigen::Vector3f& p, Eigen::Vector3f& output);
    // p should be a point on the plane
    virtual bool isInside(const Eigen::Vector3f& p);
    virtual ConvexPolygon flipConvex();
    virtual Eigen::Vector3f getCentroid();
    static ConvexPolygon fromROSMsg(const geometry_msgs::Polygon& polygon);
  protected:
    Vertices vertices_;
  private:
  };
  
}

#endif
