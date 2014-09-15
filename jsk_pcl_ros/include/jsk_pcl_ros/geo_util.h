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
#define BOOST_PARAMETER_MAX_ARITY 7 

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Polygon.h>
#include <jsk_pcl_ros/BoundingBox.h>
#include <boost/tuple/tuple.hpp>

////////////////////////////////////////////////////////
// PCL headers
////////////////////////////////////////////////////////
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

#include "jsk_pcl_ros/pcl_util.h"

namespace jsk_pcl_ros
{
  typedef std::vector<Eigen::Vector3f,
                      Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;
  
  ////////////////////////////////////////////////////////
  // compute quaternion from 3 unit vector
  // these vector should be normalized and diagonal
  ////////////////////////////////////////////////////////
  Eigen::Quaternionf rotFrom3Axis(const Eigen::Vector3f& ex,
                                  const Eigen::Vector3f& ey,
                                  const Eigen::Vector3f& ez);
  typedef Eigen::Vector3f Point;
  typedef Eigen::Vector3f Vertex;
  typedef std::vector<Eigen::Vector3f,
                      Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;
  typedef boost::tuple<Point, Point> PointPair;
  // (infinite) line
  class Line
  {
  public:
    typedef boost::shared_ptr<Line> Ptr;
    Line(const Eigen::Vector3f& direction, const Eigen::Vector3f& origin);
    virtual void getDirection(Eigen::Vector3f& output) const;
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

  class Segment: public Line
  {
  public:
    typedef boost::shared_ptr<Segment> Ptr;
    Segment(const Eigen::Vector3f& from, const Eigen::Vector3f to);
    virtual void foot(const Eigen::Vector3f& point, Eigen::Vector3f& output) const;
    virtual double dividingRatio(const Eigen::Vector3f& point) const;
    virtual double distance(const Eigen::Vector3f& point) const;
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
    virtual bool isProjectableInside(const Eigen::Vector3f& p);
    virtual Vertices getVertices() { return vertices_; };
    // p should be a point on the plane
    virtual bool isInside(const Eigen::Vector3f& p);
    virtual ConvexPolygon flipConvex();
    virtual Eigen::Vector3f getCentroid();
    virtual Ptr magnify(const double scale_factor);
    
    template<class PointT> void boundariesToPointCloud(
      pcl::PointCloud<PointT>& output) {
      output.points.resize(vertices_.size());
      for (size_t i = 0; i < vertices_.size(); i++) {
        Eigen::Vector3f v = vertices_[i];
        PointT p;
        p.x = v[0]; p.y = v[1]; p.z = v[2];
        output.points[i] = p;
      }
    }
    
    static ConvexPolygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold,
      double& output_distance);
    double area();
    bool allEdgesLongerThan(double thr);
    geometry_msgs::Polygon toROSMsg();
  protected:
    Vertices vertices_;
  private:
  };

  template<class PointT>
  ConvexPolygon::Ptr convexFromCoefficientsAndInliers(
    const typename pcl::PointCloud<PointT>::Ptr cloud,
    const pcl::PointIndices::Ptr inliers,
    const pcl::ModelCoefficients::Ptr coefficients) {
    typedef typename pcl::PointCloud<PointT> POINTCLOUD;
    typename POINTCLOUD::Ptr projected_cloud(new pcl::PointCloud<PointT>);
    // project inliers based on coefficients
    // std::cout << "inliers: " << inliers->indices.size() << std::endl;
    // std::cout << "coefficients: " << *coefficients << std::endl;
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.setIndices(inliers);
    proj.filter(*projected_cloud);
    // compute convex with giant mutex
    {
      boost::mutex::scoped_lock lock(global_chull_mutex);
      typename POINTCLOUD::Ptr convex_cloud(new pcl::PointCloud<PointT>);
      pcl::ConvexHull<PointT> chull;
      chull.setDimension(2);
      chull.setInputCloud (projected_cloud);
      chull.reconstruct (*convex_cloud);
      if (convex_cloud->points.size() > 0) {
        // convert pointcloud to vertices
        Vertices vs;
        for (size_t i = 0; i < convex_cloud->points.size(); i++) {
          Eigen::Vector3f v = convex_cloud->points[i].getVector3fMap();
          vs.push_back(v);
        }
        return ConvexPolygon::Ptr(new ConvexPolygon(vs));
      }
      else {
        return ConvexPolygon::Ptr();
      }
    }
  }
  
  class Cube
  {
  public:
    typedef boost::shared_ptr<Cube> Ptr;
    Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot);
    Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
         const std::vector<double>& dimensions);
    Cube(const Eigen::Vector3f& pos, // centroid
         const Line& line_a, const Line& line_b, const Line& line_c);
    virtual ~Cube();
    std::vector<double> getDimensions() const { return dimensions_; };
    void setDimensions(const std::vector<double>& new_dimensions) {
      dimensions_[0] = new_dimensions[0];
      dimensions_[1] = new_dimensions[1];
      dimensions_[2] = new_dimensions[2];
    }
    BoundingBox toROSMsg();
  protected:
    Eigen::Vector3f pos_;
    Eigen::Quaternionf rot_;
    std::vector<double> dimensions_;
  private:
    
  };
  
}

#endif
