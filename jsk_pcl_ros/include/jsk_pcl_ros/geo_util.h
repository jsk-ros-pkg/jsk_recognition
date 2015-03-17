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

#ifndef JSK_PCL_ROS_GEO_UTIL_H_
#define JSK_PCL_ROS_GEO_UTIL_H_
//#define BOOST_PARAMETER_MAX_ARITY 7 

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Polygon.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/SimpleOccupancyGrid.h>
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
#include <visualization_msgs/Marker.h>
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
  typedef boost::tuple<size_t, size_t> PointIndexPair;
  
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
    Plane(const boost::array<float, 4>& coefficients);
    Plane(Eigen::Vector3f normal, double d);
    Plane(Eigen::Vector3f normal, Eigen::Vector3f p);
    virtual ~Plane();
    virtual Plane flip();
    virtual Plane::Ptr faceToOrigin();
    virtual bool isSameDirection(const Plane& another);
    virtual bool isSameDirection(const Eigen::Vector3f& another_normal);
    virtual double signedDistanceToPoint(const Eigen::Vector4f p);
    virtual double distanceToPoint(const Eigen::Vector4f p);
    virtual double signedDistanceToPoint(const Eigen::Vector3f p);
    virtual double distanceToPoint(const Eigen::Vector3f p);
    virtual double distance(const Plane& another);
    virtual double angle(const Plane& another);
    virtual double angle(const Eigen::Vector3f& vector);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3d& output);
    virtual void project(const Eigen::Vector3d& p, Eigen::Vector3f& output);
    virtual void project(const Eigen::Vector3f& p, Eigen::Vector3d& output);
    virtual Eigen::Vector3f getNormal();
    virtual Eigen::Vector3f getPointOnPlane();
    virtual Plane transform(const Eigen::Affine3d& transform);
    virtual void toCoefficients(std::vector<float>& output);
    virtual std::vector<float> toCoefficients();
    virtual double getD();
    virtual Eigen::Affine3f coordinates();
  protected:
    virtual void initializeCoordinates();
    Eigen::Vector3f normal_;
    double d_;
    Eigen::Affine3f plane_coordinates_;
  private:
  };

  std::vector<Plane::Ptr> convertToPlanes(
    std::vector<pcl::ModelCoefficients::Ptr>);

  class Polygon: public Plane
  {
  public:
    typedef boost::shared_ptr<Polygon> Ptr;
    typedef boost::tuple<Ptr, Ptr> PtrPair;
    Polygon(const Vertices& vertices);
    Polygon(const Vertices& vertices,
            const std::vector<float>& coefficients);
    virtual ~Polygon();
    virtual std::vector<Polygon::Ptr> decomposeToTriangles();
    virtual bool isTriangle();
    virtual size_t getNumVertices();
    virtual size_t getFarestPointIndex(const Eigen::Vector3f& O);
    virtual Eigen::Vector3f directionAtPoint(size_t i);
    virtual Eigen::Vector3f getVertex(size_t i);
    virtual PointIndexPair getNeighborIndex(size_t index);
    virtual Vertices getVertices() { return vertices_; };
    virtual double area();
    virtual bool isPossibleToRemoveTriangleAtIndex(
      size_t index,
      const Eigen::Vector3f& direction);
    virtual PtrPair separatePolygon(size_t index);
    virtual bool isInside(const Eigen::Vector3f& p);
    size_t previousIndex(size_t i);
    size_t nextIndex(size_t i);
    static Polygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    static Polygon createPolygonWithSkip(const Vertices& vertices);
    virtual bool isConvex();
    virtual Eigen::Vector3f centroid();
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

  protected:
    Vertices vertices_;
  private:
    
  };
  
  
  class ConvexPolygon: public Polygon
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
    // p should be a point on the plane
    virtual ConvexPolygon flipConvex();
    virtual Eigen::Vector3f getCentroid();
    virtual Ptr magnify(const double scale_factor);
    virtual Ptr magnifyByDistance(const double distance);
        
    static ConvexPolygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold,
      double& output_distance);
    bool allEdgesLongerThan(double thr);
    geometry_msgs::Polygon toROSMsg();
  protected:
    
  private:
  };

  template<class PointT>
  ConvexPolygon::Ptr convexFromCoefficientsAndInliers(
    const typename pcl::PointCloud<PointT>::Ptr cloud,
    const pcl::PointIndices::Ptr inliers,
    const pcl::ModelCoefficients::Ptr coefficients) {
    typedef typename pcl::PointCloud<PointT> POINTCLOUD;
    typename POINTCLOUD::Ptr projected_cloud(new pcl::PointCloud<PointT>);
    // check inliers has enough points
    if (inliers->indices.size() == 0) {
      return ConvexPolygon::Ptr();
    }
    // project inliers based on coefficients
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

  /**
   * @brief
   * Grid based representation of planar region.
   *
   * Each cell represents a square region as belows:
   *        +--------+
   *        |        |
   *        |   +    |
   *        |        |
   *        +--------+
   *
   * The width and height of the cell is equivalent to resolution_,
   * and the value of cells_ represents a center point.
   * (i, j) means rectanglar region of (x, y) which satisfies followings:
   * i * resolution - 0.5 * resolution < x <= i * resolution + 0.5 * resolution
   * j * resolution - 0.5 * resolution < y <= j * resolution + 0.5 * resolution
   * 
   *
   */
  class GridPlane
  {
  public:
    typedef boost::shared_ptr<GridPlane> Ptr;
    typedef boost::tuple<int, int> IndexPair;
    GridPlane(ConvexPolygon::Ptr plane, const double resolution);
    virtual ~GridPlane();
    virtual void fillCellsFromPointCloud(
      const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold);
    virtual double getResolution() { return resolution_; }
    virtual jsk_recognition_msgs::SimpleOccupancyGrid toROSMsg();
    
    /**
     * @brief
     * Project 3-D point to GridPlane::IndexPair.
     * p should be represented in local coordinates.
     */
    virtual IndexPair projectLocalPointAsIndexPair(const Eigen::Vector3f& p);

    /**
     * @brief
     * Unproject GridPlane::IndexPair to 3-D local point.
     */
    virtual Eigen::Vector3f unprojectIndexPairAsLocalPoint(const IndexPair& pair);

    /**
     * @brief
     * Unproject GridPlane::IndexPair to 3-D global point.
     */
    virtual Eigen::Vector3f unprojectIndexPairAsGlobalPoint(const IndexPair& pair);

    /**
     * @brief
     * Add IndexPair to this instance.
     */
    virtual void addIndexPair(IndexPair pair);
  protected:
    ConvexPolygon::Ptr convex_;
    std::set<IndexPair> cells_;
    double resolution_;
  private:
    
  };

  
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
    jsk_recognition_msgs::BoundingBox toROSMsg();
  protected:
    Eigen::Vector3f pos_;
    Eigen::Quaternionf rot_;
    std::vector<double> dimensions_;
  private:
    
  };

  template <class PointT>
  jsk_recognition_msgs::BoundingBox boundingBoxFromPointCloud(const pcl::PointCloud<PointT>& cloud)
  {
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<PointT>(cloud, minpt, maxpt);
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.dimensions.x = std::abs(minpt[0] - maxpt[0]);
    bbox.dimensions.y = std::abs(minpt[1] - maxpt[1]);
    bbox.dimensions.z = std::abs(minpt[2] - maxpt[2]);
    bbox.pose.position.x = (minpt[0] + maxpt[0]) / 2.0;
    bbox.pose.position.y = (minpt[1] + maxpt[1]) / 2.0;
    bbox.pose.position.z = (minpt[2] + maxpt[2]) / 2.0;
    bbox.pose.orientation.w = 1.0;
    return bbox;
  }

  class Cylinder                // infinite
  {
  public:
    typedef boost::shared_ptr<Cylinder> Ptr;
    Cylinder(Eigen::Vector3f point, Eigen::Vector3f direction, double radius);

    virtual void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const double threshold,
                                  pcl::PointIndices& output);
    virtual void estimateCenterAndHeight(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                         const pcl::PointIndices& indices,
                                         Eigen::Vector3f& center,
                                         double& height);
    virtual void toMarker(visualization_msgs::Marker& marker,
                          const Eigen::Vector3f& center,
                          const Eigen::Vector3f& uz,
                          const double height);
    virtual Eigen::Vector3f getDirection();
  protected:
    Eigen::Vector3f point_;
    Eigen::Vector3f direction_;
    double radius_;
  private:
    
  };
  
}

#endif
