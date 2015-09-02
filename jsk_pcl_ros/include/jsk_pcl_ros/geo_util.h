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
#include "jsk_pcl_ros/random_util.h"

// Utitlity macros
inline void ROS_INFO_EIGEN_VECTOR3(const std::string& prefix,
                                   const Eigen::Vector3f& v) {
  ROS_INFO("%s: [%f, %f, %f]", prefix.c_str(), v[0], v[1], v[2]);
}

namespace jsk_pcl_ros
{
  typedef std::vector<Eigen::Vector3f,
                      Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;

  // Prototype definition
  class Plane;
  class Cube;
  
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

  /**
   * @brief
   * Compute PointCloud from Vertices
   */
  template<class PointT>
  typename pcl::PointCloud<PointT>::Ptr verticesToPointCloud(const Vertices& v)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    for (size_t i = 0; i < v.size(); i++) {
      PointT p;
      // Do not use pointFromVectorToXYZ in order not to depend on
      // pcl_conversion_util
      //pointFromVectorToXYZ<Eigen::Vector3f, PointT>(v[i], p);
      p.x = v[i][0];
      p.y = v[i][1];
      p.z = v[i][2];
      cloud->points.push_back(p);
    }
    return cloud;
  }

  /**
   * @brief
   * Compute Vertices from PointCloud
   */
  template<class PointT>
  Vertices pointCloudToVertices(const pcl::PointCloud<PointT>& cloud)
  {
    Vertices vs;
    for (size_t i = 0; i < cloud.points.size(); i++) {
      Eigen::Vector3f p(cloud.points[i].getVector3fMap());
      vs.push_back(p);
    }
    return vs;
  }

  // geoemtry classes
  
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
    virtual double distance(const Eigen::Vector3f& point, Eigen::Vector3f& foot_point) const;
    virtual bool intersect(Plane& plane, Eigen::Vector3f& point) const;
    //virtual double distance(const Segment& other);
    friend std::ostream& operator<<(std::ostream& os, const Segment& seg);
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
    virtual void project(const Eigen::Affine3f& pose, Eigen::Affine3f& output);
    virtual Eigen::Vector3f getNormal();
    virtual Eigen::Vector3f getPointOnPlane();
    virtual Plane transform(const Eigen::Affine3d& transform);
    virtual Plane transform(const Eigen::Affine3f& transform);
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
    virtual void clearTriangleDecompositionCache() {
      cached_triangles_.clear();
    }
    
    virtual Eigen::Vector3f getNormalFromVertices();
    virtual bool isTriangle();
    Eigen::Vector3f randomSampleLocalPoint(boost::mt19937& random_generator);
    virtual void getLocalMinMax(double& min_x, double& min_y,
                                double& max_x, double& max_y);
    template <class PointT>
    typename pcl::PointCloud<PointT>::Ptr samplePoints(double grid_size)
    {
      typename pcl::PointCloud<PointT>::Ptr
        ret (new pcl::PointCloud<PointT>);
      double min_x, min_y, max_x, max_y;
      getLocalMinMax(min_x, min_y, max_x, max_y);
      // ROS_INFO("min_x: %f", min_x);
      // ROS_INFO("min_y: %f", min_y);
      // ROS_INFO("max_x: %f", max_x);
      // ROS_INFO("max_y: %f", max_y);
      // Decompose into triangle first for optimization
      std::vector<Polygon::Ptr> triangles = decomposeToTriangles();

      for (double x = min_x; x < max_x; x += grid_size) {
        for (double y = min_y; y < max_y; y += grid_size) {
          Eigen::Vector3f candidate(x, y, 0);
          Eigen::Vector3f candidate_global = coordinates() * candidate;
          // check candidate is inside of the polygon or not
          bool insidep = false;
          for (size_t i = 0; i < triangles.size(); i++) {
            if (triangles[i]->isInside(candidate_global)) {
              insidep = true;
              break;
            }
          }
          if (insidep) {
            PointT p;
            p.x = candidate_global[0];
            p.y = candidate_global[1];
            p.z = candidate_global[2];
            p.normal_x = normal_[0];
            p.normal_y = normal_[1];
            p.normal_z = normal_[2];
            ret->points.push_back(p);
          }
        }
      }
      return ret;
    }

    /**
     * @brief
     * get all the edges as point of Segment.
     */
    std::vector<Segment::Ptr> edges() const;
    
    /**
     * @brief
     * Compute nearest point from p on this polygon.
     * 
     * This method first project p onth the polygon and
     * if the projected point is inside of polygon,
     * the projected point is the nearest point.
     * If not, distances between the point and edges are
     * computed and search the nearest point.
     *
     * In case of searching edges, it is achieved in brute-force
     * searching and if the number of edges is huge, it may take
     * a lot of time to compute.
     *
     * This method cannot be used for const instance because
     * triangle decomposition will change the cache in the instance.
     *
     * Distance between p and nearest point is stored in distance.
     */
    virtual Eigen::Vector3f nearestPoint(const Eigen::Vector3f& p,
                                         double& distance);
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
    /**
     * @brief
     * return true if p is inside of polygon.
     * p should be in global coordinates.
     */
    virtual bool isInside(const Eigen::Vector3f& p);
    size_t previousIndex(size_t i);
    size_t nextIndex(size_t i);
    
    static Polygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    static Polygon::Ptr fromROSMsgPtr(const geometry_msgs::Polygon& polygon);
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
      output.height = 1;
      output.width = output.points.size();
    }
    
  protected:
    Vertices vertices_;
    std::vector<Polygon::Ptr> cached_triangles_;
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
    virtual void projectOnPlane(const Eigen::Vector3f& p,
                                Eigen::Vector3f& output);
    virtual void projectOnPlane(const Eigen::Affine3f& p,
                                Eigen::Affine3f& output);
    virtual bool isProjectableInside(const Eigen::Vector3f& p);
    // p should be a point on the plane
    virtual ConvexPolygon flipConvex();
    virtual Eigen::Vector3f getCentroid();
    virtual Ptr magnify(const double scale_factor);
    virtual Ptr magnifyByDistance(const double distance);
    
    static ConvexPolygon fromROSMsg(const geometry_msgs::Polygon& polygon);
    static ConvexPolygon::Ptr fromROSMsgPtr(const geometry_msgs::Polygon& polygon);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold);
    bool distanceSmallerThan(
      const Eigen::Vector3f& p, double distance_threshold,
      double& output_distance);
    bool allEdgesLongerThan(double thr);
    double distanceFromVertices(const Eigen::Vector3f& p);
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
          Eigen::Vector3f v(convex_cloud->points[i].getVector3fMap());
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
   * i * resolution - 0.5 * resolution <= x < i * resolution + 0.5 * resolution
   * j * resolution - 0.5 * resolution <= y < j * resolution + 0.5 * resolution
   * 
   *
   */
  class GridPlane
  {
  public:
    typedef boost::shared_ptr<GridPlane> Ptr;
    typedef boost::tuple<int, int> IndexPair;
    typedef std::set<IndexPair> IndexPairSet;
    GridPlane(ConvexPolygon::Ptr plane, const double resolution);
    virtual ~GridPlane();
    virtual GridPlane::Ptr clone(); // shallow copy
    virtual size_t fillCellsFromPointCloud(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold);
    virtual size_t fillCellsFromPointCloud(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold,
      std::set<int>& non_plane_indices);
    virtual size_t fillCellsFromPointCloud(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      double distance_threshold,
      double normal_threshold,
      std::set<int>& non_plane_indices);
    virtual void fillCellsFromCube(Cube& cube);
    virtual double getResolution() { return resolution_; }
    virtual jsk_recognition_msgs::SimpleOccupancyGrid toROSMsg();
    /**
     * @brief
     * Construct GridPlane object from
     * jsk_recognition_msgs::SimpleOccupancyGrid.
     */
    static GridPlane fromROSMsg(
      const jsk_recognition_msgs::SimpleOccupancyGrid& rosmsg,
      const Eigen::Affine3f& offset);
    virtual bool isOccupied(const IndexPair& pair);
    
    /**
     * @brief
     * p should be local coordinate
     */
    virtual bool isOccupied(const Eigen::Vector3f& p);

    /**
     * @brief
     * p should be global coordinate
     */
    virtual bool isOccupiedGlobal(const Eigen::Vector3f& p);
    
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

    /**
     * @brief
     * Erode grid cells with specified number of pixels
     */
    virtual GridPlane::Ptr erode(int num);

    /**
     * @brief
     * return ConvexPolygon pointer of this instance.
     */
    virtual ConvexPolygon::Ptr getPolygon() { return convex_; }
    
    /**
     * @brief
     * Dilate grid cells with specified number of pixels
     */
    virtual GridPlane::Ptr dilate(int num);
  protected:
    ConvexPolygon::Ptr convex_;
    IndexPairSet cells_;
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
    Cube(const Eigen::Vector3f& pos, const Eigen::Quaternionf& rot,
         const Eigen::Vector3f& dimensions);
    Cube(const Eigen::Vector3f& pos, // centroid
         const Line& line_a, const Line& line_b, const Line& line_c);
    virtual ~Cube();
    std::vector<Segment::Ptr> edges();
    ConvexPolygon::Ptr intersectConvexPolygon(Plane& plane);
    std::vector<double> getDimensions() const { return dimensions_; };
    void setDimensions(const std::vector<double>& new_dimensions) {
      dimensions_[0] = new_dimensions[0];
      dimensions_[1] = new_dimensions[1];
      dimensions_[2] = new_dimensions[2];
    }
    jsk_recognition_msgs::BoundingBox toROSMsg();

    /**
     * @brief
     * returns vertices as an array of Eigen::Vectro3f.
     * The order of the vertices is:
     * [1, 1, 1], [-1, 1, 1], [-1, -1, 1], [1, -1, 1],
     * [1, 1, -1], [-1, 1, -1], [-1, -1, -1], [1, -1, -1].
     */
    Vertices vertices();
    
    /**
     * @brief
     * returns all the 6 faces as Polygon::Ptr.
     * TODO: is it should be ConvexPolygon?
     */
    std::vector<Polygon::Ptr> faces();

    /**
     * @brief
     * compute minimum distance from point p to cube surface.
     *
     * Distance computation depends on Polygon::nearestPoint and
     * this methods just searches a face which resutnrs the smallest
     * distance.
     */
    virtual Eigen::Vector3f nearestPoint(const Eigen::Vector3f& p,
                                         double& distance);
  protected:
    Eigen::Vector3f pos_;
    Eigen::Quaternionf rot_;
    std::vector<double> dimensions_;

    /**
     * @brief
     * A helper method to build polygon from 4 vertices.
     */
    virtual Polygon::Ptr buildFace(const Eigen::Vector3f v0,
                                   const Eigen::Vector3f v1,
                                   const Eigen::Vector3f v2,
                                   const Eigen::Vector3f v3);

    /**
     * @brief
     * A helper method to build vertex from x-y-z relatiev coordinates.
     */
    virtual Eigen::Vector3f buildVertex(double i, double j, double k);
    
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
    virtual double getRadius() { return radius_; }
  protected:
    Eigen::Vector3f point_;
    Eigen::Vector3f direction_;
    double radius_;
  private:
    
  };
  
}

#endif
