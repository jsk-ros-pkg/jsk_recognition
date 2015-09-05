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

#ifndef JSK_RECOGNITION_UTILS_GEO_UTIL_H_
#define JSK_RECOGNITION_UTILS_GEO_UTIL_H_
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

#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/random_util.h"

#include <jsk_recognition_msgs/PolygonArray.h>
#include "jsk_recognition_utils/sensor_model/camera_depth_sensor.h"
#include "jsk_recognition_utils/types.h"
#include "jsk_recognition_utils/geo/line.h"
#include "jsk_recognition_utils/geo/segment.h"
#include "jsk_recognition_utils/geo/plane.h"
#include "jsk_recognition_utils/geo/polygon.h"
#include "jsk_recognition_utils/geo/convex_polygon.h"
#include "jsk_recognition_utils/geo/cube.h"

// Utitlity macros
inline void ROS_INFO_EIGEN_VECTOR3(const std::string& prefix,
                                   const Eigen::Vector3f& v) {
  ROS_INFO("%s: [%f, %f, %f]", prefix.c_str(), v[0], v[1], v[2]);
}

namespace jsk_recognition_utils
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

  std::vector<Plane::Ptr> convertToPlanes(
    std::vector<pcl::ModelCoefficients::Ptr>);

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
