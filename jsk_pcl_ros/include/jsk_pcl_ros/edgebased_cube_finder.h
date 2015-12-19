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


#ifndef JSK_PCL_ROS_EDGE_BASED_CUBE_FINDER_H_
#define JSK_PCL_ROS_EDGE_BASED_CUBE_FINDER_H_

#include <pcl_ros/pcl_nodelet.h>

////////////////////////////////////////////////////////
// messages
////////////////////////////////////////////////////////
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/ParallelEdge.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ParallelEdgeArray.h>

#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/pcl_util.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <boost/tuple/tuple.hpp>
#include <jsk_pcl_ros/EdgebasedCubeFinderConfig.h>
#include <dynamic_reconfigure/server.h>

#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros
{
  typedef boost::tuple<pcl::PointIndices::Ptr, pcl::PointIndices::Ptr>
  IndicesPair;
  typedef boost::tuple<pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr>
  CoefficientsPair;

  typedef boost::tuple<pcl::PointIndices::Ptr, pcl::PointIndices::Ptr, pcl::PointIndices::Ptr>
  IndicesTriple;
  typedef boost::tuple<pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr>
  CoefficientsTriple;
  typedef boost::tuple<IndicesTriple, CoefficientsTriple>
  IndicesCoefficientsTriple;
  
  ////////////////////////////////////////////////////////
  // class to represent a hypothesis
  //   value_ := 0.0 ~ 1.0
  ////////////////////////////////////////////////////////
  class CubeHypothesis
  {
  public:
    typedef boost::shared_ptr<CubeHypothesis> Ptr;
    CubeHypothesis(const IndicesPair& pair,
                   const CoefficientsPair& coefficients_pair,
                   const double outlier_threshold);
    virtual ~CubeHypothesis();
    virtual double getValue() { return value_; };
    virtual jsk_recognition_utils::Cube::Ptr getCube() { return cube_; };
    virtual void estimate(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) = 0;
    
    virtual double evaluatePointOnPlanes(
      const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
      jsk_recognition_utils::ConvexPolygon& polygon_a,
      jsk_recognition_utils::ConvexPolygon& polygon_b);
    virtual jsk_recognition_utils::PointPair computeAxisEndPoints(
      const jsk_recognition_utils::Line& axis,
      const jsk_recognition_utils::PointPair& a_candidates,
      const jsk_recognition_utils::PointPair& b_candidates);
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void computeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                 const pcl::PointIndices::Ptr& indices,
                                 Eigen::Vector3f& output);
    virtual void getLinePoints(const jsk_recognition_utils::Line& line,
                               const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                               const pcl::PointIndices::Ptr indices,
                               jsk_recognition_utils::Vertices& output);
    virtual jsk_recognition_utils::ConvexPolygon::Ptr buildConvexPolygon(
      const jsk_recognition_utils::PointPair& a_edge_pair, const jsk_recognition_utils::PointPair& b_edge_pair);
    
    ////////////////////////////////////////////////////////
    // variables
    ////////////////////////////////////////////////////////
    double value_;
    const IndicesPair indices_pair_;
    const CoefficientsPair coefficients_pair_;
    double outlier_threshold_;
    jsk_recognition_utils::Cube::Ptr cube_;
  private:
  };

  class PlanarCubeHypothesis: public CubeHypothesis
  {
  public:
    typedef boost::shared_ptr<PlanarCubeHypothesis> Ptr;
    PlanarCubeHypothesis(const IndicesPair& pair,
                         const CoefficientsPair& coefficients_pair,
                         const double outlier_threshold);
    virtual void estimate(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
    }
    
  protected:
  private:
  };
  
  class DiagnoalCubeHypothesis: public CubeHypothesis
  {
  public:
    typedef boost::shared_ptr<DiagnoalCubeHypothesis> Ptr;
    DiagnoalCubeHypothesis(const IndicesPair& pair,
                           const CoefficientsPair& coefficients_pair,
                           const double outlier_threshold);
    virtual void estimate(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  protected:
    int resolution_;
    double min_angle_;
  private:
  };
  
  class EdgebasedCubeFinder: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ParallelEdgeArray > SyncPolicy;
    typedef pcl::PointXYZRGB PointT;
    typedef jsk_pcl_ros::EdgebasedCubeFinderConfig Config;
    enum EdgeRelation
    {
      NOT_PERPENDICULAR,
      A_PERPENDICULAR,
      B_PERPENDICULAR,
      C_PERPENDICULAR
    };
    
  protected:
    
    
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    
    virtual void estimate(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
      const jsk_recognition_msgs::ParallelEdgeArray::ConstPtr& input_edges);
    virtual void estimate2(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
      const jsk_recognition_msgs::ParallelEdgeArray::ConstPtr& input_edges);

    ////////////////////////////////////////////////////////
    // combinateIndices
    //   indices := list of pcl::PointIndices. the length of the
    //              list should be greater equal than 2.
    ////////////////////////////////////////////////////////
    virtual std::vector<IndicesPair> combinateIndices(
      const std::vector<pcl::PointIndices::Ptr>& indices);

    virtual std::vector<CoefficientsPair> combinateCoefficients(
      const std::vector<pcl::ModelCoefficients::Ptr>& coefficients);
    
    virtual bool isPerpendicularVector(
      const Eigen::Vector3f& a,
      const Eigen::Vector3f& b);
    
    virtual std::vector<IndicesCoefficientsTriple>
    tripleIndicesAndCoefficients(
      const std::vector<pcl::PointIndices::Ptr>& indices,
      const std::vector<pcl::ModelCoefficients::Ptr>& coefficients);

    virtual EdgeRelation perpendicularEdgeTriple(
      const jsk_recognition_utils::Line& edge_a,
      const jsk_recognition_utils::Line& edge_b,
      const jsk_recognition_utils::Line& edge_c);
    
    
    virtual std::vector<IndicesCoefficientsTriple>
    filterPerpendicularEdgeTriples(
      const std::vector<IndicesCoefficientsTriple>& triples);

    virtual jsk_recognition_utils::Line::Ptr midLineFromCoefficientsPair(
      const CoefficientsPair& pair);

    virtual jsk_recognition_utils::ConvexPolygon::Ptr convexFromPairs(
      const pcl::PointCloud<PointT>::Ptr cloud,
      const CoefficientsPair& coefficients_pair,
      const IndicesPair& indices_pair);
    virtual int countInliers(
      const pcl::PointCloud<PointT>::Ptr cloud,
      const jsk_recognition_utils::ConvexPolygon::Ptr convex);
    virtual void filterBasedOnConvex(
      const pcl::PointCloud<PointT>::Ptr cloud,
      const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes,
      std::vector<int>& output_indices);
    virtual void filterPairsBasedOnParallelEdgeDistances(
      const std::vector<IndicesPair>& pairs,
      const std::vector<CoefficientsPair>& coefficients_pair,
      std::vector<IndicesPair>& filtered_indices_pairs,
      std::vector<CoefficientsPair>& filtered_coefficients_pairs);

    virtual jsk_recognition_utils::Cube::Ptr cubeFromIndicesAndCoefficients(
    const pcl::PointCloud<PointT>::Ptr cloud,
    const IndicesCoefficientsTriple& indices_coefficients_triple,
    pcl::PointCloud<EdgebasedCubeFinder::PointT>::Ptr points_on_edge);
    
    virtual pcl::PointCloud<PointT>::Ptr extractPointCloud(
      const pcl::PointCloud<PointT>::Ptr cloud,
      const pcl::PointIndices::Ptr indices);
    
    virtual jsk_recognition_utils::PointPair minMaxPointOnLine(
      const jsk_recognition_utils::Line& line,
      const pcl::PointCloud<PointT>::Ptr cloud);
    
    virtual void estimateParallelPlane(
      const jsk_recognition_utils::ConvexPolygon::Ptr convex,
      const pcl::PointCloud<PointT>::Ptr filtered_cloud,
      pcl::PointIndices::Ptr output_inliers,
      pcl::ModelCoefficients::Ptr output_coefficients);

    virtual void estimatePerpendicularPlane(
     const jsk_recognition_utils::ConvexPolygon::Ptr convex,
     const CoefficientsPair& edge_coefficients,
     const pcl::PointCloud<PointT>::Ptr filtered_cloud,
     pcl::PointIndices::Ptr output_inliers,
     pcl::ModelCoefficients::Ptr output_coefficients);
    
    //virtual pcl::PointCloud<EdgebasedCubeFinder::PointT>::Ptr
    virtual pcl::PointIndices::Ptr
    preparePointCloudForRANSAC(
      const jsk_recognition_utils::ConvexPolygon::Ptr convex,
      const CoefficientsPair& edge_coefficients_pair,
      const pcl::PointCloud<PointT>::Ptr cloud);

    virtual jsk_recognition_utils::ConvexPolygon::Ptr
    estimateConvexPolygon(
      const pcl::PointCloud<PointT>::Ptr cloud,
      const pcl::PointIndices::Ptr indices,
      pcl::ModelCoefficients::Ptr coefficients,
      pcl::PointIndices::Ptr inliers);
    
    virtual void configCallback (Config &config, uint32_t level);

    virtual void subscribe();
    virtual void unsubscribe();
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ParallelEdgeArray> sub_edges_;
    ros::Publisher pub_;
    ros::Publisher pub_pose_array_;
    ros::Publisher pub_debug_marker_, pub_debug_filtered_cloud_,
      pub_debug_polygons_, pub_debug_clusers_;
    boost::mutex mutex_;
    ////////////////////////////////////////////////////////
    // parameters for cube finnding
    ////////////////////////////////////////////////////////
    double outlier_threshold_;
    double min_inliers_;
    double convex_area_threshold_;
    double convex_edge_threshold_;
    double parallel_edge_distance_min_threshold_, parallel_edge_distance_max_threshold_;
  private:
    
  };
}

#endif
