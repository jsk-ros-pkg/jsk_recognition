// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#ifndef _REGION_ADJACENCY_GRAPH_H_
#define _REGION_ADJACENCY_GRAPH_H_

// ROS header directives
#include <ros/ros.h>
#include <ros/console.h>

// OpenCV header directives
#include <opencv2/imgproc/imgproc.hpp>

// PCL header directives
#include <pcl/point_cloud.h>
#include <pcl/features/vfh.h>

// boost header directives
#include <boost/graph/adjacency_list.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/config.hpp>

#include <string>
#include <map>

namespace jsk_pcl_ros
{
   class RegionAdjacencyGraph
   {
    private:
      struct VertexProperty {
         int v_index;
         pcl::PointXYZ v_center;
         int v_label;
         VertexProperty(
            int i = -1,
            pcl::PointXYZ center = pcl::PointXYZ(-1, -1, -1),
            int label = -1) :
            v_index(i), v_center(center), v_label(label) {}
      };
      typedef boost::property<boost::edge_weight_t, float> EdgeProperty;
      typedef typename boost::adjacency_list<boost::vecS,
                                             boost::vecS,
                                             boost::undirectedS,
                                             VertexProperty,
                                             EdgeProperty> Graph;
      typedef typename boost::graph_traits<
         Graph>::adjacency_iterator AdjacencyIterator;
      typedef typename boost::property_map<
         Graph, boost::vertex_index_t>::type IndexMap;
      typedef typename boost::graph_traits<
         Graph>::edge_descriptor EdgeDescriptor;
      typedef typename boost::property_map<
         Graph, boost::edge_weight_t>::type EdgePropertyAccess;
      typedef typename boost::property_traits<boost::property_map<
         Graph, boost::edge_weight_t>::const_type>::value_type EdgeValue;
      typedef typename boost::graph_traits<
         Graph>::vertex_iterator VertexIterator;
      typedef typename boost::graph_traits<
         Graph>::vertex_descriptor VertexDescriptor;
      typedef pcl::PointXYZRGB PointT;
      Graph graph_;
      
      void sampleRandomPointsFromCloudCluster(
         pcl::PointCloud<PointT>::Ptr,
         pcl::PointCloud<pcl::Normal>::Ptr,
         std::vector<Eigen::Vector3f> &,
         std::vector<Eigen::Vector3f> &,
         int = 3);
      template<typename T>
      T convexityCriterion(
         const Eigen::Vector3f &,
         const Eigen::Vector3f &,
         const Eigen::Vector3f &,
         const Eigen::Vector3f &);
      template<typename T>
      T getCloudClusterWeightFunction(
         const std::vector<std::vector<Eigen::Vector3f> > &,
         const std::vector<std::vector<Eigen::Vector3f> > &);
      float getVectorAngle(
         const Eigen::Vector3f &,
         const Eigen::Vector3f &,
         bool = true);
      int getCommonNeigbour(
         const std::vector<int> &,
         const std::vector<int> &);
      void computeCloudClusterRPYHistogram(
         const pcl::PointCloud<PointT>::Ptr,
         const pcl::PointCloud<pcl::Normal>::Ptr,
         cv::Mat &);

    public:
      RegionAdjacencyGraph();
      virtual void generateRAG(
         const std::vector<pcl::PointCloud<PointT>::Ptr> &,
         const std::vector<pcl::PointCloud<pcl::Normal>::Ptr> &,
         const pcl::PointCloud<pcl::PointXYZ>::Ptr,
         std::vector<std::vector<int> > &,
         const int = RAG_EDGE_WEIGHT_DISTANCE);
      virtual void splitMergeRAG(const int = 0.0f);
      virtual void getCloudClusterLabels(
         std::vector<int> &);
      virtual void printGraph(
         const Graph &);
      enum {
         RAG_EDGE_WEIGHT_DISTANCE,
         RAG_EDGE_WEIGHT_CONVEX_CRITERIA
      };
    };
}  // namespace jsk_pcl_ros
#endif  //  _REGION_ADJACENCY_GRAPH_H_
