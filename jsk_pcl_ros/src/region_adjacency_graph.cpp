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

#include <jsk_pcl_ros/region_adjacency_graph.h>
#include <jsk_topic_tools/log_utils.h>

namespace jsk_pcl_ros
{   
    RegionAdjacencyGraph::RegionAdjacencyGraph()
    {
       
    }
   
    void RegionAdjacencyGraph::generateRAG(
       const std::vector<pcl::PointCloud<PointT>::Ptr> &cloud_clusters,
       const std::vector<pcl::PointCloud<pcl::Normal>::Ptr>  &normal_clusters,
       const pcl::PointCloud<pcl::PointXYZ>::Ptr centroids,
       std::vector<std::vector<int> > &neigbor_indices,
       const int edge_weight_criteria)
    {
       if (cloud_clusters.empty() || normal_clusters.empty() ||
           centroids->empty() || neigbor_indices.empty()) {
          ROS_ERROR("ERROR: Cannot Generate RAG of empty data...");
          return;
       }
       const int comparision_points_size = 100;
       if (cloud_clusters.size() == neigbor_indices.size()) {
          std::vector<VertexDescriptor> vertex_descriptor;
          for (int j = 0; j < centroids->size(); j++) {
             VertexDescriptor v_des = boost::add_vertex(
                VertexProperty(j, centroids->points[j], -1), this->graph_);
             vertex_descriptor.push_back(v_des);
          }
          for (int j = 0; j < neigbor_indices.size(); j++) {
             VertexDescriptor r_vd = vertex_descriptor[j];
             std::vector<Eigen::Vector3f> center_point;
             std::vector<Eigen::Vector3f> center_normal;
             cv::Mat r_histogram;
             if (edge_weight_criteria == RAG_EDGE_WEIGHT_CONVEX_CRITERIA) {
                this->sampleRandomPointsFromCloudCluster(
                   cloud_clusters[j],
                   normal_clusters[j],
                   center_point,
                   center_normal,
                   comparision_points_size);
             } else if (edge_weight_criteria == RAG_EDGE_WEIGHT_DISTANCE) {
                if (cloud_clusters[j]->size() > sizeof(char) &&
                    normal_clusters[j]->size() > sizeof(char)) {
                   this->computeCloudClusterRPYHistogram(
                      cloud_clusters[j],
                      normal_clusters[j],
                      r_histogram);
                }
             } else {
                ROS_ERROR("Incorrect Measurement type");
                return;
             }
             for (int i = 0; i < neigbor_indices[j].size(); i++) {
                int n_index = neigbor_indices[j][i];
                VertexDescriptor vd = vertex_descriptor[n_index];
                float distance = 0.0f;
                if (edge_weight_criteria == RAG_EDGE_WEIGHT_CONVEX_CRITERIA) {
                   std::vector<Eigen::Vector3f> n1_point;
                   std::vector<Eigen::Vector3f> n1_normal;
                   this->sampleRandomPointsFromCloudCluster(
                      cloud_clusters[n_index],
                      normal_clusters[n_index],
                      n1_point,
                      n1_normal,
                      comparision_points_size);
                   // Common Neigbour
                   int commonIndex = this->getCommonNeigbour(
                      neigbor_indices[j],
                      neigbor_indices[n_index]);
                   if (commonIndex == -1) {
                      // distance = this->getCloudClusterWeightFunction<float>(
                      //    center_point, n1_point, center_normal, n1_normal);
                   } else {
                      std::vector<Eigen::Vector3f> n2_point;
                      std::vector<Eigen::Vector3f> n2_normal;
                      this->sampleRandomPointsFromCloudCluster(
                         cloud_clusters[commonIndex],
                         normal_clusters[commonIndex],
                         n2_point,
                         n2_normal,
                         comparision_points_size);
                      std::vector<std::vector<Eigen::Vector3f> > _points;
                      std::vector<std::vector<Eigen::Vector3f> > _normals;
                      _points.push_back(center_point);
                      _points.push_back(n1_point);
                      // _points.push_back(n2_point);
                      _normals.push_back(center_normal);
                      _normals.push_back(n1_normal);
                      // _normals.push_back(n2_normal);
                      distance = this->getCloudClusterWeightFunction<float>(
                         _points, _normals);
                   }
                } else if (edge_weight_criteria == RAG_EDGE_WEIGHT_DISTANCE) {
                   if (cloud_clusters[j]->size() > sizeof(char) &&
                       cloud_clusters[n_index]->size() > sizeof(char)) {
                      cv::Mat n_histogram;
                      this->computeCloudClusterRPYHistogram(
                         cloud_clusters[n_index],
                         normal_clusters[n_index],
                         n_histogram);
                      distance = static_cast<float>(
                         cv::compareHist(
                            r_histogram, n_histogram, CV_COMP_CORREL));
                   } else {
                      distance = 0.0f;
                   }
                } else {
                   distance = 0.0f;
                }
                if (r_vd != vd) {
                   bool found = false;
                   EdgeDescriptor e_descriptor;
                   boost::tie(e_descriptor, found) = boost::edge(r_vd, vd, this->graph_);
                   if (!found) {
                      boost::add_edge(
                         r_vd, vd, EdgeProperty(distance), this->graph_);
                   }
                }
             }
          }
       } else {
          ROS_WARN("Elements not same size..");
       }
    }

    template<typename T>
    T RegionAdjacencyGraph::getCloudClusterWeightFunction(
       const std::vector<std::vector<Eigen::Vector3f> > &_points,
       const std::vector<std::vector<Eigen::Vector3f> > &_normal)
    {
#define ANGLE_THRESHOLD (10)
       if (_points.size() == 2 && _points.size() == _normal.size()) {
          T weights_ = -1.0f;
          int concave_ = 0;
          int convex_ = 0;
          for (int i = 0; i < _points[0].size(); i++) {
          T convexC_ij = this->convexityCriterion<T>(
             _points[0][i], _points[1][i], _normal[0][i], _normal[1][i]);
          float angle_ = getVectorAngle(_normal[0][i], _normal[1][i]);
          if (convexC_ij < 0.0f && angle_ < ANGLE_THRESHOLD) {
             convexC_ij = abs(convexC_ij);
          }
          if (convexC_ij > 0.0) {
             convex_++;
          }
          if (convexC_ij <= 0.0 || std::isnan(convexC_ij)) {
             concave_++;
          }
          /*
          if (convexC_ij > weights_) {
             weights_ = convexC_ij;
             }*/
          }
          if (concave_ < convex_ + 20) {
             weights_ = 1.0f;
          }
          return weights_;
       } else if (_points.size() == 3) {
          T weights_ = FLT_MIN;
          for (int i = 0; i < _points[0].size(); i++) {
             T convexC_ij = this->convexityCriterion<T>(
                _points[0][i], _points[1][i], _normal[0][i], _normal[1][i]);
             T convexC_ic = this->convexityCriterion<T>(
                _points[0][i], _points[2][i], _normal[0][i], _normal[2][i]);
             T convexC_jc = this->convexityCriterion<T>(
                _points[1][i], _points[2][i], _normal[1][i], _normal[2][i]);
          // float angle_ = getVectorAngle(_normal[0][i], _normal[1][i]);
          // if (angle_ > ANGLE_THRESHOLD && convexC_ij <= 0) {
          //    convexC_ij =/ -1;
          // }
             weights_ = std::max(convexC_ij,
                                 std::max(convexC_ic, convexC_jc));
          }
          return weights_;
       }
    }

    float RegionAdjacencyGraph::getVectorAngle(
       const Eigen::Vector3f &vector1,
       const Eigen::Vector3f &vector2,
       bool indegree)
    {
       float angle_ = acos(vector1.dot(vector2));
       if (indegree) {
          return angle_ * 180/M_PI;
       } else {
          return angle_;
       }
    }

    template<typename T>
    T RegionAdjacencyGraph::convexityCriterion(
       const Eigen::Vector3f &center_point,
       const Eigen::Vector3f &n1_point,
       const Eigen::Vector3f &center_normal,
       const Eigen::Vector3f &neigbour_normal)
    {
       Eigen::Vector3f difference_ = center_point - n1_point;
       difference_ /= difference_.norm();
       T convexityc = static_cast<T>(center_normal.dot(difference_) -
                                     neigbour_normal.dot(difference_));
       return convexityc;
    }

    void RegionAdjacencyGraph::sampleRandomPointsFromCloudCluster(
       pcl::PointCloud<PointT>::Ptr cloud,
       pcl::PointCloud<pcl::Normal>::Ptr normal,
       std::vector<Eigen::Vector3f> &point_vector,
       std::vector<Eigen::Vector3f> &normal_vector,
       int gen_sz)
    {
       for (int i = 0; i < std::max(gen_sz, (int)cloud->size()); i++) {
          int _idx = rand() % cloud->size();
          Eigen::Vector3f cv = cloud->points[_idx].getVector3fMap();
          Eigen::Vector3f nv = Eigen::Vector3f(
             normal->points[_idx].normal_x,
             normal->points[_idx].normal_y,
             normal->points[_idx].normal_z);
          point_vector.push_back(cv);
          normal_vector.push_back(nv);
       }
    }

    void RegionAdjacencyGraph::splitMergeRAG(
       const int _threshold)
    {
       if (num_vertices(this->graph_) == 0) {
          ROS_ERROR("ERROR: Cannot Merge Empty RAG ...");
          return;
       }
       IndexMap index_map = get(boost::vertex_index, this->graph_);
       EdgePropertyAccess edge_weights = get(boost::edge_weight, this->graph_);
       VertexIterator i, end;
       int label = -1;
       for (boost::tie(i, end) = vertices(this->graph_); i != end; i++) {
          if (this->graph_[*i].v_label == -1) {
             graph_[*i].v_label = ++label;
          }
          AdjacencyIterator ai, a_end;
          boost::tie(ai, a_end) = adjacent_vertices(*i, this->graph_);
          for (; ai != a_end; ++ai) {
             bool found = false;
             EdgeDescriptor e_descriptor;
             boost::tie(e_descriptor, found) = boost::edge(*i, *ai, this->graph_);
             if (found) {
                EdgeValue edge_val = boost::get(
                   boost::edge_weight, this->graph_, e_descriptor);
                float weights_ = edge_val;
                if (weights_ < _threshold) {
                   remove_edge(e_descriptor, this->graph_);
                } else {
                   if (this->graph_[*ai].v_label == -1) {
                      this->graph_[*ai].v_label = this->graph_[*i].v_label;
                   }
                }
             }
          }
       }
#ifdef DEBUG
       // this->printGraph(this->graph_);
       std::cout << "\nPRINT INFO. \n --Graph Size: "
                 << num_vertices(this->graph_) <<
          std::endl << "--Total Label: " << label << "\n\n";
#endif  // DEBUG
    }

    int RegionAdjacencyGraph::getCommonNeigbour(
       const std::vector<int> &c1_neigbour,
       const std::vector<int> &c2_neigbour)
    {
       int commonIndex = -1;
       for (int j = 0; j < c1_neigbour.size(); j++) {
          int c1_val = c1_neigbour[j];
          for (int i = 0; i < c2_neigbour.size(); i++) {
             int c2_val = c2_neigbour[i];
             if (c1_val == c2_val) {
                commonIndex = c1_val;
                break;
             }
          }
       }
       return commonIndex;
    }

    void RegionAdjacencyGraph::getCloudClusterLabels(
       std::vector<int> &labelMD)
    {
       labelMD.clear();
       VertexIterator i, end;
       for (boost::tie(i, end) = vertices(this->graph_); i != end; ++i) {
          labelMD.push_back(static_cast<int>(this->graph_[*i].v_label));
       }
    }

    void RegionAdjacencyGraph::printGraph(
       const Graph &_graph)
    {
       VertexIterator i, end;
       for (boost::tie(i, end) = vertices(_graph); i != end; ++i) {
          AdjacencyIterator ai, a_end;
          boost::tie(ai, a_end) = adjacent_vertices(*i, _graph);
          std::cout << *i << "\t" << _graph[*i].v_label << std::endl;
       }
    }

    void RegionAdjacencyGraph::computeCloudClusterRPYHistogram(
       const pcl::PointCloud<PointT>::Ptr _cloud,
       const pcl::PointCloud<pcl::Normal>::Ptr _normal,
       cv::Mat &_histogram)
    {
       pcl::VFHEstimation<PointT,
                          pcl::Normal,
                          pcl::VFHSignature308> vfh;
       vfh.setInputCloud(_cloud);
       vfh.setInputNormals(_normal);
       pcl::search::KdTree<PointT>::Ptr tree(
          new pcl::search::KdTree<PointT>);
       vfh.setSearchMethod(tree);
       pcl::PointCloud<pcl::VFHSignature308>::Ptr _vfhs(
          new pcl::PointCloud<pcl::VFHSignature308>());
       vfh.compute(*_vfhs);
       _histogram = cv::Mat(sizeof(char), 308, CV_32F);
       for (int i = 0; i < _histogram.cols; i++) {
          _histogram.at<float>(0, i) = _vfhs->points[0].histogram[i];
       }
       float curvature_ = 0.0f;
       for (int i = 0; i < _normal->size(); i++) {
          curvature_ += _normal->points[i].curvature;
       }
       curvature_ /= static_cast<float>(_normal->size());
       cv::normalize(
          _histogram, _histogram, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    }
}  // namespace jsk_pcl_ros
