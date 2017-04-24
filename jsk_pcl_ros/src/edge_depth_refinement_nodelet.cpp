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

#include "jsk_pcl_ros/edge_depth_refinement.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include "jsk_recognition_utils/pcl_util.h"
#include <sstream>

namespace jsk_pcl_ros
{
  void EdgeDepthRefinement::onInit()
  {
    ConnectionBasedNodelet::onInit();

    ////////////////////////////////////////////////////////
    // publishers
    ////////////////////////////////////////////////////////
    pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output", 1);
    pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output_coefficients", 1);
    pub_edges_ = advertise<jsk_recognition_msgs::SegmentArray>(
      *pnh_, "output_edges", 1);
    pub_outlier_removed_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output_outlier_removed", 1);
    pub_outlier_removed_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output_outlier_removed_coefficients", 1);
    pub_outlier_removed_edges_= advertise<jsk_recognition_msgs::SegmentArray>(
      *pnh_, "output_outlier_removed_edges", 1);
    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EdgeDepthRefinement::configCallback, this, _1, _2);
    srv_->setCallback (f);

    onInitPostProcess();
  }

  void EdgeDepthRefinement::subscribe()
  {
    ////////////////////////////////////////////////////////
    // subscribesrs
    ////////////////////////////////////////////////////////
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_indices_);
    sync_->registerCallback(boost::bind(&EdgeDepthRefinement::refine,
                                        this, _1, _2));
  }

  void EdgeDepthRefinement::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_indices_.unsubscribe();
  }
  
  ////////////////////////////////////////////////////////
  // line coefficients are:
  // 0: point.x
  // 1: point.y
  // 2. point.z
  // 3. direction.x
  // 4: direction.y
  // 5. direction.z
  ////////////////////////////////////////////////////////
  jsk_recognition_utils::Line::Ptr EdgeDepthRefinement::lineFromCoefficients(
    const pcl::ModelCoefficients::Ptr coefficients)
  {
    Eigen::Vector3f p(coefficients->values[0],
                      coefficients->values[1],
                      coefficients->values[2]);
    Eigen::Vector3f d(coefficients->values[3],
                      coefficients->values[4],
                      coefficients->values[5]);
    jsk_recognition_utils::Line::Ptr ret (new jsk_recognition_utils::Line(d, p));
    return ret;
  }

  boost::tuple<int, int> EdgeDepthRefinement::findMinMaxIndex(
    const int width, const int height,
    const std::vector<int>& indices)
  {
    
    int min_y_index, max_y_index, min_x_index, max_x_index;
    int min_y = INT_MAX;
    int min_x = INT_MAX;
    int max_y = INT_MIN;
    int max_x = INT_MIN;
    for (size_t i = 0; i < indices.size(); i++) {
      int index = indices[i];
      int x = index % width;
      int y = index / width;

      if (x > max_x) {
        max_x = x;
        max_x_index = index;
      }
      if (x < min_x) {
        min_x = x;
        min_x_index = index;
      }
      if (y > max_y) {
        max_y = y;
        max_y_index = index;
      }
      if (y < min_y) {
        min_y = y;
        min_y_index = index;
      }
    }

    if (min_x_index != max_x_index) {
      return boost::make_tuple(
        min_x_index, max_x_index);
    }
    else {
      return boost::make_tuple(
        min_y_index, max_y_index);
    }
  }

  
  jsk_recognition_utils::Segment::Ptr EdgeDepthRefinement::segmentFromIndices(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& indices,
    const jsk_recognition_utils::Line::Ptr& line)
  {
    boost::tuple<int, int> min_max
      = findMinMaxIndex(cloud->width, cloud->height, indices);
    PointT min_point = cloud->points[min_max.get<0>()];
    PointT max_point = cloud->points[min_max.get<1>()];
    Eigen::Vector3f min_point_f = min_point.getVector3fMap();
    Eigen::Vector3f max_point_f = max_point.getVector3fMap();
    Eigen::Vector3f min_foot, max_foot;
    line->foot(min_point_f, min_foot);
    line->foot(max_point_f, max_foot);
    jsk_recognition_utils::Segment::Ptr segment (new jsk_recognition_utils::Segment(min_foot, max_foot));
    return segment;
  }

  void EdgeDepthRefinement::integrateDuplicatedIndices(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::set<int>& duplicated_set,
    const std::vector<pcl::PointIndices::Ptr> all_inliers,
    pcl::PointIndices::Ptr& output_indices)
  {
    std::vector<int> integrated_indices;
    for (std::set<int>::iterator it = duplicated_set.begin();
         it != duplicated_set.end();
         ++it) {
      integrated_indices = jsk_recognition_utils::addIndices(all_inliers[*it]->indices,
                                      integrated_indices);
    }
    output_indices->indices = integrated_indices;
  }
  
  void EdgeDepthRefinement::removeDuplicatedEdges(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<pcl::PointIndices::Ptr> all_inliers,
    const std::vector<pcl::ModelCoefficients::Ptr> all_coefficients,
    std::vector<pcl::PointIndices::Ptr>& output_inliers,
    std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients)
  {
    if (all_inliers.size() == 0) {
      NODELET_ERROR("no edges are specified");
      return;
    }

    // buildup Lines and Segments
    std::vector<jsk_recognition_utils::Line::Ptr> lines;
    std::vector<jsk_recognition_utils::Segment::Ptr> segments;
    
    for (size_t i = 0; i < all_inliers.size(); i++) {
      pcl::PointIndices::Ptr the_inliers = all_inliers[i];
      pcl::ModelCoefficients::Ptr the_coefficients = all_coefficients[i];
      jsk_recognition_utils::Line::Ptr the_line = lineFromCoefficients(the_coefficients);
      jsk_recognition_utils::Segment::Ptr the_segment
        = segmentFromIndices(cloud, the_inliers->indices, the_line);
      lines.push_back(the_line);
      segments.push_back(the_segment);
    }

    ////////////////////////////////////////////////////////
    // build duplication map
    // duplication map is a hash map from int to a list of int
    ////////////////////////////////////////////////////////
    std::map<int, std::vector<int> > duplication_map;
    for (size_t i = 0; i < all_inliers.size() - 1; i++) {
      duplication_map[i] = std::vector<int>(); // add empty map
      jsk_recognition_utils::Line::Ptr the_line = lines[i];
      jsk_recognition_utils::Segment::Ptr the_segment = segments[i];
      for (size_t j = i + 1; j < all_inliers.size(); j++) {
        jsk_recognition_utils::Line::Ptr candidate_line = lines[j];
        jsk_recognition_utils::Segment::Ptr candidate_segment = segments[j];
        Eigen::Vector3f candidate_midpoint;
        candidate_segment->midpoint(candidate_midpoint);

        double angle_diff = the_line->angle(*candidate_line);
        if (duplication_angle_threshold_ > angle_diff) {
          double distance_diff = the_segment->distance(candidate_midpoint);
          if (duplication_distance_threshold_ > distance_diff) {
            duplication_map[i].push_back(j);
          }
        }
      }
    }

    ////////////////////////////////////////////////////////
    // convert duplication map into set
    ////////////////////////////////////////////////////////
    std::vector<std::set<int> > duplication_set_list;
    std::set<int> duplicated_indices;
    for (size_t i = 0; i < all_inliers.size(); i++) {
      std::vector<int> duplication_list;
      if (i < all_inliers.size() - 1) {
        duplication_list = duplication_map[i];
      }
      if (duplicated_indices.find(i) == duplicated_indices.end()) {
        if (i == all_inliers.size() - 1 || duplication_list.size() == 0) { // no duplication found
          std::set<int> no_duplication_set;
          no_duplication_set.insert(i);
          duplication_set_list.push_back(no_duplication_set);
          jsk_recognition_utils::addSet<int>(duplicated_indices, no_duplication_set);
        }
        else { // some duplication found
          std::set<int> new_duplication_set;
          jsk_recognition_utils::buildGroupFromGraphMap(duplication_map,
                                 i,
                                 duplication_list,
                                 new_duplication_set);
          duplication_set_list.push_back(new_duplication_set);
          // add new_duplication_set to duplicated_indices
          jsk_recognition_utils::addSet<int>(duplicated_indices, new_duplication_set);
        }
      }
    }


    for (size_t i = 0; i < duplication_set_list.size(); i++) {
      pcl::PointIndices::Ptr integrated_indices (new pcl::PointIndices);
      integrateDuplicatedIndices(cloud, duplication_set_list[i],
                                 all_inliers,
                                 integrated_indices);
      output_inliers.push_back(integrated_indices);
      
      // use the first one,,, ok?
      pcl::ModelCoefficients::Ptr integrated_coefficients 
        = all_coefficients[(*duplication_set_list[i].begin())];
      output_coefficients.push_back(integrated_coefficients);
    }

    // print result for debug
    // NODELET_INFO("%lu duplication set", duplication_set_list.size());
    // for (size_t i = 0; i < duplication_set_list.size(); i++) {
    //   std::stringstream ss;
    //   ss << "[";
    //   for (std::set<int>::iterator it = duplication_set_list[i].begin();
    //        it != duplication_set_list[i].end();
    //        ++it)
    //   {
    //     ss << *it << ", ";
    //   }
    //   NODELET_INFO("%s", ss.str().c_str());
    // }
    
    // for (size_t i = 0; i < all_inliers.size() - 1; i++) {
    //   std::stringstream ss;
    //   std::vector<int> similar_indices = duplication_map[i];
    //   ss << i << " -> ";
    //   for (size_t j = 0; j < similar_indices.size(); j++) {
    //     ss << similar_indices[j] << ", ";
    //   }
    //   NODELET_INFO("%s", ss.str().c_str());
    // }
  }
  
  void EdgeDepthRefinement::removeOutliers(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<PCLIndicesMsg>& indices,
    std::vector<pcl::PointIndices::Ptr>& output_inliers,
    std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients)
  {
    // output_inliers.resize(indices.size());
    // output_coefficients.resize(indices.size());
    for (size_t i = 0; i < indices.size(); i++) {
      std::vector<int> cluster_indices = indices[i].indices;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      removeOutliersByLine(cloud, cluster_indices, *inliers, *coefficients);
      if (inliers->indices.size() > min_inliers_) {
        output_inliers.push_back(inliers);
        output_coefficients.push_back(coefficients);
      }
    }
  }
  
  void EdgeDepthRefinement::removeOutliersByLine(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& indices,
    pcl::PointIndices& inliers,
    pcl::ModelCoefficients& coefficients)
  {
    // one line shoud have ONE line at most...?
    // in that case, we can estimate the true line by RANSAC
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (outlier_distance_threshold_);
    seg.setInputCloud(cloud);
    pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices);
    indices_ptr->indices = indices;
    seg.setIndices(indices_ptr);
    seg.segment (inliers, coefficients);
  }

  void EdgeDepthRefinement::publishIndices(
    ros::Publisher& pub,
    ros::Publisher& pub_coefficients,
    ros::Publisher& pub_edges,
    const std::vector<pcl::PointIndices::Ptr> inliers,
    const std::vector<pcl::ModelCoefficients::Ptr> coefficients,
    const std_msgs::Header& header)
  {
    jsk_recognition_msgs::ClusterPointIndices output_ros_msg;
    jsk_recognition_msgs::ModelCoefficientsArray output_ros_coefficients_msg;
    jsk_recognition_msgs::SegmentArray output_ros_edges_msg;
    output_ros_msg.header = header;
    output_ros_coefficients_msg.header = header;
    output_ros_edges_msg.header = header;
    for (size_t i = 0; i < inliers.size(); i++) {
      PCLIndicesMsg output_indices_msg;
      PCLModelCoefficientMsg output_coefficients_msg;
      jsk_recognition_msgs::Segment output_edge_msg;
      output_indices_msg.header = header;
      output_indices_msg.indices = inliers[i]->indices;
      output_ros_msg.cluster_indices.push_back(output_indices_msg);

      output_coefficients_msg.header = header;
      output_coefficients_msg.values = coefficients[i]->values;
      output_ros_coefficients_msg.coefficients.push_back(output_coefficients_msg);

      output_edge_msg.start_point.x = coefficients[i]->values[0] - coefficients[i]->values[3];
      output_edge_msg.start_point.y = coefficients[i]->values[1] - coefficients[i]->values[4];
      output_edge_msg.start_point.z = coefficients[i]->values[2] - coefficients[i]->values[5];
      output_edge_msg.end_point.x = coefficients[i]->values[0] + coefficients[i]->values[3];
      output_edge_msg.end_point.y = coefficients[i]->values[1] + coefficients[i]->values[4];
      output_edge_msg.end_point.z = coefficients[i]->values[2] + coefficients[i]->values[5];
      output_ros_edges_msg.segments.push_back(output_edge_msg);
    }
    pub.publish(output_ros_msg);
    pub_coefficients.publish(output_ros_coefficients_msg);
    pub_edges.publish(output_ros_edges_msg);
  }

  void EdgeDepthRefinement::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    outlier_distance_threshold_ = config.outlier_distance_threshold;
    min_inliers_ = config.min_inliers;
    duplication_angle_threshold_ = config.duplication_angle_threshold;
    duplication_distance_threshold_ = config.duplication_distance_threshold;
  }
  
  void EdgeDepthRefinement::refine(
    const sensor_msgs::PointCloud2ConstPtr &input,
    const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input, *cloud);
    
    std::vector<pcl::PointIndices::Ptr> inliers;
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;

    removeOutliers(cloud, indices->cluster_indices, inliers, coefficients);
    std::vector<pcl::PointIndices::Ptr> non_duplicated_inliers;
    std::vector<pcl::ModelCoefficients::Ptr> non_duplicated_coefficients;
    removeDuplicatedEdges(cloud, inliers, coefficients,
                          non_duplicated_inliers,
                          non_duplicated_coefficients);
    publishIndices(pub_outlier_removed_indices_,
                   pub_outlier_removed_coefficients_,
                   pub_outlier_removed_edges_,
                   inliers, coefficients,
                   input->header);
    publishIndices(pub_indices_,
                   pub_coefficients_,
                   pub_edges_,
                   non_duplicated_inliers,
                   non_duplicated_coefficients,
                   input->header);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::EdgeDepthRefinement, nodelet::Nodelet);
