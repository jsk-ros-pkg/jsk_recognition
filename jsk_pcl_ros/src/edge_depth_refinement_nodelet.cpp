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

#include "jsk_pcl_ros/edge_depth_refinement.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include "jsk_pcl_ros/pcl_util.h"
#include <sstream>

namespace jsk_pcl_ros
{
  void EdgeDepthRefinement::onInit()
  {
    PCLNodelet::onInit();

    ////////////////////////////////////////////////////////
    // publishers
    ////////////////////////////////////////////////////////
    pub_indices_ = pnh_->advertise<ClusterPointIndices>(
      "output", 1);
    pub_coefficients_ = pnh_->advertise<ModelCoefficientsArray>(
      "output_coefficients", 1);
    pub_outlier_removed_indices_ = pnh_->advertise<ClusterPointIndices>(
      "output_outlier_removed", 1);
    pub_outlier_removed_coefficients_ = pnh_->advertise<ModelCoefficientsArray>(
      "output_outlier_removed_coefficients", 1);
    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&EdgeDepthRefinement::configCallback, this, _1, _2);
    srv_->setCallback (f);

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

  ////////////////////////////////////////////////////////
  // line coefficients are:
  // 0: point.x
  // 1: point.y
  // 2. point.z
  // 3. direction.x
  // 4: direction.y
  // 5. direction.z
  ////////////////////////////////////////////////////////
  Line::Ptr EdgeDepthRefinement::lineFromCoefficients(
    const pcl::ModelCoefficients::Ptr coefficients)
  {
    Eigen::Vector3f p(coefficients->values[0],
                      coefficients->values[1],
                      coefficients->values[2]);
    Eigen::Vector3f d(coefficients->values[3],
                      coefficients->values[4],
                      coefficients->values[5]);
    Line::Ptr ret (new Line(d, p));
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
        max_y = max_y;
        max_y_index = index;
      }
      if (y < min_y) {
        min_y = y;
        min_y_index = index;
      }
    }

    if (max_x_index != max_x_index) {
      return boost::make_tuple(
        max_x_index, min_x_index);
    }
    else {
      return boost::make_tuple(
        max_y_index, min_y_index);
    }
  }

  
  Segment::Ptr EdgeDepthRefinement::segmentFromIndices(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& indices,
    const Line::Ptr& line)
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
    Segment::Ptr segment (new Segment(min_foot, max_foot));
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
      integrated_indices = addIndices(all_inliers[*it]->indices,
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
    std::vector<pcl::PointIndices::Ptr> nonduplicated_inliers;
    std::vector<pcl::ModelCoefficients::Ptr> cnonduplicated_oefficients;

    // buildup Lines and Segments
    std::vector<Line::Ptr> lines;
    std::vector<Segment::Ptr> segments;
    
    for (size_t i = 0; i < all_inliers.size(); i++) {
      pcl::PointIndices::Ptr the_inliers = all_inliers[i];
      pcl::ModelCoefficients::Ptr the_coefficients = all_coefficients[i];
      Line::Ptr the_line = lineFromCoefficients(the_coefficients);
      Segment::Ptr the_segment
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
      Line::Ptr the_line = lines[i];
      Segment::Ptr the_segment = segments[i];
      for (size_t j = i + 1; j < all_inliers.size(); j++) {
        Line::Ptr candidate_line = lines[j];
        Segment::Ptr candidate_segment = segments[j];
        
        double angle_diff = the_line->angle(*candidate_line);
        if (duplication_angle_threshold_ > angle_diff) {
          double distance_diff = the_line->distance(*candidate_line);
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
    for (size_t i = 0; i < all_inliers.size() - 1; i++) {
      std::vector<int> duplication_list = duplication_map[i];
      if (duplicated_indices.find(i) == duplicated_indices.end()) {
        if (duplication_list.size() == 0) {
          // nothing to do...
        }
        else {
          std::set<int> new_duplication_set;
          buildGroupFromGraphMap(duplication_map,
                                 i,
                                 duplication_list,
                                 new_duplication_set);
          duplication_set_list.push_back(new_duplication_set);
          // add new_duplication_set to duplicated_indices
          addSet<int>(duplicated_indices, new_duplication_set);
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
    const std::vector<pcl::PointIndices::Ptr> inliers,
    const std::vector<pcl::ModelCoefficients::Ptr> coefficients,
    const std_msgs::Header& header)
  {
    ClusterPointIndices output_ros_msg;
    ModelCoefficientsArray output_ros_coefficients_msg;
    output_ros_msg.header = header;
    output_ros_coefficients_msg.header = header;
    for (size_t i = 0; i < inliers.size(); i++) {
      PCLIndicesMsg output_indices_msg;
      PCLModelCoefficientMsg output_coefficients_msg;
      output_indices_msg.header = header;
      output_indices_msg.indices = inliers[i]->indices;
      output_ros_msg.cluster_indices.push_back(output_indices_msg);

      output_coefficients_msg.header = header;
      output_coefficients_msg.values = coefficients[i]->values;
      output_ros_coefficients_msg.coefficients.push_back(output_coefficients_msg);
    }
    pub.publish(output_ros_msg);
    pub_coefficients.publish(output_ros_coefficients_msg);
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
    const jsk_pcl_ros::ClusterPointIndicesConstPtr &indices)
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
                   inliers, coefficients,
                   input->header);
    publishIndices(pub_indices_,
                   pub_coefficients_,
                   non_duplicated_inliers,
                   non_duplicated_coefficients,
                   input->header);
  }
  
}

#include <pluginlib/class_list_macros.h>
typedef jsk_pcl_ros::EdgeDepthRefinement EdgeDepthRefinement;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, EdgeDepthRefinement, EdgeDepthRefinement, nodelet::Nodelet);
