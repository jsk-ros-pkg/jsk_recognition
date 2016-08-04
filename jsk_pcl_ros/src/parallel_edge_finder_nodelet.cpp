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

#include "jsk_pcl_ros/parallel_edge_finder.h"
#include "jsk_recognition_utils/pcl_util.h"
#include <jsk_recognition_msgs/ParallelEdgeArray.h>

namespace jsk_pcl_ros
{
  void ParallelEdgeFinder::onInit()
  {
    ConnectionBasedNodelet::onInit();
    
    ////////////////////////////////////////////////////////
    // publishers
    ////////////////////////////////////////////////////////
    pub_ = advertise<jsk_recognition_msgs::ParallelEdgeArray>(*pnh_, "output_edges_groups", 1);
    pub_clusters_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output_clusters", 1);

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ParallelEdgeFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);
    onInitPostProcess();
  }

  void ParallelEdgeFinder::subscribe()
  {
    ////////////////////////////////////////////////////////
    // subscription
    ////////////////////////////////////////////////////////
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_indices_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&ParallelEdgeFinder::estimate,
                                        this, _1, _2));
  }

  void ParallelEdgeFinder::unsubscribe()
  {
    sub_indices_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }

  void ParallelEdgeFinder::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    angle_threshold_ = config.angular_threshold;
  }
  
  void ParallelEdgeFinder::estimate(
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& input_coefficients)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ////////////////////////////////////////////////////////
    // first, build Line instances
    ////////////////////////////////////////////////////////
    std::vector<jsk_recognition_utils::Line::Ptr> lines;
    if (input_coefficients->coefficients.size() == 0) {
      return;
    }
    for (size_t i = 0; i < input_coefficients->coefficients.size(); i++) {
      std::vector<float> the_coefficients
        = input_coefficients->coefficients[i].values;
      lines.push_back(jsk_recognition_utils::Line::fromCoefficients(the_coefficients));
    }

    std::map<int, std::vector<int> > parallel_map;
    for (size_t i = 0; i < lines.size() - 1; i++) {
      parallel_map[i] = std::vector<int>();
      jsk_recognition_utils::Line::Ptr the_line = lines[i];
      for (size_t j = i + 1; j < lines.size(); j++) {
        jsk_recognition_utils::Line::Ptr candidate_line = lines[j];
        if (the_line->isParallel(*candidate_line, angle_threshold_)) {
          parallel_map[i].push_back(j);
        }
      }
    }
    
    // build 'Group' recursively
    // list of set of the indices of parallel edges
    std::vector<std::set<int> > parallel_groups_list;
    
    // set of the indices which are already regarded as one of
    // parallel edges
    std::set<int> listed_indices;

    for (size_t i = 0; i < lines.size() - 1; i++) {
      std::vector<int> parallel_indices = parallel_map[i];
      if (listed_indices.find(i) == listed_indices.end()) {
        if (parallel_indices.size() == 0) {
          // nothing to do
        }
        else {
          std::set<int> new_parallel_set;
          jsk_recognition_utils::buildGroupFromGraphMap(parallel_map,
                                 i,
                                 parallel_indices,
                                 new_parallel_set);
          parallel_groups_list.push_back(new_parallel_set);
          jsk_recognition_utils::addSet(listed_indices, new_parallel_set);
        }
      }
    }
    // publish the result
    publishResult(parallel_groups_list, input_indices, input_coefficients);
    publishResultAsCluser(parallel_groups_list, input_indices,
                          input_coefficients);
  }

  void ParallelEdgeFinder::publishResult(
    const std::vector<std::set<int> >& parallel_groups_list,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& input_coefficients)
  {
    jsk_recognition_msgs::ParallelEdgeArray ros_output_msg;
    ros_output_msg.header = input_indices->header;
    for (size_t i = 0; i < parallel_groups_list.size(); i++) {
      std::set<int> parallel_groups = parallel_groups_list[i];
      jsk_recognition_msgs::ParallelEdge edge_group;
      edge_group.header = input_indices->header;
      for (std::set<int>::iterator it = parallel_groups.begin();
           it != parallel_groups.end();
           ++it) {
        int the_index = *it;
        std::vector<int> indices
          = input_indices->cluster_indices[the_index].indices;
        std::vector<float> coefficients
          = input_coefficients->coefficients[the_index].values;
        PCLIndicesMsg indices_msg;
        indices_msg.header = input_indices->header;
        indices_msg.indices = indices;
        PCLModelCoefficientMsg coefficients_msg;
        coefficients_msg.header = input_coefficients->header;
        coefficients_msg.values = coefficients;
        edge_group.cluster_indices.push_back(indices_msg);
        edge_group.coefficients.push_back(coefficients_msg);
      }
      ros_output_msg.edge_groups.push_back(edge_group);
    }
    pub_.publish(ros_output_msg);
  }

  void ParallelEdgeFinder::publishResultAsCluser(
      const std::vector<std::set<int> >& parallel_groups_list,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& input_coefficients)
  {
    jsk_recognition_msgs::ClusterPointIndices ros_output_msg;
    ros_output_msg.header = input_indices->header;

    for (size_t i = 0; i < parallel_groups_list.size(); i++) {
      std::set<int> parallel_groups = parallel_groups_list[i];
      PCLIndicesMsg indices_msg;
      indices_msg.header = input_indices->header;
      for (std::set<int>::iterator it = parallel_groups.begin();
           it != parallel_groups.end();
           ++it) {
        indices_msg.indices = jsk_recognition_utils::addIndices(
          indices_msg.indices,
          input_indices->cluster_indices[*it].indices);
      }
      ros_output_msg.cluster_indices.push_back(indices_msg);
    }
    pub_clusters_.publish(ros_output_msg);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ParallelEdgeFinder, nodelet::Nodelet);

