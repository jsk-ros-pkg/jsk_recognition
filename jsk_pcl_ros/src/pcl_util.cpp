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

#include "jsk_pcl_ros/pcl_util.h"
#include <set>
#include <algorithm>

namespace jsk_pcl_ros
{
  std::vector<int> addIndices(const std::vector<int>& a,
                              const std::vector<int>& b)
  {
    std::set<int> all(b.begin(), b.end());
    for (size_t i = 0; i < a.size(); i++) {
      all.insert(a[i]);
    }
    return std::vector<int>(all.begin(), all.end());
  }

  pcl::PointIndices::Ptr addIndices(const pcl::PointIndices& a,
                                    const pcl::PointIndices& b)
  {
    std::vector<int> indices = addIndices(a.indices, b.indices);
    pcl::PointIndices::Ptr ret(new pcl::PointIndices);
    ret->indices = indices;
    return ret;
  }
  
  std_msgs::ColorRGBA colorCategory20(int i)
  {
    std_msgs::ColorRGBA c;
    c.a = 1.0;
    switch (i % 20) {
    case 0:
    {
      c.r = 0.121569;
      c.g = 0.466667;
      c.b = 0.705882;
    }
    break;
    case 1:
    {
      c.r = 0.682353;
      c.g = 0.780392;
      c.b = 0.909804;
    }
    break;
    case 2:
    {
      c.r = 1.000000;
      c.g = 0.498039;
      c.b = 0.054902;
    }
    break;
    case 3:
    {
      c.r = 1.000000;
      c.g = 0.733333;
      c.b = 0.470588;
    }
    break;
    case 4:
    {
      c.r = 0.172549;
      c.g = 0.627451;
      c.b = 0.172549;
    }
    break;
    case 5:
    {
      c.r = 0.596078;
      c.g = 0.874510;
      c.b = 0.541176;
    }
    break;
    case 6:
    {
      c.r = 0.839216;
      c.g = 0.152941;
      c.b = 0.156863;
    }
    break;
    case 7:
    {
      c.r = 1.000000;
      c.g = 0.596078;
      c.b = 0.588235;
    }
    break;
    case 8:
    {
      c.r = 0.580392;
      c.g = 0.403922;
      c.b = 0.741176;
    }
    break;
    case 9:
    {
      c.r = 0.772549;
      c.g = 0.690196;
      c.b = 0.835294;
    }
    break;
    case 10:
    {
      c.r = 0.549020;
      c.g = 0.337255;
      c.b = 0.294118;
    }
    break;
    case 11:
    {
      c.r = 0.768627;
      c.g = 0.611765;
      c.b = 0.580392;
    }
    break;
    case 12:
    {
      c.r = 0.890196;
      c.g = 0.466667;
      c.b = 0.760784;
    }
    break;
    case 13:
    {
      c.r = 0.968627;
      c.g = 0.713725;
      c.b = 0.823529;
    }
    break;
    case 14:
    {
      c.r = 0.498039;
      c.g = 0.498039;
      c.b = 0.498039;
    }
    break;
    case 15:
    {
      c.r = 0.780392;
      c.g = 0.780392;
      c.b = 0.780392;
    }
    break;
    case 16:
    {
      c.r = 0.737255;
      c.g = 0.741176;
      c.b = 0.133333;
    }
    break;
    case 17:
    {
      c.r = 0.858824;
      c.g = 0.858824;
      c.b = 0.552941;
    }
    break;
    case 18:
    {
      c.r = 0.090196;
      c.g = 0.745098;
      c.b = 0.811765;
    }
    break;
    case 19:
    {
      c.r = 0.619608;
      c.g = 0.854902;
      c.b = 0.898039;
    }
    break;
    }
    return c;
  }
  
  void Counter::add(double v)
  {
    acc_(v);
  }
  
  double Counter::mean()
  {
    return boost::accumulators::mean(acc_);
  }

  double Counter::min()
  {
    return boost::accumulators::min(acc_);
  }

  double Counter::max()
  {
    return boost::accumulators::max(acc_);
  }

  int Counter::count()
  {
    return boost::accumulators::count(acc_);
  }
  
  double Counter::variance()
  {
    return boost::accumulators::variance(acc_);
  }

  void buildGroupFromGraphMap(IntegerGraphMap graph_map,
                              const int from_index_arg,
                              std::vector<int>& to_indices_arg,
                              std::set<int>& output_set)
  {
    // convert graph_map into one-directional representation
    IntegerGraphMap onedirectional_map(graph_map);
    for (IntegerGraphMap::iterator it = onedirectional_map.begin();
         it != onedirectional_map.end();
         ++it) {
      int from_index = it->first;
      std::vector<int> to_indices = it->second;
      for (size_t i = 0; i < to_indices.size(); i++) {
        int to_index = to_indices[i];
        if (onedirectional_map.find(to_index) == onedirectional_map.end()) {
          // not yet initialized
          onedirectional_map[to_index] = std::vector<int>(); 
        }
        if (std::find(onedirectional_map[to_index].begin(),
                      onedirectional_map[to_index].end(),
                      from_index) == onedirectional_map[to_index].end()) {
          onedirectional_map[to_index].push_back(from_index);
        }
      }
    }
    _buildGroupFromGraphMap(onedirectional_map,
                            from_index_arg,
                            to_indices_arg,
                            output_set);
  }
  
  void _buildGroupFromGraphMap(IntegerGraphMap graph_map,
                               const int from_index,
                               std::vector<int>& to_indices,
                               std::set<int>& output_set)
  {    
    output_set.insert(from_index);
    for (size_t i = 0; i < to_indices.size(); i++) {
      int to_index = to_indices[i];
      if (output_set.find(to_index) == output_set.end()) {
        output_set.insert(to_index);
        //std::cout << "__connection__: " << from_index << " --> " << to_index << std::endl;
        std::vector<int> next_indices = graph_map[to_index];
        _buildGroupFromGraphMap(graph_map,
                               to_index,
                               next_indices,
                               output_set);
      }
    }
  }

  void buildAllGroupsSetFromGraphMap(IntegerGraphMap graph_map,
                                     std::vector<std::set<int> >& output_sets)
  {
    std::set<int> duplication_check_set;
    for (IntegerGraphMap::iterator it = graph_map.begin();
         it != graph_map.end();
         ++it) {
      int from_index = it->first;
      if (duplication_check_set.find(from_index)
          == duplication_check_set.end()) {
        std::set<int> new_graph_set;
        buildGroupFromGraphMap(graph_map, from_index, it->second,
                               new_graph_set);
        output_sets.push_back(new_graph_set);
        // update duplication_check_set
        addSet<int>(duplication_check_set, new_graph_set);
      }
    }
  }

  void addDiagnosticInformation(
    const std::string& string_prefix,
    jsk_topic_tools::TimeAccumulator& accumulator,
    diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.add(string_prefix + " (Avg.)", accumulator.mean());
    stat.add(string_prefix + " (Max)", accumulator.max());
    stat.add(string_prefix + " (Min)", accumulator.min());
    stat.add(string_prefix + " (Var.)", accumulator.variance());
  }

  //static boost::mutex global_chull_mutex;
  
}

