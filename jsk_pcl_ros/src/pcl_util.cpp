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

  void buildGroupFromGraphMap(std::map<int, std::vector<int> > graph_map,
                              const int from_index,
                              std::vector<int>& to_indices,
                              std::set<int>& output_set)
  {
    output_set.insert(from_index);
    for (size_t i = 0; i < to_indices.size(); i++) {
      int to_index = to_indices[i];
      if (output_set.find(to_index) == output_set.end()) {
        output_set.insert(to_index);
        std::vector<int> next_indices = graph_map[to_index];
        buildGroupFromGraphMap(graph_map,
                               to_index,
                               next_indices,
                               output_set);
      }
    }
  }
}

