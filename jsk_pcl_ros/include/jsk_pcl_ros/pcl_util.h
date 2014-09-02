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

#ifndef JSK_PCL_ROS_PCL_UTIL_H_
#define JSK_PCL_ROS_PCL_UTIL_H_

#include <pcl/point_types.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>

#include <set>
#include <map>

namespace jsk_pcl_ros
{
  std::vector<int> addIndices(const std::vector<int>& a,
                              const std::vector<int>& b);

  class Counter
  {
  public:
    typedef boost::accumulators::accumulator_set<
    double,
    boost::accumulators::stats<boost::accumulators::tag::count,
                               boost::accumulators::tag::mean,
                               boost::accumulators::tag::min,
                               boost::accumulators::tag::max,
                               boost::accumulators::tag::variance> > Accumulator;
    virtual void add(double v);
    virtual double mean();
    virtual double min();
    virtual double max();
    virtual int count();
    virtual double variance();
  protected:
    Accumulator acc_;
  };

  ////////////////////////////////////////////////////////
  // Graph utility function
  ////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////
  // buildGroupFromGraphMap (recursive function)
  //   This function retrieves a directional graph, and build
  //   a set of the indices where can be arrived from a specified
  //   vertex.
  // 
  //   graph_map := A map representeing edges of the graph.
  //                The graph is one-directional graph.
  //                the key means "from vertex" and the value
  //                means the "to indices" from the key vertex.
  //   from_index := The index to pay attension
  //   to_indices := The "to indices" from from_index
  //   output_set := result
  ////////////////////////////////////////////////////////
  void buildGroupFromGraphMap(std::map<int, std::vector<int> > graph_map,
                              const int from_index,
                              std::vector<int>& to_indices,
                              std::set<int>& output_set);
  
  template <class T>
  void addSet(std::set<T>& output,
              const std::set<T>& new_set)
  {
    typedef typename std::set<T> Set;
    typedef typename Set::iterator Iterator;
    for (Iterator it = new_set.begin();
         it != new_set.end();
         ++it) {
      output.insert(*it);
    }
  }
}

#endif
