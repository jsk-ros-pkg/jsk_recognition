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


#ifndef JSK_PCL_ROS_ONE_DATA_STAT_H_
#define JSK_PCL_ROS_ONE_DATA_STAT_H_

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>

namespace jsk_pcl_ros
{
  /**
   * @brief
   * class to store sensor value and compute mean, error and stddev and so on.
   */
  class OneDataStat
  {
  public:
    typedef boost::shared_ptr<OneDataStat> Ptr;
    typedef boost::accumulators::accumulator_set<
      double,
      boost::accumulators::stats<boost::accumulators::tag::count,
                                 boost::accumulators::tag::mean,
                                 boost::accumulators::tag::min,
                                 boost::accumulators::tag::max,
                                 boost::accumulators::tag::variance> > Accumulator;
    virtual void addData(float r)
      {
        acc_(r);
      }

    virtual double mean() const { return boost::accumulators::mean(acc_); }
    virtual double min() const { return boost::accumulators::min(acc_); }
    virtual double max() const { return boost::accumulators::max(acc_); }
    virtual double count() const { return boost::accumulators::count(acc_); }
    virtual double variance() const { return boost::accumulators::variance(acc_); }
    virtual double stddev() const { return sqrt(variance()); }
  
  protected:
    Accumulator acc_;
  private:
  };

/**
 * @brief
 * wrapper function for mean method for boost::function
 */
  double mean(const OneDataStat& d) 
  {
    return d.mean();
  }

/**
 * @brief
 * wrapper function for min method for boost::function
 */
  double min(const OneDataStat& d) 
  {
    return d.min();
  }

/**
 * @brief
 * wrapper function for max method for boost::function
 */
  double max(const OneDataStat& d) 
  {
    return d.max();
  }

/**
 * @brief
 * wrapper function for count method for boost::function
 */
  double count(const OneDataStat& d)
  {
    return d.count();
  }

/**
 * @brief
 * wrapper function for variance method for boost::function
 */
  double variance(const OneDataStat& d)
  {
    return d.variance();
  }

/**
 * @brief
 * wrapper function for stddev method for boost::function
 */
  double stddev(const OneDataStat& d)
  {
    return d.stddev();
  }

}

#endif
