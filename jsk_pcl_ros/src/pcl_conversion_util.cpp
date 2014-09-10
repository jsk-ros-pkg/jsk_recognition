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

#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace pcl_conversions
{
  std::vector<pcl::PointIndices::Ptr>
  convertToPCLPointIndices(
    const std::vector<PCLIndicesMsg>& cluster_indices)
  {
    std::vector<pcl::PointIndices::Ptr> ret;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      std::vector<int> indices = cluster_indices[i].indices;
      pcl::PointIndices::Ptr pcl_indices (new pcl::PointIndices);
      pcl_indices->indices = indices;
      ret.push_back(pcl_indices);
    }
    return ret;
  }

  std::vector<pcl::ModelCoefficients::Ptr>
  convertToPCLModelCoefficients(
    const std::vector<PCLModelCoefficientMsg>& coefficients)
  {
    std::vector<pcl::ModelCoefficients::Ptr> ret;
    for (size_t i = 0; i < coefficients.size(); i++) {
      pcl::ModelCoefficients::Ptr pcl_coefficients (new pcl::ModelCoefficients);
      pcl_coefficients->values = coefficients[i].values;
      ret.push_back(pcl_coefficients);
    }
    return ret;
  }
  
}
