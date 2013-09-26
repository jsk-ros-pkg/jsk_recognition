/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "jsk_pcl_ros/cluster_point_indices_decomposer_z_axis.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void ClusterPointIndicesDecomposerZAxis::sortIndicesOrder
  (pcl::PointCloud<pcl::PointXYZ>::Ptr input,
   std::vector<pcl::IndicesPtr> indices_array,
   std::vector<pcl::IndicesPtr> &output_array)
  {
    output_array.resize(indices_array.size());
    std::vector<double> z_values;
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(input);
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      Eigen::Vector4f center;
      ex.setIndices(indices_array[i]);
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
      ex.filter(*tmp);
      pcl::compute3DCentroid(*tmp, center);
      z_values.push_back(center[2]); // only focus on z value
    }
    
    // sort centroids
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      size_t minimum_index = 0;
      double minimum_value = DBL_MAX;
      for (size_t j = 0; j < indices_array.size(); j++)
      {
        if (z_values[j] < minimum_value)
        {
          minimum_value = z_values[j];
          minimum_index = j;
        }
      }
      // ROS_INFO("%lu => %lu", i, minimum_index);
      output_array[i] = indices_array[minimum_index];
      z_values[minimum_index] = DBL_MAX;
    }
  }
}


typedef jsk_pcl_ros::ClusterPointIndicesDecomposerZAxis ClusterPointIndicesDecomposerZAxis;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ClusterPointIndicesDecomposerZAxis, ClusterPointIndicesDecomposerZAxis, nodelet::Nodelet);
