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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/uniform_sampling.h"
#if ( PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7)
#include <pcl/keypoints/uniform_sampling.h>
#elif ( PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 8)
#include <pcl/filters/uniform_sampling.h>
#endif


namespace jsk_pcl_ros
{
  void UniformSampling::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&UniformSampling::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<PCLIndicesMsg>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void UniformSampling::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &UniformSampling::sampling, this);
  }

  void UniformSampling::unsubscribe()
  {
    sub_.shutdown();
  }

  void UniformSampling::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    search_radius_ = config.search_radius;
  }

  void UniformSampling::sampling(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(search_radius_);
#if ( PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7)
    pcl::PointCloud<int> indices;
    uniform_sampling.compute(indices);
#elif ( PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 8)
    pcl::PointCloud<pcl::PointXYZ> output;
    uniform_sampling.filter(output);
    pcl::PointIndicesPtr indices_ptr;
    std::vector<int> &indices = *uniform_sampling.getIndices();
#endif
    PCLIndicesMsg ros_indices;
#if ( PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7)
    for (size_t i = 0; i < indices.points.size(); i++) {
      ros_indices.indices.push_back(indices.points[i]);
    }
#elif ( PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 8)
    for (size_t i = 0; i < indices.size(); i++) {
      ros_indices.indices.push_back(indices[i]);
    }
#endif
    ros_indices.header = msg->header;
    pub_.publish(ros_indices);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::UniformSampling, nodelet::Nodelet);
