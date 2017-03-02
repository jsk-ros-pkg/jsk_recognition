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
#include "jsk_pcl_ros/grid_sampler.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/common/common.h>

#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  void GridSampler::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&GridSampler::configCallback, this, _1, _2);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    srv_->setCallback (f);
    onInitPostProcess();
  }

  void GridSampler::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &GridSampler::sample, this);
  }
  
  void GridSampler::unsubscribe()
  {
    sub_.shutdown();
  }
  
  void GridSampler::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    if (config.grid_size == 0.0) {
      NODELET_WARN("grid_size == 0.0 is prohibited");
      return;
    }
    else {
      grid_size_ = config.grid_size;
      min_indices_ = config.min_indices;
    }
  }

  void GridSampler::sample(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*pcl_cloud, minpt, maxpt);
    int x_bin_num = ceil((maxpt[0] - minpt[0]) / grid_size_);
    int y_bin_num = ceil((maxpt[1] - minpt[1]) / grid_size_);
    int z_bin_num = ceil((maxpt[2] - minpt[2]) / grid_size_);

    //        x             y             z
    std::map<int, std::map<int, std::map<int, std::vector<size_t > > > > grid;
    for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
      pcl::PointXYZRGB point = pcl_cloud->points[i];
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        // skip nan
        continue;
      }
      int xbin = int((point.x - minpt[0]) / grid_size_);
      int ybin = int((point.y - minpt[1]) / grid_size_);
      int zbin = int((point.z - minpt[2]) / grid_size_);
      std::map<int, std::map<int, std::map<int, std::vector<size_t> > > >::iterator xit
        = grid.find(xbin);
      if (xit == grid.end()) {  // cannot find x bin
        NODELET_DEBUG_STREAM("no x bin" << xbin);
        std::map<int, std::vector<size_t> > new_z;
        std::vector<size_t> new_indices;
        new_indices.push_back(i);
        new_z[zbin] = new_indices;
        std::map<int, std::map<int, std::vector<size_t> > > new_y;
        new_y[ybin] = new_z;
        grid[xbin] = new_y;
      }
      else {
        NODELET_DEBUG_STREAM("found x bin" << xbin);
        std::map<int, std::map<int, std::vector<size_t> > > ybins = xit->second;
        std::map<int, std::map<int, std::vector<size_t> > >::iterator yit
          = ybins.find(ybin);
        if (yit == ybins.end()) { // cannot find y bin
          NODELET_DEBUG_STREAM("no y bin" << ybin);
          std::map<int, std::vector<size_t> > new_z;
          std::vector<size_t> new_indices;
          new_indices.push_back(i);
          new_z[zbin] = new_indices;
          xit->second[ybin] = new_z;
        }
        else {
          NODELET_DEBUG_STREAM("found y bin" << ybin);
          std::map<int, std::vector<size_t> > zbins = yit->second;
          std::map<int, std::vector<size_t> >::iterator zit
            = zbins.find(zbin);
          if (zit == zbins.end()) {
            NODELET_DEBUG_STREAM("no z bin" << zbin);
            std::vector<size_t> new_indices;
            new_indices.push_back(i);
            xit->second[ybin][zbin] = new_indices;
          }
          else {
            NODELET_DEBUG_STREAM("found z bin" << zbin);
            xit->second[ybin][zbin].push_back(i);
          }
        }
      }
    }
    // publish the result
    jsk_recognition_msgs::ClusterPointIndices output;
    output.header = msg->header;
    for (std::map<int, std::map<int, std::map<int, std::vector<size_t> > > >::iterator xit = grid.begin();
         xit != grid.end();
         xit++) {
      std::map<int, std::map<int, std::vector<size_t> > > ybins = xit->second;
      for (std::map<int, std::map<int, std::vector<size_t> > >::iterator yit = ybins.begin();
           yit != ybins.end();
           yit++) {
        std::map<int, std::vector<size_t> > zbins = yit->second;
        for (std::map<int, std::vector<size_t> >::iterator zit = zbins.begin();
             zit != zbins.end();
             zit++) {
          std::vector<size_t> indices = zit->second;
          NODELET_DEBUG_STREAM("size: " << indices.size());
          if (indices.size() > min_indices_) {
            PCLIndicesMsg ros_indices;
            ros_indices.header = msg->header;
            for (size_t j = 0; j < indices.size(); j++) {
              ros_indices.indices.push_back(indices[j]);
            }
            output.cluster_indices.push_back(ros_indices);
          }
        }
      }
    }
    pub_.publish(output);
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::GridSampler, nodelet::Nodelet);

