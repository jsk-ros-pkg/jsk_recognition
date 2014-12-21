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

#include "jsk_pcl_ros/linemod_trainer.h"
#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>
#include <pcl/filters/extract_indices.h>
namespace jsk_pcl_ros
{
  void LINEMODTrainer::onInit()
  {
    PCLNodelet::onInit();
    pnh_->param("output_file", output_file_, std::string("template.lmt"));
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input/indices", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_indices_);
    sync_->registerCallback(boost::bind(&LINEMODTrainer::store,
                                        this, _1, _2));

    start_training_srv_
      = pnh_->advertiseService(
        "start_training", &LINEMODTrainer::startTraining,
        this);
    clear_data_srv_
      = pnh_->advertiseService(
        "clear", &LINEMODTrainer::clearData,
        this);
  }

  void LINEMODTrainer::store(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const PCLIndicesMsg::ConstPtr& indices_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    pcl_conversions::toPCL(*indices_msg, *indices);
    samples_.push_back(cloud);
    sample_indices_.push_back(indices);
    ROS_INFO("%lu samples", samples_.size());
  }

  bool LINEMODTrainer::clearData(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("clearing %lu samples", samples_.size());
    samples_.clear();
    sample_indices_.clear();
    return true;
  }
  
  bool LINEMODTrainer::startTraining(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("Start LINEMOD training from %lu samples", samples_.size());
    pcl::LINEMOD linemod;
    for (size_t i = 0; i < samples_.size(); i++) {
      ROS_INFO("Processing %lu-th data", i);
      pcl::PointIndices::Ptr mask = sample_indices_[i];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = samples_[i];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr masked_cloud
        (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::ExtractIndices<pcl::PointXYZRGBA> ex;
      ex.setKeepOrganized(true);
      ex.setInputCloud(cloud);
      ex.setIndices(mask);
      ex.filter(*masked_cloud);

      pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
      color_grad_mod.setInputCloud(masked_cloud);
      color_grad_mod.processInputData();
      pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
      surface_norm_mod.setInputCloud(masked_cloud);
      surface_norm_mod.processInputData();
      std::vector<pcl::QuantizableModality*> modalities(2);
      modalities[0] = &color_grad_mod;
      modalities[1] = &surface_norm_mod;
      
      size_t min_x(masked_cloud->width), min_y(masked_cloud->height), max_x(0), max_y(0);
      pcl::MaskMap mask_map(masked_cloud->width, masked_cloud->height);
      for (size_t j = 0; j < masked_cloud->height; ++j) {
        for (size_t i = 0; i < masked_cloud->width; ++i) {
          pcl::PointXYZRGBA p
            = masked_cloud->points[j * masked_cloud->width + i];
          if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
            mask_map(i, j) = 1;
            min_x = std::min(min_x, i);
            max_x = std::max(max_x, i);
            min_y = std::min(min_y, j);
            max_y = std::max(max_y, j);
          }
          else {
            mask_map(i, j) = 0;
          }
        }
      }
      std::vector<pcl::MaskMap*> masks(2);
      masks[0] = &mask_map;
      masks[1] = &mask_map;
      pcl::RegionXY region;
      region.x = static_cast<int>(min_x);
      region.y = static_cast<int>(min_y);
      region.width = static_cast<int>(max_x - min_x + 1);
      region.height = static_cast<int>(max_y - min_y + 1);
      linemod.createAndAddTemplate(modalities, masks, region);
    }
    // dump template
    ROS_INFO("Dump %lu trained data into %s", linemod.getNumOfTemplates(),
             output_file_.c_str());
    std::ofstream file_stream;
    file_stream.open(output_file_.c_str(),
                      std::ofstream::out | std::ofstream::binary);
    linemod.serialize(file_stream);
    file_stream.close();
    return true;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LINEMODTrainer, nodelet::Nodelet);
