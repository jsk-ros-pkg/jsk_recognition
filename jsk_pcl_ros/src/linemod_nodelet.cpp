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
#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_pcl_ros/linemod.h"
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <glob.h>
#include <boost/algorithm/string/predicate.hpp>
#include <pcl/common/transforms.h>

namespace jsk_pcl_ros
{
  void LINEMODTrainer::onInit()
  {
    PCLNodelet::onInit();
    pnh_->param("output_file", output_file_, std::string("template.lmt"));
    pnh_->param("rotation_quantization", rotation_quantization_, 1);
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
    NODELET_INFO("%lu samples", samples_.size());
  }

  bool LINEMODTrainer::clearData(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_INFO("clearing %lu samples", samples_.size());
    samples_.clear();
    sample_indices_.clear();
    return true;
  }
  
  bool LINEMODTrainer::startTraining(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_INFO("Start LINEMOD training from %lu samples", samples_.size());
    pcl::LINEMOD linemod;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> masked_clouds;
    for (size_t i = 0; i < samples_.size(); i++) {
      NODELET_INFO("Processing %lu-th data", i);
      pcl::PointIndices::Ptr mask = sample_indices_[i];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud = samples_[i];
      for (size_t j = 0; j < rotation_quantization_; j++) {
        double theta = 2 * M_PI / rotation_quantization_ * j;
        Eigen::Affine3f transform
          = Eigen::Affine3f::Identity() *
          Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
          cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::transformPointCloud<pcl::PointXYZRGBA>(*raw_cloud,
                                                    *cloud,
                                                    transform);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr masked_cloud
          (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::ExtractIndices<pcl::PointXYZRGBA> ex;
        ex.setKeepOrganized(true);
        ex.setInputCloud(cloud);
        ex.setIndices(mask);
        ex.filter(*masked_cloud);
        masked_clouds.push_back(masked_cloud);
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
    }
    // dump template
    // lmt file is a tar file
    //   tar --format=ustar -cf model.lmt ./*template*
    // 1. mkdir template directory
    // 2. dump templates into the directory as template_&05d.pcd
    // 3. call tar
    NODELET_INFO("Dump %lu trained data into %s", linemod.getNumOfTemplates(),
             output_file_.c_str());
    
    boost::filesystem::path temp = boost::filesystem::unique_path();
    const std::string tempstr = temp.native();
    NODELET_INFO("mkdir %s", tempstr.c_str());
    boost::filesystem::create_directory(temp);
    std::stringstream command_stream;
    command_stream << "tar --format=ustar -cf " << output_file_;
    for (size_t i = 0; i < linemod.getNumOfTemplates(); i++) {
      {
        // sqmmt
        std::stringstream filename_stream;
        filename_stream << boost::format("%s/%05lu_template.sqmmt") % tempstr % i;
        std::string filename = filename_stream.str();
        NODELET_INFO("writing %s", filename.c_str());
        std::ofstream file_stream;
        file_stream.open(filename.c_str(),
                         std::ofstream::out | std::ofstream::binary);
        linemod.getTemplate(i).serialize(file_stream);
        file_stream.close();
        command_stream << " " << filename;
      }
      {
        // pcd
        std::stringstream filename_stream;
        filename_stream << boost::format("%s/%05lu_template.pcd") % tempstr % i;
        std::string filename = filename_stream.str();
        NODELET_INFO("writing %s", filename.c_str());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = masked_clouds[i];
        pcl::PCDWriter writer;
        writer.writeBinaryCompressed(filename, *cloud);
        command_stream << " " << filename;
      }
    }
    NODELET_INFO("executing %s", command_stream.str().c_str());
    int ret = system(command_stream.str().c_str());
    NODELET_INFO("done");
    return true;
  }

  void LINEMODDetector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("use_raw_templates", use_raw_templates_, false);
    pnh_->param("minimum_template_points", minimum_template_points_, 3000);
    if (!use_raw_templates_) {
      pnh_->param("template_file", template_file_, std::string("template.ltm"));
    }
    else {
      std::string template_files;
      pnh_->param("template_files", template_files, std::string("template"));
      // glob files under template_directory
      glob_t glob_result;
      glob(template_files.c_str(), GLOB_TILDE, NULL, &glob_result);
      std::vector<std::string> template_pcd_files;
      std::vector<std::string> template_sqmmt_files;
      // PCD file driven
      for (unsigned int i=0;i<glob_result.gl_pathc;++i) {
        std::string file(glob_result.gl_pathv[i]);
        NODELET_INFO("file: %s", file.c_str());
        if (boost::algorithm::ends_with(file, ".pcd")) {
          // check sqmmt file exists or not
          std::string pcd_file = file;
          std::string sqmmt_file = boost::algorithm::replace_all_copy(file, ".pcd", ".sqmmt");
          if (boost::filesystem::exists(sqmmt_file)) {
            template_pcd_files.push_back(pcd_file);
            template_sqmmt_files.push_back(sqmmt_file);
          }
          else {
            NODELET_WARN("cannot find %s", sqmmt_file.c_str());
          }
        }
      }
      // error check
      if (template_pcd_files.size() == 0) {
        NODELET_FATAL("no pcd/sqmmt files is found");
        return;
      }
      template_pointclouds_.resize(template_pcd_files.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (size_t i = 0; i < template_pointclouds_.size(); i++) {
        NODELET_INFO("reading %s", template_pcd_files[i].c_str());
        pcl::PCDReader reader;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
          ref(new pcl::PointCloud<pcl::PointXYZRGBA>);
        reader.read(template_pcd_files[i], *ref);
        // count up non-nan points
        int valid_points = 0;
        for (size_t j = 0; j < ref->points.size(); j++) {
          pcl::PointXYZRGBA p = ref->points[j];
          if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)) {
            ++valid_points;
          }
        }
        NODELET_INFO("%s -- %d", template_pcd_files[i].c_str(), valid_points);
        if (valid_points > minimum_template_points_) {
          template_pointclouds_[i] = ref;
        }
      }
      template_sqmmts_.resize(template_sqmmt_files.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (size_t i = 0; i < template_sqmmts_.size(); i++) {
        NODELET_INFO("reading %s", template_sqmmt_files[i].c_str());
        std::ifstream file_stream(template_sqmmt_files[i].c_str());
        pcl::SparseQuantizedMultiModTemplate sqmmt;
        sqmmt.deserialize(file_stream);
        template_sqmmts_[i] = sqmmt;
      }
          
    }
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &LINEMODDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    
  }

  void LINEMODDetector::subscribe()
  {
    sub_cloud_ = pnh_->subscribe("input", 1, &LINEMODDetector::detect, this);
  }

  void LINEMODDetector::unsubscribe()
  {
    sub_cloud_.shutdown();
  }
  
  void LINEMODDetector::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    line_rgbd_ = pcl::LineRGBD<pcl::PointXYZRGBA>();
    gradient_magnitude_threshold_ = config.gradient_magnitude_threshold;
    detection_threshold_ = config.detection_threshold;
    line_rgbd_.setGradientMagnitudeThreshold(gradient_magnitude_threshold_);
    line_rgbd_.setDetectionThreshold(detection_threshold_);
    // load linemod templates
    if (!use_raw_templates_) {
      NODELET_INFO("loading LINEMOD templates from %s", template_file_.c_str());
      line_rgbd_.loadTemplates(template_file_);
      NODELET_INFO("done");
    }
    else {
      for (size_t i = 0; i < template_pointclouds_.size(); i++) {
        if (template_pointclouds_[i]) {
          NODELET_INFO("adding %lu template", i);
          line_rgbd_.addTemplate(template_sqmmts_[i], 
                                 *template_pointclouds_[i]);
        }
      }
      NODELET_INFO("done");
    }
  }

  void LINEMODDetector::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "LINEMODDetector running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "LINEMODDetector", vital_checker_, stat);
    }
  }
  
  void LINEMODDetector::detect(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    NODELET_INFO("detect");
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    line_rgbd_.setInputCloud(cloud);
    line_rgbd_.setInputColors(cloud);

    std::vector<pcl::LineRGBD<pcl::PointXYZRGBA>::Detection> detections;
    //line_rgbd_.detect(detections);
    line_rgbd_.detectSemiScaleInvariant(detections);
    NODELET_INFO("detected %lu result", detections.size());
    // lookup the best result
    if (detections.size() > 0) {
      double max_response = 0;
      size_t max_object_id = 0;
      size_t max_template_id = 0;
      size_t max_detection_id = 0;
      for (size_t i = 0; i < detections.size(); i++) {
        pcl::LineRGBD<pcl::PointXYZRGBA>::Detection detection = detections[i];
        if (max_response < detection.response) {
          max_response = detection.response;
          max_object_id = detection.object_id;
          max_template_id = detection.template_id;
          max_detection_id = detection.detection_id;
        }
      }
      NODELET_INFO("(%lu, %lu)", max_object_id, max_template_id);
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
        result (new pcl::PointCloud<pcl::PointXYZRGBA>);
      line_rgbd_.computeTransformedTemplatePoints(max_detection_id,
                                                  *result);
      sensor_msgs::PointCloud2 ros_result;
      pcl::toROSMsg(*result, ros_result);
      ros_result.header = cloud_msg->header;
      pub_cloud_.publish(ros_result);
    }
  }
  
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LINEMODTrainer, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LINEMODDetector, nodelet::Nodelet);
