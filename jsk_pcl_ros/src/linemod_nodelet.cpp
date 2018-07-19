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
#define BOOST_PARAMETER_MAX_ARITY 7

#include <limits>
#include "jsk_pcl_ros/linemod.h"
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <glob.h>
#include <boost/algorithm/string/predicate.hpp>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include "jsk_pcl_ros/viewpoint_sampler.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>
#include <pcl/common/common.h>
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_pcl_ros
{
  void LINEMODTrainer::onInit()
  {
    PCLNodelet::onInit();
    pnh_->param("output_file", output_file_, std::string("template"));
    pnh_->param("sample_viewpoint", sample_viewpoint_, true);
    pnh_->param("sample_viewpoint_angle_step", sample_viewpoint_angle_step_,
                40.0);
    pnh_->param("sample_viewpoint_angle_min", sample_viewpoint_angle_min_,
                -80.0);
    pnh_->param("sample_viewpoint_angle_max", sample_viewpoint_angle_max_,
                80.0);
    pnh_->param("sample_viewpoint_radius_step", sample_viewpoint_radius_step_,
                0.2);
    pnh_->param("sample_viewpoint_radius_min", sample_viewpoint_radius_min_,
                0.4);
    pnh_->param("sample_viewpoint_radius_max", sample_viewpoint_radius_max_,
                0.8);
    if (!sample_viewpoint_) {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_indices_.subscribe(*pnh_, "input/indices", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_indices_);
      sync_->registerCallback(boost::bind(&LINEMODTrainer::store,
                                          this, _1, _2));
    }
    else {
      sub_input_nonsync_ = pnh_->subscribe("input", 1,
                                           &LINEMODTrainer::subscribeCloud,
                                           this);
      sub_camera_info_nonsync_ = pnh_->subscribe(
        "input/info", 1,
        &LINEMODTrainer::subscribeCameraInfo,
        this);
      pub_range_image_ = pnh_->advertise<sensor_msgs::Image>(
        "output/range_image", 1);
      pub_colored_range_image_ = pnh_->advertise<sensor_msgs::Image>(
          "output/colored_range_image", 1);
      pub_sample_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>(
        "output/sample_cloud", 1);
    }
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

  void LINEMODTrainer::subscribeCloud(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    samples_before_sampling_.push_back(cloud);
    NODELET_INFO("%lu samples", samples_.size());
  }

  void LINEMODTrainer::subscribeCameraInfo(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_ = info_msg;
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

  void LINEMODTrainer::organizedPointCloudWithViewPoint(
    const Eigen::Affine3f& transform,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud,
    const image_geometry::PinholeCameraModel& model,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud,
    pcl::PointIndices& mask)
  {
    int width = model.fullResolution().width;
    int height = model.fullResolution().height;
    double fx = model.fx();
    double fy = model.fy();
    double cx = model.cx();
    double cy = model.cy();
    Eigen::Affine3f viewpoint_transform = transform;
    Eigen::Affine3f object_transform = viewpoint_transform.inverse();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud<pcl::PointXYZRGBA>(
      *raw_cloud, *cloud, object_transform);
    pcl::RangeImagePlanar range_image;
    Eigen::Affine3f dummytrans;
    dummytrans.setIdentity();
    range_image.createFromPointCloudWithFixedSize(
      *cloud, width, height,
      cx, cy, fx, fy, dummytrans);
    cv::Mat mat(range_image.height, range_image.width, CV_32FC1);
    float *tmpf = (float *)mat.ptr();
    for(unsigned int i = 0; i < range_image.height * range_image.width; i++) {
      tmpf[i] = range_image.points[i].z;
    }
    std_msgs::Header dummy_header;
    dummy_header.stamp = ros::Time::now();
    dummy_header.frame_id = camera_info_->header.frame_id;
    cv_bridge::CvImage range_bridge(dummy_header,
                                    "32FC1",
                                    mat);
    pub_range_image_.publish(range_bridge.toImageMsg());
    pcl::KdTreeFLANN<pcl::PointXYZRGBA>::Ptr
      kdtree (new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
    kdtree->setInputCloud(cloud);

    colored_cloud->width = range_image.width;
    colored_cloud->height = range_image.height;
    colored_cloud->is_dense = range_image.is_dense;
    pcl::PointXYZRGBA nan_point;
    nan_point.x = nan_point.y = nan_point.z
      = std::numeric_limits<float>::quiet_NaN();
    pcl::PointIndices::Ptr mask_indices (new pcl::PointIndices);
    colored_cloud->points.resize(range_image.points.size());
    
    cv::Mat colored_image = cv::Mat::zeros(
      range_image.height, range_image.width, CV_8UC3);
    for (size_t pi = 0; pi < range_image.points.size(); pi++) {
      if (std::isnan(range_image.points[pi].x) ||
          std::isnan(range_image.points[pi].y) ||
          std::isnan(range_image.points[pi].z)) {
        // nan
        colored_cloud->points[pi] = nan_point;
      }
      else {
        pcl::PointXYZRGBA input_point;
        input_point.x = range_image.points[pi].x;
        input_point.y = range_image.points[pi].y;
        input_point.z = range_image.points[pi].z;
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree->nearestKSearch(input_point, 1, indices, distances);
        if (indices.size() > 0) {
          input_point.rgba = cloud->points[indices[0]].rgba;
        }
        colored_image.at<cv::Vec3b>(pi / range_image.width,
                                    pi % range_image.width)
          = cv::Vec3b(cloud->points[indices[0]].r,
                      cloud->points[indices[0]].g,
                      cloud->points[indices[0]].b);
        colored_cloud->points[pi] = input_point;
      }
    }
    for (size_t pi = 0; pi < range_image.points.size(); pi++) {
      if (!std::isnan(range_image.points[pi].x) &&
          !std::isnan(range_image.points[pi].y) &&
          !std::isnan(range_image.points[pi].z)) {
        // nan
        mask.indices.push_back(pi);
      }
    }
    cv_bridge::CvImage colored_range_bridge(dummy_header,
                                            sensor_msgs::image_encodings::RGB8,
                                            colored_image);
    pub_colored_range_image_.publish(colored_range_bridge.toImageMsg());
    // // trick, rgba -> rgb
    sensor_msgs::PointCloud2 ros_sample_cloud;
    pcl::toROSMsg(*colored_cloud, ros_sample_cloud);
    pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
    pcl::fromROSMsg(ros_sample_cloud, rgb_cloud);
    pcl::toROSMsg(rgb_cloud, ros_sample_cloud);
    ros_sample_cloud.header = dummy_header;
    pub_sample_cloud_.publish(ros_sample_cloud);
  }
  
  void LINEMODTrainer::trainWithViewpointSampling()
  {
    NODELET_INFO("Start LINEMOD training from %lu samples", samples_before_sampling_.size());
    if (!camera_info_) {
      NODELET_FATAL("no camera info is available");
      return;
    }
    if (samples_before_sampling_.size() != 1) {
      NODELET_FATAL("we expect only one training pointcloud, but it has %lu pointclouds",
                    samples_before_sampling_.size());
      return;
    }
    
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_);
    
    if (samples_before_sampling_.size() != 1) {
      NODELET_FATAL("we expect only one sample data");
      return;
    }
    ViewpointSampler sampler(sample_viewpoint_angle_step_,
                               sample_viewpoint_angle_min_,
                               sample_viewpoint_angle_max_,
                               sample_viewpoint_radius_step_,
                               sample_viewpoint_radius_min_,
                               sample_viewpoint_radius_max_,
                               150);
    std::vector<Eigen::Affine3f> transforms;
    transforms.resize(sampler.sampleNum());
    for (size_t i = 0; i < sampler.sampleNum(); i++) {
      Eigen::Affine3f transform;
      sampler.get(transform);
      transforms[i] = transform;
      sampler.next();
    }
    
    // NB:
    // This line is super important.
    // dummy Range Image to avoid static method initialization in
    // multi-thread environment. In detail,
    // pcl::RangeImagePlanar::createLookupTables is not thread-safe.
    pcl::RangeImagePlanar dummy_range_image;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud
      = samples_before_sampling_[0];

    // prepare for multi-threaded optimization
    std::vector<pcl::ColorGradientModality<pcl::PointXYZRGBA> >
      color_modalities (sampler.sampleNum());
    std::vector<pcl::SurfaceNormalModality<pcl::PointXYZRGBA> >
      surface_norm_modalities (sampler.sampleNum());
    std::vector<pcl::MaskMap> mask_maps (sampler.sampleNum());
    std::vector<pcl::RegionXY> regions (sampler.sampleNum());
    boost::mutex train_mutex;
    pcl::LINEMOD linemod;
    int counter = 0;
    std::vector<Eigen::Affine3f> pose_in_order_of_training;
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t j = 0; j < sampler.sampleNum(); j++) {
      
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      organizedPointCloudWithViewPoint(transforms[j], raw_cloud, model,
                                       cloud, *indices);
      // generate mask and modalities
      pcl::ColorGradientModality<pcl::PointXYZRGBA> color_modality;
      pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
      pcl::MaskMap mask_map(model.fullResolution().width,
                            model.fullResolution().height);
      pcl::RegionXY region;
      generateLINEMODTrainingData(cloud, indices,
                                  color_modality, surface_norm_mod,
                                  mask_map, region);
      std::vector<pcl::QuantizableModality*> modalities(2);
      modalities[0] = &color_modality;
      modalities[1] = &surface_norm_mod;
      std::vector<pcl::MaskMap*> masks(2);
      masks[0] = &mask_map;
      masks[1] = &mask_map;
      {
        boost::mutex::scoped_lock lock(train_mutex);
        ++counter;
        NODELET_INFO("training: %d/%lu", counter, sampler.sampleNum());
        linemod.createAndAddTemplate(modalities, masks, region);
        pose_in_order_of_training.push_back(transforms[j]);
      }
    }
    // dump result into file
    // 1. linemod file
    // 2. original pointcloud into .pcd file
    // 3. pose yaml about the template
    std::ofstream linemod_file;
    const std::string linemod_file_name = output_file_ + ".linemod";
    NODELET_INFO("writing to %s", linemod_file_name.c_str());
    linemod_file.open(linemod_file_name.c_str(),
                      std::ofstream::out | std::ofstream::binary);
    linemod.serialize(linemod_file);
    linemod_file.close();
    const std::string pcd_file_name = output_file_ + ".pcd";
    NODELET_INFO("writing to %s", pcd_file_name.c_str());
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(pcd_file_name, *raw_cloud);
    // pose yaml
    std::ofstream pose_file;
    const std::string pose_file_name = output_file_ + "_poses.yaml";
    pose_file.open(pose_file_name.c_str(), std::ofstream::out);
    pose_file << "template_poses: [" << std::endl;
    for (size_t i = 0; i < pose_in_order_of_training.size(); i++) {
      Eigen::Affine3f pose = pose_in_order_of_training[i];
      Eigen::Vector3f pos(pose.translation());
      Eigen::Quaternionf rot(pose.rotation());
      pose_file << "["
                << pos[0] << ", " << pos[1] << ", " << pos[2] << ", "
                << rot.x() << ", " << rot.y() << ", " << rot.z() << ", " << rot.w() << "]";
      if (i != pose_in_order_of_training.size() - 1) {
        pose_file << ", " << std::endl;
      }
      else {
        pose_file << "]" << std::endl;
      }
    }
    pose_file.close();
    NODELET_INFO("done");
  }

  void LINEMODTrainer::generateLINEMODTrainingData(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
    pcl::PointIndices::Ptr mask,
    pcl::ColorGradientModality<pcl::PointXYZRGBA>& color_grad_mod,
    pcl::SurfaceNormalModality<pcl::PointXYZRGBA>& surface_norm_mod,
    pcl::MaskMap& mask_map,
    pcl::RegionXY& region)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr masked_cloud
      (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ExtractIndices<pcl::PointXYZRGBA> ex;
    ex.setKeepOrganized(true);
    ex.setInputCloud(cloud);
    ex.setIndices(mask);
    ex.filter(*masked_cloud);
    color_grad_mod.setInputCloud(masked_cloud);
    color_grad_mod.processInputData();
    surface_norm_mod.setInputCloud(cloud);
    surface_norm_mod.processInputData();
    size_t min_x(masked_cloud->width), min_y(masked_cloud->height), max_x(0), max_y(0);
    for (size_t j = 0; j < masked_cloud->height; ++j) {
      for (size_t i = 0; i < masked_cloud->width; ++i) {
        pcl::PointXYZRGBA p
          = masked_cloud->points[j * masked_cloud->width + i];
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
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
    region.x = static_cast<int>(min_x);
    region.y = static_cast<int>(min_y);
    region.width = static_cast<int>(max_x - min_x + 1);
    region.height = static_cast<int>(max_y - min_y + 1);
  }
                                   

  std::vector<std::string> LINEMODTrainer::trainOneData(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
    pcl::PointIndices::Ptr mask,
    std::string& tempstr,
    int i)
  {
    pcl::LINEMOD linemod;
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
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
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
    
    std::vector<std::string> ret;
    {
      // sqmmt
      std::stringstream filename_stream;
      filename_stream << boost::format("%s/%05d_template.sqmmt") % tempstr % i;
      std::string filename = filename_stream.str();
      std::cerr << "writing " << filename << std::endl;
      std::ofstream file_stream;
      file_stream.open(filename.c_str(),
                       std::ofstream::out | std::ofstream::binary);
      linemod.getTemplate(0).serialize(file_stream);
      file_stream.close();
      ret.push_back(filename);
    }
    {
      // pcd
      std::stringstream filename_stream;
      filename_stream << boost::format("%s/%05d_template.pcd") % tempstr % i;
      std::string filename = filename_stream.str();
      std::cerr << "writing " << filename << std::endl;
      pcl::PCDWriter writer;
      writer.writeBinaryCompressed(filename, *masked_cloud);
      ret.push_back(filename);
    }
    return ret;
  }
  
  void LINEMODTrainer::trainWithoutViewpointSampling()
  {
    NODELET_INFO("Start LINEMOD training from %lu samples", samples_.size());
    boost::filesystem::path temp = boost::filesystem::unique_path();
    boost::filesystem::create_directory(temp);
    std::string tempstr = temp.native();
    NODELET_INFO("mkdir %s", tempstr.c_str());
    std::vector<std::string> all_files;
    for (size_t i = 0; i < samples_.size(); i++) {
      NODELET_INFO("Processing %lu-th data", i);
      pcl::PointIndices::Ptr mask = sample_indices_[i];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = samples_[i];
      //std::vector<std::string> files = trainOneData(cloud, mask, tempstr, i);
      // for (size_t i = 0; i < files.size(); i++) {
      //   all_files.push_back(files[i]);
      // }
    }
    tar(tempstr, output_file_);
    NODELET_INFO("done");
  }

  void LINEMODTrainer::tar(const std::string& directory, const std::string& output)
  {
    std::stringstream command_stream;
    command_stream << "tar --format=ustar -cf " << output << " " << directory << "/*";
    NODELET_INFO("executing %s", command_stream.str().c_str());
    int ret = system(command_stream.str().c_str());
  }
  
  bool LINEMODTrainer::startTraining(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (sample_viewpoint_) {
      trainWithViewpointSampling();
    }
    else {
      trainWithoutViewpointSampling();
    }
    return true;
  }

  void LINEMODDetector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("template_file", template_file_, std::string("template"));
    // load original point and poses
    template_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCDReader reader;
    reader.read(template_file_ + ".pcd", *template_cloud_);
    const std::string pose_yaml = template_file_ + "_poses.yaml";
    YAML::Node doc;
#ifdef USE_OLD_YAML
    std::ifstream pose_fin;
    pose_fin.open(pose_yaml.c_str(), std::ifstream::in);
    YAML::Parser parser(pose_fin);
    while (parser.GetNextDocument(doc)) {
      const YAML::Node& template_pose_yaml = doc["template_poses"];
      for (size_t i = 0; i < template_pose_yaml.size(); i++) {
        const YAML::Node& pose = template_pose_yaml[i];
        Eigen::Affine3f trans = jsk_recognition_utils::affineFromYAMLNode(pose);
        template_poses_.push_back(trans);
        // set template_bboxes
        pcl::PointCloud<pcl::PointXYZRGBA> transformed_cloud;
        pcl::transformPointCloud<pcl::PointXYZRGBA>(
          *template_cloud_, transformed_cloud, trans);
        // compute size of bounding box
        Eigen::Vector4f minpt, maxpt;
        pcl::getMinMax3D<pcl::PointXYZRGBA>(transformed_cloud, minpt, maxpt);
        jsk_recognition_msgs::BoundingBox bbox = jsk_recognition_utils::boundingBoxFromPointCloud(transformed_cloud);
        //ROS_INFO("bounding box size: [%f, %f, %f]", bbox.dimensions.x, bbox.dimensions.y, bbox.dimensions.z);
        template_bboxes_.push_back(bbox);
      }
    }
    pose_fin.close();
#else
     // yaml-cpp is greater than 0.5.0
     doc = YAML::LoadFile(pose_yaml);
#endif

    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &LINEMODDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_detect_mask_ = advertise<sensor_msgs::Image>(*pnh_, "output/mask", 1);
    pub_pose_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose", 1);
    pub_original_template_cloud_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output/template", 1);

    onInitPostProcess();
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
    gradient_magnitude_threshold_ = config.gradient_magnitude_threshold;
    detection_threshold_ = config.detection_threshold;
    color_gradient_mod_.setGradientMagnitudeThreshold(gradient_magnitude_threshold_);
    linemod_.setDetectionThreshold(detection_threshold_);

    // desearlize
    const std::string linemod_file = template_file_ + ".linemod";
    std::ifstream linemod_in;
    linemod_in.open(linemod_file.c_str(), std::ifstream::in);
    linemod_.deserialize(linemod_in);
    linemod_in.close();
  }

  void LINEMODDetector::computeCenterOfTemplate(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
    const pcl::SparseQuantizedMultiModTemplate& linemod_template,
    const pcl::LINEMODDetection& linemod_detection,
    Eigen::Vector3f& center)
  {
    const size_t start_x = std::max(linemod_detection.x, 0);
    const size_t start_y = std::max(linemod_detection.y, 0);
    const size_t end_x = std::min(
      static_cast<size_t> (start_x + linemod_template.region.width * linemod_detection.scale),
      static_cast<size_t> (cloud->width));
    const size_t end_y = std::min(
      static_cast<size_t> (start_y + linemod_template.region.height * linemod_detection.scale),
      static_cast<size_t> (cloud->height));
    size_t counter = 0;
    for (size_t row_index = start_y; row_index < end_y; ++row_index) {
      for (size_t col_index = start_x; col_index < end_x; ++col_index) {
        const pcl::PointXYZRGBA & point = (*cloud) (col_index, row_index);
        if (pcl_isfinite (point.x) &&
            pcl_isfinite (point.y) &&
            pcl_isfinite (point.z)) {
          center = center + point.getVector3fMap();
          ++counter;
        }
      }
    }
    center = center / counter;
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

    surface_normal_mod_.setInputCloud(cloud);
    surface_normal_mod_.processInputData ();
    color_gradient_mod_.setInputCloud (cloud);
    color_gradient_mod_.processInputData ();
    std::vector<pcl::LINEMODDetection> linemod_detections;
    std::vector<pcl::QuantizableModality*> modalities;
    modalities.push_back(&color_gradient_mod_);
    modalities.push_back(&surface_normal_mod_);
    linemod_.detectTemplatesSemiScaleInvariant(modalities, linemod_detections,
                                               0.6944444f, 1.44f, 1.2f);
    NODELET_INFO("detected %lu result", linemod_detections.size());
    // lookup the best result
    if (linemod_detections.size() > 0) {
      double max_score = 0;
      size_t max_template_id = 0;
      double max_scale = 0;
      pcl::LINEMODDetection linemod_detection;
      for (size_t i = 0; i < linemod_detections.size(); i++) {
        const pcl::LINEMODDetection& detection = linemod_detections[i];
        if (max_score < detection.score) {
          linemod_detection = detection;
          max_score = detection.score;
        }
      }
      
      const pcl::SparseQuantizedMultiModTemplate& linemod_template = 
        linemod_.getTemplate(linemod_detection.template_id);
      Eigen::Vector3f center(0, 0, 0);
      computeCenterOfTemplate(
        cloud, linemod_template, linemod_detection, center);
      // publish mask image here
      cv::Mat detect_mask = cv::Mat::zeros(cloud->width, cloud->height, CV_8UC1);
      int scaled_template_width
        = linemod_template.region.width * linemod_detection.scale;
      int scaled_template_height
        = linemod_template.region.height * linemod_detection.scale;
      cv::rectangle(
        detect_mask,
        cv::Point(linemod_detection.x, linemod_detection.y),
        cv::Point(linemod_detection.x + scaled_template_width,
                  linemod_detection.y + scaled_template_height),
        cv::Scalar(255), CV_FILLED);
      pub_detect_mask_.publish(cv_bridge::CvImage(cloud_msg->header,
                                                  "8UC1",
                                                  detect_mask).toImageMsg());
      // compute translation
      jsk_recognition_msgs::BoundingBox bbox = template_bboxes_[linemod_detection.template_id];
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr 
        result (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::transformPointCloud<pcl::PointXYZRGBA>(
        *template_cloud_, *result, template_poses_[linemod_detection.template_id]);
      Eigen::Vector4f minpt, maxpt;
      pcl::getMinMax3D<pcl::PointXYZRGBA>(*result, minpt, maxpt);
      Eigen::Vector4f template_center = (minpt + maxpt) / 2;
      Eigen::Vector3f translation = center - Eigen::Vector3f(template_center[0],
                                                             template_center[1],
                                                             template_center[2]);
      Eigen::Affine3f pose = template_poses_[linemod_detection.template_id] * Eigen::Translation3f(translation);
      geometry_msgs::PoseStamped ros_pose;
      tf::poseEigenToMsg(pose, ros_pose.pose);
      ros_pose.header = cloud_msg->header;
      pub_pose_.publish(ros_pose);
      for (size_t i = 0; i < result->points.size(); i++) {
        result->points[i].getVector3fMap()
          = result->points[i].getVector3fMap() + translation;
      }
      sensor_msgs::PointCloud2 ros_result;
      pcl::toROSMsg(*result, ros_result);
      ros_result.header = cloud_msg->header;
      pub_cloud_.publish(ros_result);
      sensor_msgs::PointCloud2 ros_template;
      pcl::toROSMsg(*template_cloud_, ros_template);
      ros_template.header = cloud_msg->header;
      pub_original_template_cloud_.publish(ros_template);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LINEMODTrainer, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LINEMODDetector, nodelet::Nodelet);
