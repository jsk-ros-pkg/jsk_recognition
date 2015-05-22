// -*- mode: C++ -*-
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

#include "jsk_pcl_ros/multi_plane_extraction.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/filters/project_inliers.h>
#include <set>

namespace jsk_pcl_ros
{

  void MultiPlaneExtraction::onInit()
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    DiagnosticNodelet::onInit();
    pnh_->param("use_indices", use_indices_, true);
    pnh_->param("use_async", use_async_, false);
    ////////////////////////////////////////////////////////
    // Publishers
    ////////////////////////////////////////////////////////
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    nonplane_pub_ = advertise<pcl::PointCloud<pcl::PointXYZRGB> >(*pnh_, "output_nonplane_cloud", 1);
    pub_indices_ = advertise<PCLIndicesMsg>(*pnh_, "output/indices", 1);
    if (!pnh_->getParam("max_queue_size", maximum_queue_size_)) {
      maximum_queue_size_ = 100;
    }
    pnh_->param("use_sensor_frame", use_sensor_frame_, false);
    if (use_sensor_frame_) {
      pnh_->param("sensor_frame", sensor_frame_, std::string("head_root"));
      tf_listener_ = TfListenerSingleton::getInstance();
    }
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&MultiPlaneExtraction::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void MultiPlaneExtraction::subscribe()
  {
    ////////////////////////////////////////////////////////
    // Subscribe
    ////////////////////////////////////////////////////////

    
    sub_input_.subscribe(*pnh_, "input", 1);
    
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    if (use_async_) {
      if (use_indices_) {
        sub_indices_.subscribe(*pnh_, "indices", 1);
        async_ = boost::make_shared<message_filters::Synchronizer<ASyncPolicy> >(maximum_queue_size_);
        async_->connectInput(sub_input_, sub_indices_, sub_coefficients_, sub_polygons_);
        async_->registerCallback(boost::bind(&MultiPlaneExtraction::extract, this, _1, _2, _3, _4));
      }
      else {
        async_wo_indices_ = boost::make_shared<message_filters::Synchronizer<ASyncWithoutIndicesPolicy> >(maximum_queue_size_);
        async_wo_indices_->connectInput(sub_input_, sub_coefficients_, sub_polygons_);
        async_wo_indices_->registerCallback(boost::bind(&MultiPlaneExtraction::extract, this, _1, _2, _3));
      }    }
    else {
      if (use_indices_) {
        sub_indices_.subscribe(*pnh_, "indices", 1);
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(maximum_queue_size_);
        sync_->connectInput(sub_input_, sub_indices_, sub_coefficients_, sub_polygons_);
        sync_->registerCallback(boost::bind(&MultiPlaneExtraction::extract, this, _1, _2, _3, _4));
      }
      else {
        sync_wo_indices_ = boost::make_shared<message_filters::Synchronizer<SyncWithoutIndicesPolicy> >(maximum_queue_size_);
        sync_wo_indices_->connectInput(sub_input_, sub_coefficients_, sub_polygons_);
        sync_wo_indices_->registerCallback(boost::bind(&MultiPlaneExtraction::extract, this, _1, _2, _3));
      }
    }
  }

  void MultiPlaneExtraction::unsubscribe()
  {
    sub_input_.unsubscribe();
    if (use_indices_) {
      sub_indices_.unsubscribe();
    }
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }

  void MultiPlaneExtraction::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_height_ = config.min_height;
    max_height_ = config.max_height;
    maginify_ = config.maginify;
  }

  void MultiPlaneExtraction::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "MultiPlaneExtraction running");
      stat.add("Minimum Height", min_height_);
      stat.add("Maximum Height", max_height_);
      stat.add("Number of Planes", plane_counter_.mean());
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "MultiPlaneExtraction", vital_checker_, stat);
    }
  }

  void MultiPlaneExtraction::extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                                     const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients,
                                     const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons)
  {
    extract(input, jsk_recognition_msgs::ClusterPointIndices::ConstPtr(),
            coefficients, polygons);
  }
  
  void MultiPlaneExtraction::extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                                     const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices,
                                     const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients,
                                     const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    Eigen::Vector3f viewpoint;
    try {
      if (use_sensor_frame_) {
        tf::StampedTransform transform
          = lookupTransformWithDuration(tf_listener_,
                                        input->header.frame_id,
                                        sensor_frame_,
                                        input->header.stamp,
                                        ros::Duration(5.0));
        Eigen::Affine3f sensor_pose;
        tf::transformTFToEigen(transform, sensor_pose);
        viewpoint = Eigen::Vector3f(sensor_pose.translation());
      }
    }
    catch (tf2::ConnectivityException &e)
    {
      JSK_NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      JSK_NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (...)
    {
      JSK_NODELET_ERROR("Unknown transform error");
    }
    // convert all to the pcl types
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*input, *input_cloud);
    if (indices) {
      // concat indices into one PointIndices
      pcl::PointIndices::Ptr all_indices (new pcl::PointIndices);
      for (size_t i = 0; i < indices->cluster_indices.size(); i++) {
        std::vector<int> one_indices = indices->cluster_indices[i].indices;
        for (size_t j = 0; j < one_indices.size(); j++) {
          all_indices->indices.push_back(one_indices[j]);
        }
      }

    
      pcl::ExtractIndices<pcl::PointXYZRGB> extract_nonplane;
      extract_nonplane.setNegative(true);
      extract_nonplane.setInputCloud(input_cloud);
      extract_nonplane.setIndices(all_indices);
      extract_nonplane.filter(*nonplane_cloud);
      nonplane_pub_.publish(nonplane_cloud);
    }
    else {
      nonplane_cloud = input_cloud;
    }
    // for each plane, project nonplane_cloud to the plane and find the points
    // inside of the polygon
    
    std::set<int> result_set;
    plane_counter_.add(coefficients->coefficients.size());
    for (size_t plane_i = 0; plane_i < coefficients->coefficients.size(); plane_i++) {

      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism_extract;
      prism_extract.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      geometry_msgs::Polygon the_polygon = polygons->polygons[plane_i].polygon;
      if (the_polygon.points.size() <= 2) {
        JSK_NODELET_WARN("too small polygon");
        continue;
      }
      // compute centroid first
      Eigen::Vector3f centroid(0, 0, 0);
      for (size_t i = 0; i < the_polygon.points.size(); i++) {
        pcl::PointXYZRGB p;
        pointFromXYZToXYZ<geometry_msgs::Point32, pcl::PointXYZRGB>(
          the_polygon.points[i], p);
        centroid = centroid + p.getVector3fMap();
      }
      centroid = centroid / the_polygon.points.size();
      
      for (size_t i = 0; i < the_polygon.points.size(); i++) {
        pcl::PointXYZRGB p;
        pointFromXYZToXYZ<geometry_msgs::Point32, pcl::PointXYZRGB>(
          the_polygon.points[i], p);
        Eigen::Vector3f dir = (p.getVector3fMap() - centroid).normalized();
        p.getVector3fMap() = dir * maginify_ + p.getVector3fMap();
        hull_cloud->points.push_back(p);
      }
      
      pcl::PointXYZRGB p_last;
        pointFromXYZToXYZ<geometry_msgs::Point32, pcl::PointXYZRGB>(
          the_polygon.points[0], p_last);
      hull_cloud->points.push_back(p_last);
      
      prism_extract.setInputCloud(nonplane_cloud);
      prism_extract.setHeightLimits(min_height_, max_height_);
      prism_extract.setInputPlanarHull(hull_cloud);
      pcl::PointIndices output_indices;
      prism_extract.segment(output_indices);
      // append output to result_cloud
      for (size_t i = 0; i < output_indices.indices.size(); i++) {
        result_set.insert(output_indices.indices[i]);
      }
    }

    // convert std::set to PCLIndicesMsg
    //PCLIndicesMsg output_indices;
    pcl::PointCloud<pcl::PointXYZRGB> result_cloud;
    pcl::PointIndices::Ptr all_result_indices (new pcl::PointIndices());
    for (std::set<int>::iterator it = result_set.begin();
         it != result_set.end();
         it++) {
      all_result_indices->indices.push_back(*it);
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_all_indices;
    extract_all_indices.setInputCloud(nonplane_cloud);
    extract_all_indices.setIndices(all_result_indices);
    extract_all_indices.filter(result_cloud);
    
    sensor_msgs::PointCloud2 ros_result;
    pcl::toROSMsg(result_cloud, ros_result);
    ros_result.header = input->header;
    pub_.publish(ros_result);
    PCLIndicesMsg ros_indices;
    pcl_conversions::fromPCL(*all_result_indices, ros_indices);
    ros_indices.header = input->header;
    pub_indices_.publish(ros_indices);
    diagnostic_updater_->update();
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::MultiPlaneExtraction, nodelet::Nodelet);
