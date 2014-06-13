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

#include "jsk_pcl_ros/multi_plane_extraction.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/filters/project_inliers.h>
#include <set>

#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
typedef pcl_msgs::ModelCoefficients PCLModelCoefficientMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
typedef pcl::ModelCoefficients PCLModelCoefficientMsg;
#endif



namespace jsk_pcl_ros
{

  void MultiPlaneExtraction::onInit()
  {
    PCLNodelet::onInit();
    //pub_ = pnh_->advertise<PCLIndicesMsg>("output", 1);
    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    nonplane_pub_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("output_nonplane_cloud", 1);
    if (!pnh_->getParam("max_queue_size", maximum_queue_size_)) {
      maximum_queue_size_ = 100;
    }
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&MultiPlaneExtraction::configCallback, this, _1, _2);
    srv_->setCallback (f);

    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(maximum_queue_size_);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_input_, sub_indices_, sub_coefficients_, sub_polygons_);
    sync_->registerCallback(boost::bind(&MultiPlaneExtraction::extract, this, _1, _2, _3, _4));
  }

  void MultiPlaneExtraction::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    min_height_ = config.min_height;
    max_height_ = config.max_height;
  }

  void MultiPlaneExtraction::extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                                     const jsk_pcl_ros::ClusterPointIndices::ConstPtr& indices,
                                     const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
                                     const jsk_pcl_ros::PolygonArray::ConstPtr& polygons)
  {
    boost::mutex::scoped_lock(mutex_);
    // convert all to the pcl types
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*input, *input_cloud);
    
    // concat indices into one PointIndices
    pcl::PointIndices::Ptr all_indices (new pcl::PointIndices);
    for (size_t i = 0; i < indices->cluster_indices.size(); i++) {
      std::vector<int> one_indices = indices->cluster_indices[i].indices;
      for (size_t j = 0; j < one_indices.size(); j++) {
        all_indices->indices.push_back(one_indices[j]);
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nonplane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_nonplane;
    extract_nonplane.setNegative(true);
    extract_nonplane.setInputCloud(input_cloud);
    extract_nonplane.setIndices(all_indices);
    extract_nonplane.filter(*nonplane_cloud);
    nonplane_pub_.publish(nonplane_cloud);
    // for each plane, project nonplane_cloud to the plane and find the points
    // inside of the polygon
    
    std::set<int> result_set;
    for (size_t plane_i = 0; plane_i < coefficients->coefficients.size(); plane_i++) {

      pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism_extract;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      geometry_msgs::Polygon the_polygon = polygons->polygons[plane_i].polygon;
      for (size_t i = 0; i < the_polygon.points.size(); i++) {
        pcl::PointXYZRGB p;
        p.x = the_polygon.points[i].x;
        p.y = the_polygon.points[i].y;
        p.z = the_polygon.points[i].z;
        hull_cloud->points.push_back(p);
      }
      
      prism_extract.setInputCloud(nonplane_cloud);
      prism_extract.setHeightLimits(min_height_, max_height_);
      prism_extract.setInputPlanarHull(hull_cloud);
      //pcl::PointCloud<pcl::PointXYZRGB> output;
      pcl::PointIndices output_indices;
      prism_extract.segment(output_indices);
      // append output to result_cloud
      for (size_t i = 0; i < output_indices.indices.size(); i++) {
        result_set.insert(output_indices.indices[i]);
        //result_cloud.points.push_back(output.points[i]);
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
  }
  
}

typedef jsk_pcl_ros::MultiPlaneExtraction MultiPlaneExtraction;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, MultiPlaneExtraction, MultiPlaneExtraction, nodelet::Nodelet);
