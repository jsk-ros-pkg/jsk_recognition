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
    if (!pnh_->getParam("max_queue_size", maximum_queue_size_)) {
      maximum_queue_size_ = 100;
    }
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(maximum_queue_size_);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_input_, sub_indices_, sub_coefficients_, sub_polygons_);
    sync_->registerCallback(boost::bind(&MultiPlaneExtraction::extract, this, _1, _2, _3, _4));
  }

  void MultiPlaneExtraction::extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                                     const jsk_pcl_ros::ClusterPointIndices::ConstPtr& indices,
                                     const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
                                     const jsk_pcl_ros::PolygonArray::ConstPtr& polygons)
  {
    // convert all to the pcl types
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::fromROSMsg(*input, *input_cloud);
    
    // concat indices into one PointIndices
    pcl::PointIndices::Ptr all_indices (new pcl::PointIndices);
    for (size_t i = 0; i < indices->cluster_indices.size(); i++) {
      std::vector<int> one_indices = indices->cluster_indices[i].indices;
      for (size_t j = 0; j < one_indices.size(); j++) {
        all_indices->indices.push_back(one_indices[j]);
      }
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr nonplane_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract_nonplane;
    extract_nonplane.setNegative(true);
    extract_nonplane.setInputCloud(input_cloud);
    extract_nonplane.setIndices(all_indices);
    extract_nonplane.filter(*nonplane_cloud);

    // for each plane, project nonplane_cloud to the plane and find the points
    // inside of the polygon
    std::set<size_t> result_set;
    for (size_t plane_i = 0; plane_i < coefficients->coefficients.size(); plane_i++) {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
      geometry_msgs::Polygon the_polygon = polygons->polygons[plane_i].polygon;
      pcl::ModelCoefficients::Ptr pcl_coefficients (new pcl::ModelCoefficients());
      pcl_coefficients->values = coefficients->coefficients[plane_i].values;
      
      pcl::ProjectInliers<pcl::PointXYZRGBNormal> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setInputCloud(nonplane_cloud);
      //proj.setInputCloud(input_cloud);
      //proj.setIndices(all_indices);
      //proj.setNegative(true);
      proj.setModelCoefficients(pcl_coefficients);
      proj.filter(*projected_cloud);
      Eigen::Vector3f n;
      n[0] = pcl_coefficients->values[0];
      n[1] = pcl_coefficients->values[1];
      n[2] = pcl_coefficients->values[2];
      // check the points is inside of the convex hull or not for all the points
      for (size_t point_i = 0; point_i < projected_cloud->points.size(); point_i++) {
        bool insidep = true;
        Eigen::Vector3f P;
        P[0] = projected_cloud->points[point_i].x;
        P[1] = projected_cloud->points[point_i].y;
        P[2] = projected_cloud->points[point_i].z;
        for (size_t convex_vertex_i = 0; convex_vertex_i < the_polygon.points.size() - 1; convex_vertex_i++) {
          Eigen::Vector3f O, B;
          int b_i = convex_vertex_i + 1;
          O[0] = the_polygon.points[convex_vertex_i].x;
          O[1] = the_polygon.points[convex_vertex_i].y;
          O[2] = the_polygon.points[convex_vertex_i].z;
          B[0] = the_polygon.points[b_i].x;
          B[1] = the_polygon.points[b_i].y;
          B[2] = the_polygon.points[b_i].z;
          if ((B - O).cross(P - O).dot(n) < 0) {
            insidep = false;
            break;
          }
          // compute the anglew
        }
        if (insidep) {
          //result_indices.push_back(point_i);
          result_set.insert(point_i);
        }
      }
    }

    // convert std::set to PCLIndicesMsg
    //PCLIndicesMsg output_indices;
    pcl::PointCloud<pcl::PointXYZRGBNormal> result;
    sensor_msgs::PointCloud2 ros_result;
    for (std::set<size_t>::iterator it = result_set.begin();
         it != result_set.end();
         it++) {
      result.points.push_back(nonplane_cloud->points[*it]);
      //output_indices.indices.push_back(*it);
    }
    pcl::toROSMsg(result, ros_result);
    ros_result.header = input->header;
    pub_.publish(ros_result);
    
  }
  
}

typedef jsk_pcl_ros::MultiPlaneExtraction MultiPlaneExtraction;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, MultiPlaneExtraction, MultiPlaneExtraction, nodelet::Nodelet);
