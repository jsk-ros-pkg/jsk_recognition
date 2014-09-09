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

#include "jsk_pcl_ros/cluster_point_indices_decomposer.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <boost/format.hpp>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/pca.h>

#include <Eigen/Geometry> 

#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include "jsk_pcl_ros/pcl_util.h"
namespace jsk_pcl_ros
{
  ClusterPointIndicesDecomposer::ClusterPointIndicesDecomposer() {}
  ClusterPointIndicesDecomposer::~ClusterPointIndicesDecomposer() {}

  
  void ClusterPointIndicesDecomposer::onInit()
  {
    PCLNodelet::onInit();

    pnh_.reset (new ros::NodeHandle (getPrivateNodeHandle ()));
    pc_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("debug_output", 1);
    box_pub_ = pnh_->advertise<jsk_pcl_ros::BoundingBoxArray>("boxes", 1);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_target_.subscribe(*pnh_, "target", 1);

    pnh_->param("publish_tf", publish_tf_, true);
    if (!pnh_->getParam("tf_prefix", tf_prefix_))
    {
      if (publish_tf_) {
        ROS_WARN("~tf_prefix is not specified, using %s", getName().c_str());
      }
      tf_prefix_ = getName();
    }

    pnh_->param("publish_clouds", publish_clouds_, true);
    
    pnh_->param("align_boxes", align_boxes_, false);
    pnh_->param("use_pca", use_pca_, false);
    
    if (align_boxes_) {
      sync_align_ = boost::make_shared<message_filters::Synchronizer<SyncAlignPolicy> >(100);
      sub_polygons_.subscribe(*pnh_, "align_planes", 1);
      sub_coefficients_.subscribe(*pnh_, "align_planes_coefficients", 1);
      sync_align_->connectInput(sub_input_, sub_target_, sub_polygons_, sub_coefficients_);
      sync_align_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2, _3, _4));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_target_);
      sync_->registerCallback(boost::bind(&ClusterPointIndicesDecomposer::extract, this, _1, _2));
    }
  }
  
  void ClusterPointIndicesDecomposer::sortIndicesOrder
  (pcl::PointCloud<pcl::PointXYZ>::Ptr input,
   std::vector<pcl::IndicesPtr> indices_array,
   std::vector<pcl::IndicesPtr> &output_array)
  {
    output_array.resize(indices_array.size());
    for (size_t i = 0; i < indices_array.size(); i++)
    {
      output_array[i] = indices_array[i];
    }
  }
  
  int ClusterPointIndicesDecomposer::findNearestPlane(const Eigen::Vector4f& center,
                                                      const jsk_pcl_ros::PolygonArrayConstPtr& planes,
                                                      const jsk_pcl_ros::ModelCoefficientsArrayConstPtr& coefficients)
  {
    double min_distance = DBL_MAX;
    int nearest_index = -1;
    for (size_t i = 0; i < coefficients->coefficients.size(); i++) {
      geometry_msgs::PolygonStamped polygon_msg = planes->polygons[i];
      Vertices vertices;
      for (size_t j = 0; j < polygon_msg.polygon.points.size(); j++) {
        Vertex v;
        v[0] = polygon_msg.polygon.points[j].x;
        v[1] = polygon_msg.polygon.points[j].y;
        v[2] = polygon_msg.polygon.points[j].z;
        vertices.push_back(v);
      }
      ConvexPolygon p(vertices, coefficients->coefficients[i].values);
      double distance = p.distanceToPoint(center);
      if (distance < min_distance) {
        min_distance = distance;
        nearest_index = i;
      }
    }
    return nearest_index;
  }

  void ClusterPointIndicesDecomposer::computeBoundingBox
  (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
   const std_msgs::Header header,
   const Eigen::Vector4f center,
   const jsk_pcl_ros::PolygonArrayConstPtr& planes,
   const jsk_pcl_ros::ModelCoefficientsArrayConstPtr& coefficients,
   jsk_pcl_ros::BoundingBox& bounding_box)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      segmented_cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    // align boxes if possible
    Eigen::Matrix4f m4 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    if (align_boxes_) {
      int nearest_plane_index = findNearestPlane(center, planes, coefficients);
      if (nearest_plane_index == -1) {
        segmented_cloud_transformed = segmented_cloud;
        NODELET_ERROR("no planes to align boxes are given");
      }
      else {
        Eigen::Vector3f normal, z_axis;
        normal[0] = coefficients->coefficients[nearest_plane_index].values[0];
        normal[1] = coefficients->coefficients[nearest_plane_index].values[1];
        normal[2] = coefficients->coefficients[nearest_plane_index].values[2];
        normal = normal.normalized();
        z_axis[0] = 0; z_axis[1] = 0; z_axis[2] = 1;
        Eigen::Vector3f rotation_axis = z_axis.cross(normal).normalized();
        double theta = acos(z_axis.dot(normal));
        if (isnan(theta)) {
          segmented_cloud_transformed = segmented_cloud;
          NODELET_ERROR("cannot compute angle to align the point cloud: [%f, %f, %f], [%f, %f, %f]",
                        z_axis[0], z_axis[1], z_axis[2],
                        normal[0], normal[1], normal[2]);
        }
        else {
          Eigen::Matrix3f m = Eigen::Matrix3f::Identity();
          m = m * Eigen::AngleAxisf(theta, rotation_axis);

          if (use_pca_) {
            // first project points to the plane
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud
              (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::ProjectInliers<pcl::PointXYZRGB> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            pcl::ModelCoefficients::Ptr
              plane_coefficients (new pcl::ModelCoefficients);
            plane_coefficients->values
              = coefficients->coefficients[nearest_plane_index].values;
            proj.setModelCoefficients(plane_coefficients);
            proj.setInputCloud(segmented_cloud);
            proj.filter(*projected_cloud);

            pcl::PCA<pcl::PointXYZRGB> pca;
            pca.setInputCloud(projected_cloud);
            Eigen::Matrix3f eigen = pca.getEigenVectors();
            m.col(0) = eigen.col(0);
            m.col(1) = eigen.col(1);
            // flip axis to satisfy right-handed system
            if (m.col(0).cross(m.col(1)).dot(m.col(2)) < 0) {
              m.col(0) = - m.col(0);
            }
          }
            
          // m4 <- m
          for (size_t row = 0; row < 3; row++) {
            for (size_t column = 0; column < 3; column++) {
              m4(row, column) = m(row, column);
            }
          }
          q = m;
          Eigen::Matrix4f inv_m = m4.inverse();
          pcl::transformPointCloud(*segmented_cloud, *segmented_cloud_transformed, inv_m);
        }
      }
    }
    else {
      segmented_cloud_transformed = segmented_cloud;
    }
      
    // create a bounding box
    Eigen::Vector4f minpt, maxpt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*segmented_cloud_transformed, minpt, maxpt);

    double xwidth = maxpt[0] - minpt[0];
    double ywidth = maxpt[1] - minpt[1];
    double zwidth = maxpt[2] - minpt[2];

    Eigen::Vector4f center2((maxpt[0] + minpt[0]) / 2.0, (maxpt[1] + minpt[1]) / 2.0, (maxpt[2] + minpt[2]) / 2.0, 1.0);
    Eigen::Vector4f center_transformed = m4 * center2;
      
    bounding_box.header = header;
      
    bounding_box.pose.position.x = center_transformed[0];
    bounding_box.pose.position.y = center_transformed[1];
    bounding_box.pose.position.z = center_transformed[2];
    bounding_box.pose.orientation.x = q.x();
    bounding_box.pose.orientation.y = q.y();
    bounding_box.pose.orientation.z = q.z();
    bounding_box.pose.orientation.w = q.w();
    bounding_box.dimensions.x = xwidth;
    bounding_box.dimensions.y = ywidth;
    bounding_box.dimensions.z = zwidth;
  }

  void ClusterPointIndicesDecomposer::addToDebugPointCloud
  (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
   size_t i,
   pcl::PointCloud<pcl::PointXYZRGB>& debug_output)
  {
    uint32_t rgb = colorRGBAToUInt32(colorCategory20(i));
    for (size_t j = 0; j < segmented_cloud->points.size(); j++) {
      pcl::PointXYZRGB p;
      p.x= segmented_cloud->points[j].x;
      p.y= segmented_cloud->points[j].y;
      p.z= segmented_cloud->points[j].z;
      p.rgb = *reinterpret_cast<float*>(&rgb);
      debug_output.points.push_back(p);
    }
  }
  
  void ClusterPointIndicesDecomposer::extract
  (const sensor_msgs::PointCloud2ConstPtr &input,
   const jsk_pcl_ros::ClusterPointIndicesConstPtr &indices_input,
   const jsk_pcl_ros::PolygonArrayConstPtr& planes,
   const jsk_pcl_ros::ModelCoefficientsArrayConstPtr& coefficients)
  {
    if (publish_clouds_) {
      allocatePublishers(indices_input->cluster_indices.size());
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::fromROSMsg(*input, *cloud_xyz);

    std::vector<pcl::IndicesPtr> converted_indices;
    std::vector<pcl::IndicesPtr> sorted_indices;
    for (size_t i = 0; i < indices_input->cluster_indices.size(); i++)
    {
      pcl::IndicesPtr vindices;
      vindices.reset (new std::vector<int> (indices_input->cluster_indices[i].indices));
      converted_indices.push_back(vindices);
    }
    
    sortIndicesOrder(cloud_xyz, converted_indices, sorted_indices);
    extract.setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZRGB> debug_output;
    jsk_pcl_ros::BoundingBoxArray bounding_box_array;
    bounding_box_array.header = input->header;
    for (size_t i = 0; i < sorted_indices.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
      extract.setIndices(sorted_indices[i]);
      extract.filter(*segmented_cloud);
      if (publish_clouds_) {
        sensor_msgs::PointCloud2::Ptr out_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*segmented_cloud, *out_cloud);
        out_cloud->header = input->header;
        publishers_[i].publish(out_cloud);
      }
      // publish tf
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*segmented_cloud, center);
      if (publish_tf_) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
        transform.setRotation(tf::createIdentityQuaternion());
        br_.sendTransform(tf::StampedTransform(transform, input->header.stamp,
                                               input->header.frame_id,
                                               tf_prefix_ + (boost::format("output%02u") % (i)).str()));
      }
      // adding the pointcloud into debug_output
      addToDebugPointCloud(segmented_cloud, i, debug_output);
      
      jsk_pcl_ros::BoundingBox bounding_box;
      computeBoundingBox(segmented_cloud, input->header, center, planes, coefficients, bounding_box);
      bounding_box_array.boxes.push_back(bounding_box);
    }
    
    sensor_msgs::PointCloud2 debug_ros_output;
    pcl::toROSMsg(debug_output, debug_ros_output);
    debug_ros_output.header = input->header;
    debug_ros_output.is_dense = false;
    pc_pub_.publish(debug_ros_output);
    box_pub_.publish(bounding_box_array);
  }
  
  void ClusterPointIndicesDecomposer::extract
  (const sensor_msgs::PointCloud2ConstPtr &input,
   const jsk_pcl_ros::ClusterPointIndicesConstPtr &indices_input)
  {
    extract(input, indices_input,
            jsk_pcl_ros::PolygonArrayConstPtr(),
            jsk_pcl_ros::ModelCoefficientsArrayConstPtr());
  }

  void ClusterPointIndicesDecomposer::allocatePublishers(size_t num)
  {
    if (num > publishers_.size())
    {
        for (size_t i = publishers_.size(); i < num; i++)
        {
            std::string topic_name = (boost::format("output%02u") % (i)).str();
            ROS_INFO("advertising %s", topic_name.c_str());
            ros::Publisher publisher = pnh_->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
            publishers_.push_back(publisher);
        }
    }
  }
  
}

typedef jsk_pcl_ros::ClusterPointIndicesDecomposer ClusterPointIndicesDecomposer;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ClusterPointIndicesDecomposer, ClusterPointIndicesDecomposer, nodelet::Nodelet);

