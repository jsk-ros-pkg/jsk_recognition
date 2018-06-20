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

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_pcl_ros_utils/plane_reasoner.h"
#include <tf_conversions/tf_eigen.h>

namespace jsk_pcl_ros_utils
{
  void PlaneReasoner::onInit()
  {
    ////////////////////////////////////////////////////////
    // Diagnostics
    ////////////////////////////////////////////////////////
    DiagnosticNodelet::onInit();
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &PlaneReasoner::configCallback, this, _1, _2);
    srv_->setCallback (f);

    ////////////////////////////////////////////////////////
    // Publishers
    ////////////////////////////////////////////////////////
    pub_vertical_inliers_
      = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output/vertical/inliers", 1);
    pub_vertical_coefficients_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_, "output/vertical/coefficients", 1);
    pub_vertical_polygons_
      = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output/vertical/polygons", 1);
    pub_horizontal_inliers_
      = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output/horizontal/inliers", 1);
    pub_horizontal_coefficients_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_, "output/horizontal/coefficients", 1);
    pub_horizontal_polygons_
      = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output/horizontal/polygons", 1);

    onInitPostProcess();
  }

  void PlaneReasoner::subscribe()
  {
    ////////////////////////////////////////////////////////
    // Subscribers
    ////////////////////////////////////////////////////////
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_inliers_.subscribe(*pnh_, "input_inliers", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_inliers_,
                        sub_coefficients_, sub_polygons_);
    sync_->registerCallback(boost::bind(&PlaneReasoner::reason,
                                        this, _1, _2, _3, _4));
  }

  void PlaneReasoner::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_inliers_.unsubscribe();
    sub_coefficients_.unsubscribe();
    sub_polygons_.unsubscribe();
  }
                                
  
  void PlaneReasoner::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    global_frame_id_ = config.global_frame_id;
    horizontal_angular_threshold_ = config.horizontal_angular_threshold;
    vertical_angular_threshold_ = config.vertical_angular_threshold;
  }

  void PlaneReasoner::reason(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& inliers_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // Check the size of the array messages first
    if ((inliers_msg->cluster_indices.size()
         != coefficients_msg->coefficients.size()) ||
        (inliers_msg->cluster_indices.size()
         != polygons_msg->polygons.size())) {
      NODELET_FATAL("the size of inliers, coefficients and polygons are not same");
      return;
    }
    vital_checker_->poke();
    pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *input_cloud);
    
    // convert ROS msg to PCL/jsk_pcl messages
    std::vector<pcl::PointIndices::Ptr> inliers
      = pcl_conversions::convertToPCLPointIndices(inliers_msg->cluster_indices);
    std::vector<pcl::ModelCoefficients::Ptr> coefficients
      = pcl_conversions::convertToPCLModelCoefficients(
        coefficients_msg->coefficients);
    std::vector<jsk_recognition_utils::Plane::Ptr> planes = jsk_recognition_utils::convertToPlanes(coefficients);
    std::vector<geometry_msgs::PolygonStamped> polygons = polygons_msg->polygons;
    std::vector<PlaneInfoContainer> plane_infos
      = packInfo(inliers, coefficients, planes, polygons);
    std::vector<PlaneInfoContainer> horizontal_planes
      = filterHorizontalPlanes(plane_infos);
    std::vector<PlaneInfoContainer> vertical_planes
      = filterVerticalPlanes(plane_infos);
    publishPlaneInfo(vertical_planes,
                     cloud_msg->header,
                     input_cloud,
                     pub_vertical_inliers_,
                     pub_vertical_coefficients_,
                     pub_vertical_polygons_);
    publishPlaneInfo(horizontal_planes,
                     cloud_msg->header,
                     input_cloud,
                     pub_horizontal_inliers_,
                     pub_horizontal_coefficients_,
                     pub_horizontal_polygons_);
  }

  void PlaneReasoner::publishPlaneInfo(
    std::vector<PlaneInfoContainer>& containers,
    const std_msgs::Header& header,
    pcl::PointCloud<PointT>::Ptr cloud,
    ros::Publisher& pub_inlier,
    ros::Publisher& pub_coefficients,
    ros::Publisher& pub_polygons)
  {
    std::vector<pcl::PointIndices::Ptr> inliers;
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    std::vector<geometry_msgs::PolygonStamped> polygons;
    for (size_t i = 0; i < containers.size(); i++) {
      inliers.push_back(containers[i].get<0>());
      coefficients.push_back(containers[i].get<1>());
      polygons.push_back(containers[i].get<3>());
    }
    jsk_recognition_msgs::ClusterPointIndices ros_indices;
    jsk_recognition_msgs::ModelCoefficientsArray ros_coefficients;
    jsk_recognition_msgs::PolygonArray ros_polygons;
    ros_indices.header = header;
    ros_coefficients.header = header;
    ros_polygons.header = header;
    ros_indices.cluster_indices = pcl_conversions::convertToROSPointIndices(
      inliers, header);
    ros_coefficients.coefficients
      = pcl_conversions::convertToROSModelCoefficients(
        coefficients, header);
    ros_polygons.polygons = polygons;
    pub_inlier.publish(ros_indices);
    pub_coefficients.publish(ros_coefficients);
    pub_polygons.publish(ros_polygons);
  }
  
  std::vector<PlaneInfoContainer>
  PlaneReasoner::filterPlanesAroundAngle(
    double reference_angle,
    double thrshold,
    std::vector<PlaneInfoContainer>& infos)
  {
    
    std::vector<PlaneInfoContainer> ret;
    for (size_t i = 0; i < infos.size(); i++) {
      PlaneInfoContainer plane_info = infos[i];
      if (tf_listener_->canTransform(global_frame_id_,
                                     plane_info.get<3>().header.frame_id,
                                     plane_info.get<3>().header.stamp)) {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform(plane_info.get<3>().header.frame_id, global_frame_id_,
                                      
                                      plane_info.get<3>().header.stamp,
                                      transform);
        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);
        Eigen::Affine3f eigen_transform_3f;
        Eigen::Vector3d up_d =  (eigen_transform.rotation() * Eigen::Vector3d(0, 0, 1));
        Eigen::Vector3f up;
        jsk_recognition_utils::pointFromVectorToVector<Eigen::Vector3d, Eigen::Vector3f>(up_d, up);
        jsk_recognition_utils::Plane::Ptr plane = plane_info.get<2>();
        double angle = plane->angle(up);
        // ROS_INFO("axis: [%f, %f, %f]", up[0], up[1], up[2]);
        // ROS_INFO("plane: [%f, %f, %f, %f]", plane_info.get<1>()->values[0], plane_info.get<1>()->values[1], plane_info.get<1>()->values[2], plane_info.get<1>()->values[3]);
        // ROS_INFO("angle: %f", angle);
        if (fabs(angle - reference_angle) < thrshold) {
          ret.push_back(plane_info);
        }
      }
    }
    return ret;
  }
  
  std::vector<PlaneInfoContainer>
  PlaneReasoner::filterHorizontalPlanes(
    std::vector<PlaneInfoContainer>& infos)
  {
    return filterPlanesAroundAngle(
      0,
      horizontal_angular_threshold_,
      infos);
  }
  
  std::vector<PlaneInfoContainer>
  PlaneReasoner::filterVerticalPlanes(
    std::vector<PlaneInfoContainer>& infos)
  {
    return filterPlanesAroundAngle(
      M_PI / 2.0,
      vertical_angular_threshold_,
      infos);
  }

  std::vector<PlaneInfoContainer>
  PlaneReasoner::packInfo(
    std::vector<pcl::PointIndices::Ptr>& inliers,
    std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
    std::vector<jsk_recognition_utils::Plane::Ptr>& planes,
    std::vector<geometry_msgs::PolygonStamped>& polygons)
  {
    std::vector<PlaneInfoContainer> ret;
    for (size_t i = 0; i < inliers.size(); i++) {
      ret.push_back(boost::make_tuple<pcl::PointIndices::Ptr,
                    pcl::ModelCoefficients::Ptr,
                    jsk_recognition_utils::Plane::Ptr>(inliers[i], coefficients[i],
                                planes[i], polygons[i]));
    }
    return ret;
  }

  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PlaneReasoner, nodelet::Nodelet);
