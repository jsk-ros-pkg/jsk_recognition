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
#include "jsk_pcl_ros/hinted_handle_estimator.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <math.h>

using namespace std;

inline float NORM(float vx, float vy, float vz){return sqrt(vx*vx+vy*vy+vz*vz);}

namespace jsk_pcl_ros
{
  void HintedHandleEstimator::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_pose_ = advertise<geometry_msgs::PoseStamped>(
                                                      *pnh_, "handle_pose", 1);
    pub_length_ = advertise<std_msgs::Float64>(
                                               *pnh_, "handle_length", 1);
    pub_handle_ = advertise<jsk_recognition_msgs::SimpleHandle>(
                                               *pnh_, "handle", 1);
    pub_debug_marker_ = advertise<visualization_msgs::Marker>(*pnh_, "debug_marker", 1);
    pub_debug_marker_array_ = advertise<visualization_msgs::MarkerArray>(*pnh_, "debug_marker_array", 1);
    handle = handle_model();
    pnh_->param("finger_l", handle.finger_l, 0.03);
    pnh_->param("finger_d", handle.finger_d, 0.02);
    pnh_->param("finger_w", handle.finger_w, 0.01);
    pnh_->param("arm_l", handle.arm_l, 0.05);
    pnh_->param("arm_d", handle.arm_d, 0.02);
    pnh_->param("arm_w", handle.arm_w, 0.1);
    onInitPostProcess();
  }

  void HintedHandleEstimator::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "cloud", 1);
    sub_point_.subscribe(*pnh_, "point", 1); 
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_cloud_, sub_point_);
    sync_->registerCallback(boost::bind(&HintedHandleEstimator::estimate, this, _1, _2));
  }
  
  void HintedHandleEstimator::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_point_.unsubscribe();
  }
  
  void HintedHandleEstimator::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const geometry_msgs::PointStampedConstPtr &point_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_msg, *cloud);
    geometry_msgs::PointStamped transed_point;
    ros::Time now = ros::Time::now();
    try
      {
      listener_.waitForTransform(cloud->header.frame_id, point_msg->header.frame_id, now, ros::Duration(1.0));
      listener_.transformPoint(cloud->header.frame_id, now, *point_msg, point_msg->header.frame_id, transed_point);
    }
    catch(tf::TransformException ex)
      {
      ROS_ERROR("%s", ex.what());
      return;
    }
    pcl::PointXYZ searchPoint;
    searchPoint.x = transed_point.point.x;
    searchPoint.y = transed_point.point.y;
    searchPoint.z = transed_point.point.z;

    //remove too far cloud
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(searchPoint.x - 3*handle.arm_w, searchPoint.x + 3*handle.arm_w);
    pass.filter(*cloud);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(searchPoint.y - 3*handle.arm_w, searchPoint.y + 3*handle.arm_w);
    pass.filter(*cloud);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(searchPoint.z - 3*handle.arm_w, searchPoint.z + 3*handle.arm_w);
    pass.filter(*cloud);

    if(cloud->points.size() < 10){
      ROS_INFO("points are too small");
      return;
    }
    if(1){ //estimate_normal
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud(cloud);
      ne.setSearchMethod(kd_tree);
      ne.setRadiusSearch(0.02);
      ne.setViewPoint(0, 0, 0);
      ne.compute(*cloud_normals);
    }
    else{ //use normal of msg
      
    }
    if(! (kd_tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)){
      ROS_INFO("kdtree failed");
      return;
    }
    float x = cloud->points[pointIdxNKNSearch[0]].x;
    float y = cloud->points[pointIdxNKNSearch[0]].y;
    float z = cloud->points[pointIdxNKNSearch[0]].z;
    float v_x = cloud_normals->points[pointIdxNKNSearch[0]].normal_x;
    float v_y = cloud_normals->points[pointIdxNKNSearch[0]].normal_y;
    float v_z = cloud_normals->points[pointIdxNKNSearch[0]].normal_z;
    double theta = acos(v_x);
    // use normal for estimating handle direction
    tf::Quaternion normal(0, v_z/NORM(0, v_y, v_z) * cos(theta/2), -v_y/NORM(0, v_y, v_z) * cos(theta/2), sin(theta/2));
    tf::Quaternion final_quaternion = normal;
    double min_theta_index = 0;
    double min_width = 100;
    tf::Quaternion min_qua(0, 0, 0, 1);
    visualization_msgs::Marker debug_hand_marker;
    debug_hand_marker.header = cloud_msg->header;
    debug_hand_marker.ns = string("debug_grasp");
    debug_hand_marker.id = 0;
    debug_hand_marker.type = visualization_msgs::Marker::LINE_LIST;
    debug_hand_marker.pose.orientation.w = 1;
    debug_hand_marker.scale.x=0.003;
    tf::Matrix3x3 best_mat;
    //search 180 degree and calc the shortest direction
    for(double theta_=0; theta_<3.14/2; 
        theta_+=3.14/2/30){
      tf::Quaternion rotate_(sin(theta_), 0, 0, cos(theta_));
      tf::Quaternion temp_qua = normal * rotate_;
      tf::Matrix3x3 temp_mat(temp_qua);
      geometry_msgs::Pose pose_respected_to_tf;
      pose_respected_to_tf.position.x = x;
      pose_respected_to_tf.position.y = y;
      pose_respected_to_tf.position.z = z;
      pose_respected_to_tf.orientation.x = temp_qua.getX();
      pose_respected_to_tf.orientation.y = temp_qua.getY();
      pose_respected_to_tf.orientation.z = temp_qua.getZ();
      pose_respected_to_tf.orientation.w = temp_qua.getW();
      Eigen::Affine3d box_pose_respected_to_cloud_eigend;
      tf::poseMsgToEigen(pose_respected_to_tf, box_pose_respected_to_cloud_eigend);
      Eigen::Affine3d box_pose_respected_to_cloud_eigend_inversed
        = box_pose_respected_to_cloud_eigend.inverse();
      Eigen::Matrix4f box_pose_respected_to_cloud_eigen_inversed_matrixf;
      Eigen::Matrix4d box_pose_respected_to_cloud_eigen_inversed_matrixd
        = box_pose_respected_to_cloud_eigend_inversed.matrix();
      jsk_recognition_utils::convertMatrix4<Eigen::Matrix4d, Eigen::Matrix4f>(
                                                                    box_pose_respected_to_cloud_eigen_inversed_matrixd,
                                                                    box_pose_respected_to_cloud_eigen_inversed_matrixf);
      Eigen::Affine3f offset = Eigen::Affine3f(box_pose_respected_to_cloud_eigen_inversed_matrixf);
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*cloud, *output_cloud, offset);

      pcl::PassThrough<pcl::PointXYZ> pass;
      pcl::PointCloud<pcl::PointXYZ>::Ptr points_z(new pcl::PointCloud<pcl::PointXYZ>), points_yz(new pcl::PointCloud<pcl::PointXYZ>), points_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      pass.setInputCloud(output_cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-handle.arm_w*2, handle.arm_w*2);
      pass.filter(*points_z);
      pass.setInputCloud(points_z);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-handle.finger_d, handle.finger_d);
      pass.filter(*points_yz);
      pass.setInputCloud(points_yz);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-(handle.arm_l-handle.finger_l), handle.finger_l);
      pass.filter(*points_xyz);
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      for(size_t index=0; index<points_xyz->size(); index++){
        points_xyz->points[index].x = points_xyz->points[index].z = 0;
      }
      if(points_xyz->points.size() == 0){ROS_INFO("points are empty");return;}
      kdtree.setInputCloud(points_xyz);
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      pcl::PointXYZ search_point_tree;
      search_point_tree.x=search_point_tree.y=search_point_tree.z=0;
      if( kdtree.radiusSearch(search_point_tree, 10, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        double before_w=10, temp_w;
        for(size_t index = 0; index < pointIdxRadiusSearch.size(); ++index){
          temp_w =sqrt(pointRadiusSquaredDistance[index]);
          if(temp_w - before_w > handle.finger_w*2){
            break; // there are small space for finger
          }
          before_w=temp_w;
        }
        if(before_w < min_width){
          min_theta_index = theta_;
          min_width = before_w;
          min_qua = temp_qua;
          best_mat = temp_mat;
        }
        //for debug view
        geometry_msgs::Point temp_point;
        std_msgs::ColorRGBA temp_color;
        temp_color.r=0; temp_color.g=0; temp_color.b=1; temp_color.a=1;
        temp_point.x=x-temp_mat.getColumn(1)[0] * before_w;
        temp_point.y=y-temp_mat.getColumn(1)[1] * before_w;
        temp_point.z=z-temp_mat.getColumn(1)[2] * before_w;
        debug_hand_marker.points.push_back(temp_point);
        debug_hand_marker.colors.push_back(temp_color);
        temp_point.x+=2*temp_mat.getColumn(1)[0] * before_w;
        temp_point.y+=2*temp_mat.getColumn(1)[1] * before_w;
        temp_point.z+=2*temp_mat.getColumn(1)[2] * before_w;
        debug_hand_marker.points.push_back(temp_point);
        debug_hand_marker.colors.push_back(temp_color);
      }
    }
    geometry_msgs::PoseStamped handle_pose_stamped;
    handle_pose_stamped.header = cloud_msg->header;
    handle_pose_stamped.pose.position.x = x;
    handle_pose_stamped.pose.position.y = y;
    handle_pose_stamped.pose.position.z = z;
    handle_pose_stamped.pose.orientation.x = min_qua.getX();
    handle_pose_stamped.pose.orientation.y = min_qua.getY();
    handle_pose_stamped.pose.orientation.z = min_qua.getZ();
    handle_pose_stamped.pose.orientation.w = min_qua.getW();
    std_msgs::Float64 min_width_msg;
    min_width_msg.data = min_width;
    pub_pose_.publish(handle_pose_stamped);
    pub_debug_marker_.publish(debug_hand_marker);
    pub_debug_marker_array_.publish(make_handle_array(handle_pose_stamped, handle));
    jsk_recognition_msgs::SimpleHandle simple_handle;
    simple_handle.header = handle_pose_stamped.header;
    simple_handle.pose = handle_pose_stamped.pose;
    simple_handle.handle_width = min_width;
    pub_handle_.publish(simple_handle);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HintedHandleEstimator, nodelet::Nodelet);


