// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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
/*
 * container_occupancy_detector_nodelet.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "jsk_pcl_ros/container_occupancy_detector.h"

namespace jsk_pcl_ros{

    void ContainerOccupancyDetector::onInit(){
        jsk_topic_tools::DiagnosticNodelet::onInit();
        pnh_->param("approximate_sync", approximate_sync_, false);
        pnh_->param("queue_size", queue_size_, 100);
        boxes_occupancy_pub_
            = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "container/occupancies", 1);
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
        onInitPostProcess();
    }

    ContainerOccupancyDetector::~ContainerOccupancyDetector(){
        // message_filters::Synchronizer needs to be called reset
        // before message_filters::Subscriber is freed.
        // Calling reset fixes the following error on shutdown of the nodelet:
        // terminate called after throwing an instance of
        // 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
        //     what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
        // Also see https://github.com/ros/ros_comm/issues/720 .
        if(approximate_sync_){
            ap_sync_.reset();
        }else{
            sync_.reset();
        }
    }

    void ContainerOccupancyDetector::subscribe(){
        sub_boxes_.subscribe(*pnh_, "container/boxes", 1);
        sub_points_.subscribe(*pnh_, "container/points", 1);
        sub_point_indices_.subscribe(*pnh_, "container/point_indices", 1);
        if(approximate_sync_){
            ap_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
            ap_sync_ -> connectInput(sub_boxes_, sub_points_, sub_point_indices_);
            ap_sync_ -> registerCallback(boost::bind(&ContainerOccupancyDetector::calculate, this,
                  boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
        }else{
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
            sync_->connectInput(sub_boxes_, sub_points_, sub_point_indices_);
            sync_->registerCallback(boost::bind(&ContainerOccupancyDetector::calculate, this,
                  boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
        }
    }

    void ContainerOccupancyDetector::unsubscribe(){
        sub_boxes_.unsubscribe();
        sub_points_.unsubscribe();
        sub_point_indices_.unsubscribe();
    }

    void ContainerOccupancyDetector::calculate(
        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& boxes_msg,
        const sensor_msgs::PointCloud2::ConstPtr& points_msg,
        const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& point_indices_msg
    ){
        boost::mutex::scoped_lock lock(mutex_);
        if (!boxes_msg->boxes.empty()){
            // if boxes exist
            if(pointsTransform(boxes_msg, points_msg)){
                // if succeeded in transforming pointclouds
                jsk_recognition_msgs::BoundingBoxArray result_ = *boxes_msg;
                pcl_conversions::toPCL(*transformed_points_msg_, *pcl_pc2_ptr_); // convert to pcl
                pcl::fromPCLPointCloud2(*pcl_pc2_ptr_, *pcl_xyz_ptr_);
                for(size_t i_box = 0; i_box < boxes_msg->boxes.size(); i_box++){
                    // each boxes
                    float sum_occupancy_ = 0.0;
                    int32_t n_points_ = 0;
                    Eigen::Vector3d t_(boxes_msg->boxes.at(i_box).pose.position.x,
                                       boxes_msg->boxes.at(i_box).pose.position.y,
                                       boxes_msg->boxes.at(i_box).pose.position.z);
                    Eigen::Quaterniond q_(boxes_msg->boxes.at(i_box).pose.orientation.w,
                                          boxes_msg->boxes.at(i_box).pose.orientation.x,
                                          boxes_msg->boxes.at(i_box).pose.orientation.y,
                                          boxes_msg->boxes.at(i_box).pose.orientation.z);
                    q_.normalize();
                    Eigen::Vector3d top_(0.0, 0.0, boxes_msg->boxes.at(i_box).dimensions.z / 2.0);
                    Eigen::Vector3d buttom_(0.0, 0.0, -boxes_msg->boxes.at(i_box).dimensions.z / 2.0);

                    for(auto index = point_indices_msg->cluster_indices.at(i_box).indices.begin();
                        index != point_indices_msg->cluster_indices.at(i_box).indices.end();
                        ++index){
                        // each points in the box
                        Eigen::Vector3d eigen_point_(pcl_xyz_ptr_->at(*index).x,
                                                     pcl_xyz_ptr_->at(*index).y,
                                                     pcl_xyz_ptr_->at(*index).z);
                        Eigen::Vector3d rotated_point_ = q_.conjugate() * (eigen_point_ - t_);
                        float rate_ = (rotated_point_.z() - buttom_.z()) / (top_.z() - buttom_.z()); // remove points under the box
                        if(rate_ > 0){
                            sum_occupancy_ += rate_;
                            n_points_++;
                        }
                    }
                    result_.boxes.at(i_box).value = sum_occupancy_ / float(n_points_);
                }
                vital_checker_->poke();
                boxes_occupancy_pub_.publish(result_);
            }else{
                NODELET_WARN("Failed to transform point cloud\n");
            }
        }else{
            NODELET_DEBUG("No containers subscribed\n");
        }
    }

    bool ContainerOccupancyDetector::pointsTransform(
        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& boxes_msg,
        const sensor_msgs::PointCloud2::ConstPtr& points_msg
    ){
        // convert pc2's frame to bounding box's one
        geometry_msgs::TransformStamped transform_stamped_;
        Eigen::Matrix4f mat;
        try{
            transform_stamped_ = tf_buffer_.lookupTransform(
                boxes_msg->header.frame_id,
                points_msg->header.frame_id,
                points_msg->header.stamp,
                ros::Duration(10.0));
            mat = tf2::transformToEigen(transform_stamped_.transform).matrix().cast<float>();
            pcl_ros::transformPointCloud(mat, *points_msg, *transformed_points_msg_);
            return true;
        }catch(tf2::TransformException &ex){
            NODELET_WARN("Failed to transform tf: %s\n", ex.what());
            return false;
        }
    }

    void ContainerOccupancyDetector::updateDiagnostic(
        diagnostic_updater::DiagnosticStatusWrapper &stat){
        if(vital_checker_ -> isAlive()){
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                         "ContainerOccupancyDetector running\n");
        }
        DiagnosticNodelet::updateDiagnostic(stat);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ContainerOccupancyDetector, nodelet::Nodelet);
