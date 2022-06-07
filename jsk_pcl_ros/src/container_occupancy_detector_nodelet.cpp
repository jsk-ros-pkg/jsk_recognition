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
/*
 * container_occupancy_detector_nodelet.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "jsk_pcl_ros/container_occupancy_detector.h"

namespace jsk_pcl_ros{

    void ContainerOccupancyDetector::onInit(){
        jsk_topic_tools::ConnectionBasedNodelet::onInit();
        boxes_occupancy_pub_
            = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "boxes_occupancy", 1);
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
        onInitPostProcess();
    }

    void ContainerOccupancyDetector::subscribe(){
        sub_boxes_.subscribe(*pnh_, "container/boxes", 1);
        sub_points_.subscribe(*pnh_, "container/points", 1);
        sub_point_indices_.subscribe(*pnh_, "container/point_indices", 1);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(sub_boxes_, sub_points_, sub_point_indices_);
        sync_->registerCallback(boost::bind(&ContainerOccupancyDetector::calculate, this, _1, _2, _3));
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
        // vital_checker_->poke(); TODO fix
        if (!boxes_msg->boxes.empty()){
            // if boxes exist
            if(pointsTransform(boxes_msg, points_msg)){
                // if succeeded in transforming pointclouds
                jsk_recognition_msgs::BoundingBoxArray result_ = *boxes_msg;
                pcl_conversions::toPCL(*transformed_points_msg_, *pcl_pc2_ptr_); // convert to pcl
                pcl::fromPCLPointCloud2(*pcl_pc2_ptr_, *pcl_xyz_ptr_);
                for(size_t i_box = 0; i_box < boxes_msg->boxes.size(); i_box++){
                    // each boxes
                    float sum_occupancy_ = 0.0, occupancy_;
                    float box_top_h_ = boxes_msg->boxes.at(i_box).pose.position.z
                        + boxes_msg->boxes.at(i_box).dimensions.z / 2.0;
                    float box_buttom_h_ = boxes_msg->boxes.at(i_box).pose.position.z
                        - boxes_msg->boxes.at(i_box).dimensions.z / 2.0;

                    for(auto index = point_indices_msg->cluster_indices.at(i_box).indices.begin();
                        index != point_indices_msg->cluster_indices.at(i_box).indices.end();
                        ++index){
                        // each points in the box
                        sum_occupancy_ += (pcl_xyz_ptr_->at(*index).z - box_buttom_h_)
                            / (box_top_h_ - box_buttom_h_);
                    }
                    occupancy_ =
                        sum_occupancy_ / float(point_indices_msg->cluster_indices.at(i_box).indices.size());
                    result_.boxes.at(i_box).value = occupancy_;
                }
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
