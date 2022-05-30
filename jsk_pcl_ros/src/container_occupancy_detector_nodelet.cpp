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
 * container_occupancy_nodelet.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "jsk_pcl_ros/container_occupancy_detector.h"

const float FLOAT_MIN = std::numeric_limits<float>::min();
const float FLOAT_MAX = std::numeric_limits<float>::max();

namespace jsk_pcl_ros{

    void ContainerOccupancyDetector::onInit(){
        jsk_topic_tools::ConnectionBasedNodelet::onInit();

        occupancy_rate_pub_
            = advertise<std_msgs::Float64MultiArray>(*pnh_, "occupancy_rate", 1);
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
        onInitPostProcess();
    }

    void ContainerOccupancyDetector::subscribe(){
        sub_box_.subscribe(*pnh_, "containers", 1);
        sub_points_.subscribe(*pnh_, "points", 1);
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(sub_box_, sub_points_);
        sync_->registerCallback(boost::bind(&ContainerOccupancyDetector::calculate, this, _1, _2));
    }

    void ContainerOccupancyDetector::unsubscribe(){
        sub_box_.unsubscribe();
        sub_points_.unsubscribe();
        delete tf_listener_;
    }

    void ContainerOccupancyDetector::calculate(
        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg,
        const sensor_msgs::PointCloud2::ConstPtr& points_msg
    ){
        boost::mutex::scoped_lock lock(mutex_);
        pcl::PCLPointCloud2Ptr pcl_pc2_ptr_; // shared ptr
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz_ptr_; // shared ptr

        if(!box_array_msg->boxes.empty()){
            // if containers were subscribed
            if(pointsTransform(points_msg)){
                // when point cloud transform succeeded
                std_msgs::Float64MultiArray occupancy_rates_;

                pcl_conversions::toPCL(*transformed_points_, *pcl_pc2_ptr_);
                pcl::fromPCLPointCloud2(*pcl_pc2_ptr_, *pcl_xyz_ptr_);

                for(auto box = box_array_msg->boxes.begin();
                    box != box_array_msg->boxes.end();
                    ++box){
                    // calculate occupancy for each bounding boxes
                    pcl::CropBox<pcl::PointXYZ> box_filter_;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr box_filtered_points_;
                    pcl::PointXYZ min_pt_, max_pt_;
                    float sum_occupancy_ = 0.0, occupancy_;
                    float h_rate_;
                    const float box_top_h_ = box->pose.position.z + box->dimensions.z / 2.0;
                    const float box_buttom_h_ = box->pose.position.z - box->dimensions.z / 2.0;

                    box_filter_.setMin(Eigen::Vector4f(
                                           (box->pose.position.x -
                                            box->dimensions.x / 2.0),
                                           (box->pose.position.y -
                                            box->dimensions.y / 2.0),
                                           FLOAT_MIN, 1.0f));
                    box_filter_.setMax(Eigen::Vector4f(
                                           (box->pose.position.x +
                                            box->dimensions.x / 2.0),
                                           (box->pose.position.y +
                                            box->dimensions.y / 2.0),
                                           FLOAT_MAX, 1.0f));
                    for(auto point = box_filtered_points_->points.begin();
                        point != box_filtered_points_->points.end();
                        ++point){
                        h_rate_ = (point->z - box_buttom_h_) / (point->z - box_buttom_h_);
                        if(h_rate_ > 0){
                            // not count h_rate_ < 0 points
                            sum_occupancy_ += h_rate_;
                        }
                    }
                    occupancy_ = sum_occupancy_ / float(box_filtered_points_->size());
                    occupancy_rates_.data.push_back(occupancy_);
                }
                occupancy_rate_pub_.publish(occupancy_rates_);
            }else{
                // when point cloud transfrom not succeeded
                NODELET_WARN("Failed to transform pointcloud\n");
                return;
            }
        }else{
            // if no containers
            NODELET_DEBUG_ONCE("No containers subscribed\n");
            return;
        }
    }

    bool ContainerOccupancyDetector::pointsTransform(
        const sensor_msgs::PointCloud2::Ptr& points_msg
    ){
        geometry_msgs::TransformStamped transform_stamped_;
        try{
            transform_stamped_ = tf_buffer_.lookupTransform(
                "base_link",
                points_msg->header.frame_id,
                points_msg->header.stamp,
                ros::Duration(10.0));
            tf2::doTransform(points_msg, transformed_points_, transform_stamped_);
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
