// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, JSK Lab
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
 * skeleton_with_depth.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_perception/skeleton_with_depth.h>

namespace jsk_perception{

    void SkeletonWithDepth::onInit(){
        DiagnosticNodelet::onInit();
        pub_skeleton_ = advertise<jsk_recognition_msgs::HumanSkeletonArray>(*pnh_, "output/skeleton", 1);
        pnh_->param("approximate_sync", approximate_sync_, true);
        onInitPostProcess();
    }

    SkeletonWithDepth::SkeletonWithDepth::~SkeletonWithDepth(){
        if(approximate_sync_){
            ap_sync_.reset();
        }else{
            sync_.reset();
        }
    }

    void SkeletonWithDepth::subscribe(){
        sub_skeleton_.subscribe(*pnh_, "input/skeleton", 1);
        sub_depth_.subscribe(*pnh_, "input/depth", 1);
        sub_cam_info_ = pnh_->subscribe("input/info", 1, &SkeletonWithDepth::camInfoCallback, this);
        if(approximate_sync_){
            ap_sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
            ap_sync_->connectInput(sub_skeleton_, sub_depth_);
            ap_sync_->registerCallback(boost::bind(&SkeletonWithDepth::callback,
                                                   this,
                                                   boost::placeholders::_1,
                                                   boost::placeholders::_2));
        }else{
            sync_ = std::make_shared<message_filters::Synchronizer<ExactSyncPolicy> >(100);
            sync_->connectInput(sub_skeleton_, sub_depth_);
            sync_->registerCallback(boost::bind(&SkeletonWithDepth::callback,
                                                this,
                                                boost::placeholders::_1,
                                                boost::placeholders::_2));
        }
    }

    void SkeletonWithDepth::unsubscribe(){
        sub_skeleton_.unsubscribe();
        sub_depth_.unsubscribe();
        sub_cam_info_.shutdown();
    }

    void SkeletonWithDepth::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
        cam_info_ = *msg;
    }

    void SkeletonWithDepth::callback(const jsk_recognition_msgs::HumanSkeletonArray::ConstPtr& skeleton_array_msg,
                                     const sensor_msgs::Image::ConstPtr& depth_msg){
        std::lock_guard<std::mutex> lock(mutex_);
        // check camera info
        if(cam_info_ == sensor_msgs::CameraInfo()){
            NODELET_ERROR("Camera info is not set");
            return;
        }
        image_geometry::PinholeCameraModel cam_model_;
        cam_model_.fromCameraInfo(cam_info_);

        // convert depth image
        cv_bridge::CvImagePtr cv_depth_ptr;
        try{
            cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }catch(cv_bridge::Exception& e){
            NODELET_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if(cv_depth_ptr->image.type() == CV_16UC1){
            cv::Mat tmp;
            cv_depth_ptr->image.convertTo(tmp, CV_32FC1, 0.001);
            cv_depth_ptr->image = tmp;
        }else if(cv_depth_ptr->image.type() != CV_32FC1){
            NODELET_ERROR("Unsupported depth image type: %d", cv_depth_ptr->image.type());
            return;
        }
        const cv::Mat depth = cv_depth_ptr->image;

        // convert skeleton
        jsk_recognition_msgs::HumanSkeletonArray skeleton_array_output;
        skeleton_array_output.header = skeleton_array_msg->header;
        const int H = depth.rows;
        const int W = depth.cols;
        for(auto skeleton : skeleton_array_msg->skeletons){
            jsk_recognition_msgs::HumanSkeleton skeleton_output;
            skeleton_output.header = skeleton_array_msg->header;
            for(auto bone_info : boost::combine(skeleton.bone_names, skeleton.bones)){
                std::string bone_name;
                jsk_recognition_msgs::Segment bone;
                boost::tie(bone_name, bone) = bone_info;

                // bone start point
                if ((0 <= bone.start_point.x < W) && (0 <= bone.start_point.y < H)){
                    const float z_start = depth.at<float>((int)bone.start_point.y, (int)bone.start_point.x);
                    bone.start_point.z = z_start;
                    bone.start_point.x = (bone.start_point.x - cam_model_.cx()) * z_start / cam_model_.fx();
                    bone.start_point.y = (bone.start_point.y - cam_model_.cy()) * z_start / cam_model_.fy();
                    if (bone.start_point.z <= 0){
                        continue;
                    }
                }else{
                    continue;
                }

                // bone end point
                if ((0 <= bone.end_point.x < W) && (0 <= bone.end_point.y < H)){
                    const float z_end = depth.at<float>((int)bone.end_point.y, (int)bone.end_point.x);
                    bone.end_point.z = z_end;
                    bone.end_point.x = (bone.end_point.x - cam_model_.cx()) * z_end / cam_model_.fx();
                    bone.end_point.y = (bone.end_point.y - cam_model_.cy()) * z_end / cam_model_.fy();
                    if (bone.start_point.z <= 0){
                        continue;
                    }
                }else{
                    continue;
                }
                skeleton_output.bone_names.push_back(bone_name);
                skeleton_output.bones.push_back(bone);
            }
            skeleton_array_output.skeletons.push_back(skeleton_output);
        }
        vital_checker_->poke();
        pub_skeleton_.publish(skeleton_array_output);
    }

    void SkeletonWithDepth::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){
        if(vital_checker_ -> isAlive()){
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "SkeletonWithDepth running\n");
        }
        DiagnosticNodelet::updateDiagnostic(stat);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::SkeletonWithDepth, nodelet::Nodelet);
