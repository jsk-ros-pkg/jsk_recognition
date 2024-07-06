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
 * skeleton_with_depth.h
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef SKELETON_WITH_DEPTH_H_
#define SKELETON_WITH_DEPTH_H_

#include <boost/range/combine.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <jsk_recognition_msgs/HumanSkeleton.h>
#include <jsk_recognition_msgs/HumanSkeletonArray.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <mutex>
#include <sensor_msgs/Image.h>


namespace jsk_perception{

    class SkeletonWithDepth : public jsk_topic_tools::DiagnosticNodelet{
    public:
    SkeletonWithDepth() : DiagnosticNodelet("SkeletonWithDepth"){}
        virtual ~SkeletonWithDepth();

    protected:
        virtual void onInit();
        virtual void subscribe();
        virtual void unsubscribe();
        virtual void callback(const jsk_recognition_msgs::HumanSkeletonArray::ConstPtr& skeleton_msg,
                              const sensor_msgs::Image::ConstPtr& depth_msg);
        virtual void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg);
        virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

        message_filters::Subscriber<jsk_recognition_msgs::HumanSkeletonArray> sub_skeleton_;
        message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
        ros::Subscriber sub_cam_info_; // not synchronized
        ros::Publisher pub_skeleton_;

        sensor_msgs::CameraInfo cam_info_ = sensor_msgs::CameraInfo();

        typedef message_filters::sync_policies::ExactTime<jsk_recognition_msgs::HumanSkeletonArray, sensor_msgs::Image> ExactSyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<jsk_recognition_msgs::HumanSkeletonArray, sensor_msgs::Image> ApproximateSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> > sync_;
        std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > ap_sync_;

        std::mutex mutex_;

        bool approximate_sync_;

    private:

    };
}

#endif // SKELETON_WITH_DEPTH_H_
