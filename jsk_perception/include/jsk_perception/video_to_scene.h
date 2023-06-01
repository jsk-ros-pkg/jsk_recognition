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
 * video_to_scene.h
 * Author: Kei Okada <k-okada@jsk.t.u-tokyo.ac.jp>
 */

#ifndef VIDEO_TO_SCENE_H_
#define VIDEO_TO_SCENE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/VideoToSceneConfig.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/bgsegm.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>


namespace jsk_perception{
    class VideoToScene: public jsk_topic_tools::DiagnosticNodelet{
    public:
        typedef VideoToSceneConfig Config;
    VideoToScene() : DiagnosticNodelet("VideoToScene") {}
    protected:
        virtual void onInit();
        virtual void subscribe();
        virtual void unsubscribe();
        virtual void work(const sensor_msgs::Image::ConstPtr& image_msg);
        virtual void configCallback(Config &config, uint32_t level);

        boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
        image_transport::Subscriber sub_;
        boost::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Publisher pub_;
        boost::mutex mutex_;
    private:
        cv::Ptr<cv::bgsegm::BackgroundSubtractorGMG> bgsubtractor;
        int min_percent_;
        int max_percent_;
        bool captured_;

    };
}

#endif // VIDEO_TO_SCENE_H_
