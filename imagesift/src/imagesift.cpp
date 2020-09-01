// -*- c-basic-offset: 4; indent-tabs-mode: nil; -*-
// vim: tabstop=4 shiftwidth=4:
// Copyright (C) 2008 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "imagesift/imagesift.h"
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <posedetection_msgs/ImageFeature0D.h>
#include <posedetection_msgs/Feature0DDetect.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/highgui.hpp>
#else
#include <opencv/highgui.h>
#include <opencv/cv.h>
#endif

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <string>
#include <cstdio>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <siftfast/siftfast.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <jsk_recognition_utils/cv_utils.h>

using namespace std;
using namespace ros;

namespace imagesift
{
    void SiftNode::onInit()
    {
        DiagnosticNodelet::onInit();
        _it.reset(new image_transport::ImageTransport(*nh_));

        pnh_->param("use_mask", _useMask, false);
        
        _pubFeatures = advertise<posedetection_msgs::Feature0D>(*nh_, "Feature0D", 1);
        _pubSift = advertise<posedetection_msgs::ImageFeature0D>(*nh_, "ImageFeature0D", 1);
        _srvDetect = nh_->advertiseService("Feature0DDetect", &SiftNode::detectCb, this);
        lasttime = ros::WallTime::now();
        _bInfoInitialized = false;

        onInitPostProcess();
    }

    void SiftNode::subscribe()
    {
        if (!_useMask) {
            _subImage = _it->subscribe("image", 1, &SiftNode::imageCb, this);
        }
        else {
            _subImageWithMask.subscribe(*nh_, "image", 1);
            _subMask.subscribe(*nh_, "mask", 1);
            _sync = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
            _sync->connectInput(_subImageWithMask, _subMask);
            _sync->registerCallback(boost::bind(&SiftNode::imageCb, this, _1, _2));
        }
        _subInfo = nh_->subscribe("camera_info", 1, &SiftNode::infoCb, this);
    }

    void SiftNode::unsubscribe()
    {
        if (!_useMask) {
            _subImage.shutdown();
        }
        else {
            _subImageWithMask.unsubscribe();
            _subMask.unsubscribe();
        }
        _subInfo.shutdown();
    }

    void SiftNode::infoCb(const sensor_msgs::CameraInfoConstPtr& msg_ptr)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _sift_msg.info = *msg_ptr;
        _bInfoInitialized = true;
    }

    bool SiftNode::detectCb(posedetection_msgs::Feature0DDetect::Request& req,
                            posedetection_msgs::Feature0DDetect::Response& res)
    {
        return detect(res.features,req.image, sensor_msgs::Image::ConstPtr());
    }

    bool SiftNode::detect(posedetection_msgs::Feature0D& features, const sensor_msgs::Image& imagemsg,
                          const sensor_msgs::Image::ConstPtr& mask_ptr)
    {
        boost::mutex::scoped_lock lock(_mutex);
        Image imagesift = NULL;
        cv::Rect region;
        try {
            cv::Mat image;
            cv_bridge::CvImagePtr framefloat;

            if (!(framefloat = cv_bridge::toCvCopy(imagemsg, "mono8")) )
                return false;
            
            if(imagesift != NULL && (imagesift->cols!=imagemsg.width || imagesift->rows!=imagemsg.height)) {
                ROS_DEBUG("clear sift resources");
                DestroyAllImages();
                imagesift = NULL;
            }
            
            image = framefloat->image;

            if (mask_ptr) {
                cv::Mat mask = cv_bridge::toCvShare(mask_ptr, mask_ptr->encoding)->image;
                region = jsk_recognition_utils::boundingRectOfMaskImage(mask);
                ROS_DEBUG ("region x:%d y:%d width:%d height:%d", region.x, region.y, region.width, region.height);
                if (region.width == 0 || region.height ==0) {
                    region = cv::Rect(0, 0, imagemsg.width, imagemsg.height);
                }
                image = image(region);
            }
            else {
                region = cv::Rect(0, 0, imagemsg.width, imagemsg.height);
            }
            
            if(imagesift == NULL)
                imagesift = CreateImage(imagemsg.height,imagemsg.width);

            for(int i = 0; i < imagemsg.height; ++i) {
                uint8_t* psrc = (uint8_t*)image.data+image.step*i;
                float* pdst = imagesift->pixels+i*imagesift->stride;
                for(int j = 0; j < imagemsg.width; ++j)
                    pdst[j] = (float)psrc[j]*(1.0f/255.0f);
                //memcpy(imagesift->pixels+i*imagesift->stride,framefloat->imageData+framefloat->widthStep*i,imagemsg.width*sizeof(float));
            }
        }
        catch (cv_bridge::Exception error) {
            ROS_WARN("bad frame");
            return false;
        }

        // compute SIFT
        ros::WallTime siftbasetime = ros::WallTime::now();
        Keypoint keypts = GetKeypoints(imagesift);
        // write the keys to the output
        int numkeys = 0;
        Keypoint key = keypts;
        while(key) {
            numkeys++;
            key = key->next;
        }

        // publish
        features.header = imagemsg.header;
        features.positions.resize(numkeys*2);
        features.scales.resize(numkeys);
        features.orientations.resize(numkeys);
        features.confidences.resize(numkeys);
        features.descriptors.resize(numkeys*128);
        features.descriptor_dim = 128;
        features.type = "libsiftfast";

        int index = 0;
        key = keypts;
        while(key) {

            for(int j = 0; j < 128; ++j)
                features.descriptors[128*index+j] = key->descrip[j];

            features.positions[2*index+0] = key->col + region.x;
            features.positions[2*index+1] = key->row + region.y;
            features.scales[index] = key->scale;
            features.orientations[index] = key->ori;
            features.confidences[index] = 1.0; // SIFT has no confidence?

            key = key->next;
            ++index;
        }

        FreeKeypoints(keypts);
        DestroyAllImages();

        ROS_DEBUG("imagesift: image: %d(size=%lu), num: %d, sift time: %.3fs, total: %.3fs", imagemsg.header.seq,
                 imagemsg.data.size(),  numkeys,
                 (float)(ros::WallTime::now()-siftbasetime).toSec(), (float)(ros::WallTime::now()-lasttime).toSec());
        lasttime = ros::WallTime::now();
        return true;
    }

    void SiftNode::imageCb(const sensor_msgs::ImageConstPtr& msg_ptr,
                           const sensor_msgs::ImageConstPtr& mask_ptr)
    {
        vital_checker_->poke();
        if(_pubFeatures.getNumSubscribers()==0 && _pubSift.getNumSubscribers()==0) {
            ROS_DEBUG("number of subscribers is 0, ignoring image");
            return;
        }
        detect(_sift_msg.features,*msg_ptr, mask_ptr);
        _pubFeatures.publish(_sift_msg.features);

        if(!_bInfoInitialized) {
            ROS_DEBUG("camera info not initialized, ignoring image");
            return;
        }
        _sift_msg.image = *msg_ptr; // probably copying pointers so don't use after this call
        {
            boost::mutex::scoped_lock lock(_mutex); // needed for camerainfo
            _pubSift.publish(_sift_msg);
        }
    }
    
    void SiftNode::imageCb(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        imageCb(msg_ptr, sensor_msgs::ImageConstPtr());
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (imagesift::SiftNode, nodelet::Nodelet);
