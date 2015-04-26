// -*- c-basic-offset: 4; indent-tabs-mode: nil; -*-
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
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <posedetection_msgs/ImageFeature0D.h>
#include <posedetection_msgs/Feature0DDetect.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>

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
#include <jsk_perception/image_utils.h>

using namespace std;
using namespace ros;

class SiftNode
{
    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::Image,
        sensor_msgs::Image > SyncPolicy;

    boost::mutex _mutex;
    ros::NodeHandle _node;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _subImage;
    // for useMask
    boost::shared_ptr<image_transport::SubscriberFilter> _subImageWithMask;
        boost::shared_ptr<image_transport::SubscriberFilter> _subMask;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > _sync;
    ros::ServiceServer _srvDetect;
    Subscriber _subInfo;
    Publisher _pubSift;
    posedetection_msgs::ImageFeature0D _sift_msg;
    bool _bInfoInitialized;
    bool _useMask;
public:
    ros::Time lasttime;

    SiftNode() : _it(_node)
    {
        ros::NodeHandle pnh("~");
        pnh.param("use_mask", _useMask, false);
        
        _pubSift = _node.advertise<posedetection_msgs::ImageFeature0D>("ImageFeature0D",1);
        _srvDetect = _node.advertiseService("Feature0DDetect",&SiftNode::detectCb,this);
        if (!_useMask) {
            _subImage = _it.subscribe("image",1,&SiftNode::imageCb,this);
        }
        else {
            _subImageWithMask.reset(new image_transport::SubscriberFilter(_it, "image", 1));
            _subMask.reset(new image_transport::SubscriberFilter(_it, "mask", 1));
            _sync.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100), *_subImageWithMask, *_subMask));
            _sync->registerCallback(boost::bind(&SiftNode::imageCb, this, _1, _2));
        }
        _subInfo = _node.subscribe("camera_info",1,&SiftNode::infoCb,this);
        lasttime = ros::Time::now();
        _bInfoInitialized = false;
    }

    void infoCb(const sensor_msgs::CameraInfoConstPtr& msg_ptr)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _sift_msg.info = *msg_ptr;
        _bInfoInitialized = true;
    }

    bool detectCb(posedetection_msgs::Feature0DDetect::Request& req, posedetection_msgs::Feature0DDetect::Response& res)
    {
        return detect(res.features,req.image, sensor_msgs::Image::ConstPtr());
    }

    bool detect(posedetection_msgs::Feature0D& features, const sensor_msgs::Image& imagemsg,
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
                ROS_INFO("clear sift resources");
                DestroyAllImages();
                imagesift = NULL;
            }
            
            image = framefloat->image;

            if (mask_ptr) {
                cv::Mat mask = cv_bridge::toCvShare(mask_ptr, mask_ptr->encoding)->image;
                region = jsk_perception::boundingRectOfMaskImage(mask);
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
        ros::Time siftbasetime = ros::Time::now();
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

        ROS_INFO("imagesift: image: %d(size=%lu), num: %d, sift time: %.3fs, total: %.3fs", imagemsg.header.seq,
                 imagemsg.data.size(),  numkeys,
                 (float)(ros::Time::now()-siftbasetime).toSec(), (float)(ros::Time::now()-lasttime).toSec());
        lasttime = ros::Time::now();
        return true;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr,
                 const sensor_msgs::ImageConstPtr& mask_ptr)
    {
        if(_pubSift.getNumSubscribers()==0){ 
            ROS_DEBUG("number of subscribers is 0, ignoring image");
            return;
        }
        if(!_bInfoInitialized) {
            ROS_DEBUG("camera info not initialized, ignoring image");
            return;
        }

        detect(_sift_msg.features,*msg_ptr, mask_ptr);
        _sift_msg.image = *msg_ptr; // probably copying pointers so don't use after this call

        {
            boost::mutex::scoped_lock lock(_mutex); // needed for camerainfo
            _pubSift.publish(_sift_msg);
        }
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        imageCb(msg_ptr, sensor_msgs::ImageConstPtr());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imagesift");
    if(!ros::master::check())
        return 1;
    
    boost::shared_ptr<SiftNode> siftnode(new SiftNode());
    
    ros::spin();
    siftnode.reset();
    return 0;
}
