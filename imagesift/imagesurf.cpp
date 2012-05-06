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

#include <opencv/highgui.h>
#include <opencv/cv.h>
#if (CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4)
#include <opencv2/nonfree/nonfree.hpp>
#endif

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <map>
#include <string>
#include <cstdio>
#include <vector>

#include <cv_bridge/CvBridge.h>

using namespace std;
using namespace ros;

class SurfNode
{
    boost::mutex _mutex;
    ros::NodeHandle _node;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _subImage;
    ros::ServiceServer _srvDetect;
    Subscriber _subInfo;
    Publisher _pubSurf;
    sensor_msgs::CvBridge _bridge, _bridgefloat;
    posedetection_msgs::ImageFeature0D surf_msg;
    bool _bInfoInitialized;

    cv::SURF calc_surf;

public:
    ros::Time lasttime;

    SurfNode() : _it(_node)
    { 
        _pubSurf = _node.advertise<posedetection_msgs::ImageFeature0D>("ImageFeature0D",1);
	usleep(100000);
        _subImage = _it.subscribe("image",1,&SurfNode::image_cb,this);
	usleep(100000);
        _subInfo = _node.subscribe("camera_info",1,&SurfNode::info_cb,this);
	usleep(100000);
        _srvDetect = _node.advertiseService("Feature0DDetect",&SurfNode::detect_cb,this);
        calc_surf = cv::SURF(500,4,2,true);
        lasttime = ros::Time::now();
        _bInfoInitialized = false;
    }
    virtual ~SurfNode() {
        _srvDetect.shutdown();
        _subInfo.shutdown();
        _subImage.shutdown();
        _pubSurf.shutdown();
    }

    void info_cb(const sensor_msgs::CameraInfoConstPtr& msg_ptr)
    {
        boost::mutex::scoped_lock lock(_mutex);
        surf_msg.info = *msg_ptr;
        _bInfoInitialized = true;
    }

    bool detect_cb(posedetection_msgs::Feature0DDetect::Request& req, posedetection_msgs::Feature0DDetect::Response& res)
    {
        return Detect(res.features,req.image);
    }

    bool Detect(posedetection_msgs::Feature0D& features, const sensor_msgs::Image& imagemsg)
    {
        boost::mutex::scoped_lock lock(_mutex);
        IplImage *frame;
        cv::Mat grayImage;
        try {
          if (!_bridge.fromImage(imagemsg, "mono8"))
            return false;
          frame = _bridge.toIpl();
          grayImage = cv::Mat(frame).clone();
        }
        catch (sensor_msgs::CvBridgeException error) {
            ROS_WARN("bad frame");
            return false;
        }

        // compute SURF
        ros::Time surfbasetime = ros::Time::now();
        vector<cv::KeyPoint> kp_vec;
        vector<float> desc_vec;
        calc_surf(grayImage,cv::Mat(),kp_vec,desc_vec);

        // write the keys to the output
        int numkeys = kp_vec.size();

        // publish
        features.header = imagemsg.header;
        features.positions.resize(numkeys*2);
        features.scales.resize(numkeys);
        features.orientations.resize(numkeys);
        features.confidences.resize(numkeys);
        features.descriptors.resize(numkeys*128);
        features.descriptor_dim = 128;
        features.type = "opencv_surf";

        int index = 0;
        vector<cv::KeyPoint>::iterator key = kp_vec.begin(), key_end = kp_vec.end();
        for(; key != key_end; ++key) {

            for(int j = 0; j < 128; ++j)
                features.descriptors[128*index+j] = desc_vec[128*index+j];

            features.positions[2*index+0] = key->pt.x;
            features.positions[2*index+1] = key->pt.y;
            features.scales[index] = key->size;
            features.orientations[index] = key->angle;
            features.confidences[index] = key->response;

            ++index;
        }

        ROS_INFO("imagesurf: image: %d(size=%d), num: %d, surf time: %.3fs, total: %.3fs", (int)imagemsg.header.seq,
                 (int)imagemsg.data.size(),  (int)numkeys,
                 (float)(ros::Time::now()-surfbasetime).toSec(), (float)(ros::Time::now()-lasttime).toSec());

        lasttime = ros::Time::now();
        return true;
    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        if( _pubSurf.getNumSubscribers()==0 ){ 
            ROS_DEBUG("number of subscribers is 0, ignoring image");
            return;
        }
        if( !_bInfoInitialized ) {
            ROS_DEBUG("camera info not initialized, ignoring image");
            return;
        }

        Detect(surf_msg.features,*msg_ptr);
        surf_msg.image = *msg_ptr; // probably copying pointers so don't use af

        {
            boost::mutex::scoped_lock lock(_mutex); // needed for camerainfo
            _pubSurf.publish(surf_msg);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"imagesurf");
    if( !ros::master::check() )
        return 1;
    
    boost::shared_ptr<SurfNode> surfnode(new SurfNode());
    
    ros::spin();
    surfnode.reset();
    return 0;
}
