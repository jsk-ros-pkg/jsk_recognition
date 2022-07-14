// -*- mode: c++; indent-tabs-mode: nil; c-basic-offset: 4; -*-
// Software License Agreement (BSD License)
// Copyright (c) 2008, JSK Lab, Inc.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// author: Rosen Diankov
#include <algorithm>

#include <cstdio>
#include <vector>
#include <sstream>
#include <algorithm>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d_c.h>
#endif
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "image_geometry/pinhole_camera_model.h"
#include "posedetection_msgs/ObjectDetection.h"
#include "posedetection_msgs/Detect.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "geometry_msgs/PolygonStamped.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "eigen_conversions/eigen_msg.h"
#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>
#include <dynamic_reconfigure/server.h>
#include <checkerboard_detector/CheckerboardDetectorConfig.h>

using namespace std;
using namespace ros;

class CheckerboardDetector
{
    template <typename T>
    static vector<T> tokenizevector(const string& s)
    {
        stringstream ss(s);
        return vector<T>((istream_iterator<T>(ss)), istream_iterator<T>());
    }

public:
    struct CHECKERBOARD
    {
        CvSize griddims; ///< number of squares
        vector<cv::Point3f> grid3d;
        //vector<CvPoint2D32f> corners;
        //cv::Mat corners;
        vector<cv::Point2f> corners;
        TransformMatrix tlocaltrans;
        std::string board_type;
    };

    posedetection_msgs::ObjectDetection _objdetmsg;
    sensor_msgs::CameraInfo _camInfoMsg;

    ros::Subscriber camInfoSubscriber,camInfoSubscriber2;
    ros::Subscriber imageSubscriber,imageSubscriber2;
    ros::Publisher _pubDetection;
    ros::Publisher _pubPoseStamped;
    ros::Publisher _pubCornerPoint;
    ros::Publisher _pubPolygonArray;
    ros::Publisher _pubDebugImage;
    ros::ServiceServer _srvDetect;
    int message_throttle_;
    int message_throttle_counter_;
    string frame_id; // tf frame id
    bool invert_color;
    int display, verbose, maxboard, queue_size, publish_queue_size;
    vector<CHECKERBOARD> vcheckers; // grid points for every checkerboard
    vector< string > vstrtypes; // type names for every grid point
    map<string,int> maptypes;
    ros::Time lasttime;
    boost::mutex mutex;
    ros::NodeHandle _node;
    int dimx, dimy;
    bool use_P;
    double fRectSize[2];
    typedef checkerboard_detector::CheckerboardDetectorConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv;
    bool adaptive_thresh_flag;
    bool filter_quads_flag;
    bool normalize_image_flag;
    bool fast_check_flag;
    double axis_size_;
    int circle_size_;

    //////////////////////////////////////////////////////////////////////////////
    // Constructor
    CheckerboardDetector()
    {
        _node.param("display", display, 0);
        _node.param("verbose", verbose, 1);
        _node.param("maxboard", maxboard, -1);
        _node.param("invert_color", invert_color, false);
        _node.param("use_P", use_P, false);
        _node.param("message_throttle", message_throttle_, 1);
        _node.param("queue_size", queue_size, 1);
        _node.param("publish_queue_size", publish_queue_size, 1);
        _node.param("axis_size", axis_size_, 0.05);
        _node.param("circle_size", circle_size_, 6);

        char str[32];
        int index = 0;

        srv = boost::make_shared <dynamic_reconfigure::Server<Config> > (_node);
        typename dynamic_reconfigure::Server<Config>::CallbackType f =
            boost::bind (&CheckerboardDetector::configCallback, this, _1, _2);
        srv->setCallback (f);

        while(1) {
            string type;

            sprintf(str,"grid%d_size_x",index);
            if( !_node.getParam(str,dimx) )
                break;
            if (dimx < 3) {
                ROS_ERROR("Param: %s must be greater than 2",str);
                return;
            }

            sprintf(str,"grid%d_size_y",index);
            if( !_node.getParam(str,dimy) )
                break;
            if (dimy < 3) {
                ROS_ERROR("Param: %s must be greater than 2",str);
                return;
            }

            sprintf(str,"rect%d_size_x",index);
            if( !_node.getParam(str,fRectSize[0]) )
                break;

            sprintf(str,"rect%d_size_y",index);
            if( !_node.getParam(str,fRectSize[1]) )
                break;

            sprintf(str,"type%d",index);
            if( !_node.getParam(str,type) ) {
                sprintf(str,"checker%dx%d", dimx, dimy);
                type = str;
            }

            std::string board_type;
            _node.param("board_type", board_type, std::string("chess"));
            
            
            string strtranslation,strrotation;
            sprintf(str,"translation%d",index);
            _node.param(str,strtranslation,string());

            vector<float> vtranslation = tokenizevector<float>(strtranslation);
            sprintf(str,"rotation%d",index);
            _node.param(str,strrotation,string());

            vector<float> vrotation = tokenizevector<float>(strrotation);

            CHECKERBOARD cb;
            cb.griddims = cvSize(dimx,dimy);
            cb.board_type = board_type;
            cb.grid3d.resize(dimx*dimy);
            int j=0;
            if (board_type == "chess" || board_type == "circle" || board_type == "circles") {
              for(int y=0; y<dimy; ++y)
                for(int x=0; x<dimx; ++x)
                  cb.grid3d[j++] = cv::Point3f(x*fRectSize[0], y*fRectSize[1], 0);
            }
            else if (board_type == "acircle" || board_type == "acircles") {
              for(int ii=0; ii<dimy; ii++) {
                for(int jj=0; jj<dimx; jj++) {
                  cb.grid3d[j++] = cv::Point3f((2*jj + ii % 2)*fRectSize[0],
                                          ii*fRectSize[1],
                                          0);
                }
              }
            }

            if( vtranslation.size() == 3 )
                cb.tlocaltrans.trans = 
                    Vector(vtranslation[0],vtranslation[1],vtranslation[2]);

            if( vrotation.size() == 9 )  {
                for(int k = 0; k < 3; ++k) {
                    cb.tlocaltrans.m[4*k+0] = vrotation[3*k+0];
                    cb.tlocaltrans.m[4*k+1] = vrotation[3*k+1];
                    cb.tlocaltrans.m[4*k+2] = vrotation[3*k+2];
                }
            }

            vcheckers.push_back(cb);
            vstrtypes.push_back(type);
            maptypes[vstrtypes.back()] = index;
            index++;
        }

        _node.param("frame_id", frame_id,string(""));

        if( maptypes.size() == 0 ) {
            ROS_ERROR("no checkerboards to detect");
            return;
        }

        if( display ) {
          cv::namedWindow("Checkerboard Detector",
                          (display == 1? CV_WINDOW_NORMAL : display));
        }

        lasttime = ros::Time::now();
        if (!display) {
            ros::SubscriberStatusCallback connect_cb = boost::bind( &CheckerboardDetector::connectCb, this);
            _pubDetection =
                _node.advertise<posedetection_msgs::ObjectDetection> ("ObjectDetection", publish_queue_size,
                                                                      connect_cb, connect_cb);
            _pubPoseStamped =
                _node.advertise<geometry_msgs::PoseStamped> ("objectdetection_pose", publish_queue_size,
                                                             connect_cb, connect_cb);
            _pubCornerPoint = _node.advertise<geometry_msgs::PointStamped>("corner_point", publish_queue_size, connect_cb, connect_cb);
            _pubPolygonArray = _node.advertise<jsk_recognition_msgs::PolygonArray>("polygons", publish_queue_size, connect_cb, connect_cb);
            _pubDebugImage =
                _node.advertise<sensor_msgs::Image>("debug_image", publish_queue_size, connect_cb, connect_cb);
        }
        else {
            _pubDetection =
                _node.advertise<posedetection_msgs::ObjectDetection> ("ObjectDetection", publish_queue_size);
            _pubPoseStamped =
                _node.advertise<geometry_msgs::PoseStamped> ("objectdetection_pose", publish_queue_size);
            _pubCornerPoint = _node.advertise<geometry_msgs::PointStamped>("corner_point", publish_queue_size);
            _pubPolygonArray = _node.advertise<jsk_recognition_msgs::PolygonArray>("polygons", publish_queue_size);
            _pubDebugImage = _node.advertise<sensor_msgs::Image>("debug_image", publish_queue_size);
            subscribe();
        }
        //this->camInfoSubscriber = _node.subscribe("camera_info", 1, &CheckerboardDetector::caminfo_cb, this);
        //this->imageSubscriber = _node.subscribe("image",1, &CheckerboardDetector::image_cb, this);
        //this->camInfoSubscriber2 = _node.subscribe("CameraInfo", 1, &CheckerboardDetector::caminfo_cb2, this);
        //this->imageSubscriber2 = _node.subscribe("Image",1, &CheckerboardDetector::image_cb2, this);
        _srvDetect = _node.advertiseService("Detect",&CheckerboardDetector::detect_cb,this);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Destructor
    virtual ~CheckerboardDetector()
    {
    }

    //////////////////////////////////////////////////////////////////////////////
    // Camera info callback
    void caminfo_cb(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        boost::mutex::scoped_lock lock(this->mutex);

        this->_camInfoMsg = *msg;

        // only get the camera info once <- this is dumb
        //this->camInfoSubscriber.shutdown();
        //this->camInfoSubscriber2.shutdown();
    }
    void caminfo_cb2(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        ROS_WARN("The topic CameraInfo has been deprecated.  Please change your launch file to use camera_info instead.");
        caminfo_cb(msg);
    }
    
    void publishPolygonArray(const posedetection_msgs::ObjectDetection& obj)
    {
        jsk_recognition_msgs::PolygonArray polygon_array;
        polygon_array.header = obj.header;
        for (size_t i = 0; i < obj.objects.size(); i++) {
            geometry_msgs::Pose pose = obj.objects[i].pose;
            Eigen::Affine3d affine;
            tf::poseMsgToEigen(pose, affine);
            Eigen::Vector3d A_local(0, 0, 0);
            Eigen::Vector3d B_local((dimx - 1) * fRectSize[0], 0, 0);
            Eigen::Vector3d C_local((dimx - 1) * fRectSize[0], (dimy - 1) * fRectSize[1], 0);
            Eigen::Vector3d D_local(0, (dimy - 1) * fRectSize[1], 0);
            Eigen::Vector3d A_global = affine * A_local;
            Eigen::Vector3d B_global = affine * B_local;
            Eigen::Vector3d C_global = affine * C_local;
            Eigen::Vector3d D_global = affine * D_local;
            geometry_msgs::Point32 a, b, c, d;
            a.x = A_global[0]; a.y = A_global[1]; a.z = A_global[2];
            b.x = B_global[0]; b.y = B_global[1]; b.z = B_global[2];
            c.x = C_global[0]; c.y = C_global[1]; c.z = C_global[2];
            d.x = D_global[0]; d.y = D_global[1]; d.z = D_global[2];
            geometry_msgs::PolygonStamped polygon;
            polygon.header = obj.header;
            polygon.polygon.points.push_back(a);
            polygon.polygon.points.push_back(b);
            polygon.polygon.points.push_back(c);
            polygon.polygon.points.push_back(d);
            polygon_array.polygons.push_back(polygon);
        }
        _pubPolygonArray.publish(polygon_array);
    }

    void image_cb2(const sensor_msgs::ImageConstPtr &msg)
    {
        ROS_WARN("The topic Image has been deprecated.  Please change your launch file to use image instead.");
        boost::mutex::scoped_lock lock(this->mutex);
        ++message_throttle_counter_;
        if (message_throttle_counter_ % message_throttle_ == 0) {
            message_throttle_counter_ = 0;
            if( Detect(_objdetmsg,*msg,this->_camInfoMsg) ) {
                if (_objdetmsg.objects.size() > 0) {
                    geometry_msgs::PoseStamped pose;
                    pose.header = _objdetmsg.header;
                    pose.pose = _objdetmsg.objects[0].pose;
                    _pubPoseStamped.publish(pose);
                }
                _pubDetection.publish(_objdetmsg);
                publishPolygonArray(_objdetmsg);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////////
    // Image data callback
    void image_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        boost::mutex::scoped_lock lock(this->mutex);
        ++message_throttle_counter_;
        if (message_throttle_counter_ % message_throttle_ == 0) {
            message_throttle_counter_ = 0;
            if( Detect(_objdetmsg,*msg,this->_camInfoMsg) ) {
                if (_objdetmsg.objects.size() > 0) {
                    geometry_msgs::PoseStamped pose;
                    pose.header = _objdetmsg.header;
                    pose.pose = _objdetmsg.objects[0].pose;
                    _pubPoseStamped.publish(pose);
                }
                _pubDetection.publish(_objdetmsg);
                publishPolygonArray(_objdetmsg);
            }
        }
    }

    bool detect_cb(posedetection_msgs::Detect::Request& req, posedetection_msgs::Detect::Response& res)
    {
        bool result = Detect(res.object_detection,req.image,req.camera_info);
        return result;
    }


    void subscribe( )
    {
        if ( camInfoSubscriber == NULL )
            camInfoSubscriber = _node.subscribe("camera_info", queue_size,
                                                &CheckerboardDetector::caminfo_cb, this);
        if ( imageSubscriber == NULL ) {
            imageSubscriber = _node.subscribe("image", queue_size,
                                              &CheckerboardDetector::image_cb, this);
            if ( imageSubscriber.getTopic().find("image_rect") != std::string::npos )
                ROS_WARN("topic name seems rectified, please use unrectified image"); // rectified image has 'image_rect' in topic name
        }
        if ( camInfoSubscriber2 == NULL )
            camInfoSubscriber2 = _node.subscribe("CameraInfo", queue_size, &CheckerboardDetector::caminfo_cb2, this);
        if ( imageSubscriber2 == NULL ) {
            imageSubscriber2 = _node.subscribe("Image", queue_size, &CheckerboardDetector::image_cb2, this);
            if ( imageSubscriber2.getTopic().find("image_rect") != std::string::npos )
                ROS_WARN("topic name seems rectified, please use unrectified image"); // rectified image has 'image_rect' in topic name
        }
    }

    void unsubscribe( )
    {
        camInfoSubscriber.shutdown();
        camInfoSubscriber2.shutdown();
        imageSubscriber.shutdown();
        imageSubscriber2.shutdown();
    }
    
    void connectCb( )
    {
      boost::mutex::scoped_lock lock(this->mutex);
      if (_pubDetection.getNumSubscribers() == 0 && _pubCornerPoint.getNumSubscribers() == 0 &&
          _pubPoseStamped.getNumSubscribers() == 0 && _pubPolygonArray.getNumSubscribers() == 0 &&
          _pubDebugImage.getNumSubscribers() == 0)
        {
            unsubscribe();
        }
      else
        {
            subscribe();
        }
    }

    bool Detect(posedetection_msgs::ObjectDetection& objdetmsg,
                const sensor_msgs::Image& imagemsg,
                const sensor_msgs::CameraInfo& camInfoMsg)
    {
        image_geometry::PinholeCameraModel model;
        sensor_msgs::CameraInfo cam_info(camInfoMsg);
        if (cam_info.distortion_model.empty()) {
            cam_info.distortion_model = "plumb_bob";
            cam_info.D.resize(5, 0);
        }
        if (use_P) {
            for (size_t i = 0; i < cam_info.D.size(); i++) {
                cam_info.D[i] = 0.0;
            }
        }
        // check all the value of R is zero or not
        // if zero, normalzie it
        if (use_P || std::equal(cam_info.R.begin() + 1, cam_info.R.end(), cam_info.R.begin())) {
            cam_info.R[0] = 1.0;
            cam_info.R[4] = 1.0;
            cam_info.R[8] = 1.0;
        }
        // check all the value of K is zero or not
        // if zero, copy all the value from P
        if (use_P || std::equal(cam_info.K.begin() + 1, cam_info.K.end(), cam_info.K.begin())) {
            cam_info.K[0] = cam_info.P[0];
            cam_info.K[1] = cam_info.P[1];
            cam_info.K[2] = cam_info.P[2];
            cam_info.K[3] = cam_info.P[4];
            cam_info.K[4] = cam_info.P[5];
            cam_info.K[5] = cam_info.P[6];
            cam_info.K[6] = cam_info.P[8];
            cam_info.K[7] = cam_info.P[9];
            cam_info.K[8] = cam_info.P[10];
        }
        model.fromCameraInfo(cam_info);
        cv_bridge::CvImagePtr capture_ptr;
        try {
          if (imagemsg.encoding == "32FC1") {
            cv_bridge::CvImagePtr float_capture
              = cv_bridge::toCvCopy(imagemsg,
                                    sensor_msgs::image_encodings::TYPE_32FC1);
            cv::Mat float_image = float_capture->image;
            cv::Mat mono_image;
            float_image.convertTo(mono_image, CV_8UC1);
            capture_ptr.reset(new cv_bridge::CvImage());
            capture_ptr->image = mono_image;
          }
          else {
            capture_ptr = cv_bridge::toCvCopy(imagemsg, sensor_msgs::image_encodings::MONO8);
          }
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("failed to get image %s", e.what());
            return false;
        }
        cv::Mat capture = capture_ptr->image;
        if (invert_color) {
            capture = cv::Mat((capture + 0.0) * 1.0 / 1.0) * 1.0;
            //capture = 255 - capture;
            cv::Mat tmp;
            cv::bitwise_not(capture, tmp);
            capture = tmp;
        }

        cv::Mat frame;

        cv::cvtColor(capture, frame, CV_GRAY2BGR);

        vector<posedetection_msgs::Object6DPose> vobjects;

#pragma omp parallel for schedule(dynamic,1)
        for(int i = 0; i < (int)vcheckers.size(); ++i) {
            CHECKERBOARD& cb = vcheckers[i];
            int ncorners, board=0;
            posedetection_msgs::Object6DPose objpose;

            // do until no more checkerboards detected
            while((maxboard==-1)?1:((++board)<=maxboard)) {
                bool allfound = false;
                if (cb.board_type == "chess") {
                    allfound = cv::findChessboardCorners(
                        capture, cb.griddims, cb.corners,
                        (adaptive_thresh_flag ? CV_CALIB_CB_ADAPTIVE_THRESH : 0) |
                        (normalize_image_flag ? CV_CALIB_CB_NORMALIZE_IMAGE : 0) |
                        (filter_quads_flag ? CV_CALIB_CB_FILTER_QUADS : 0) |
                        (fast_check_flag ? CV_CALIB_CB_FAST_CHECK : 0)
                        );
                }
                else if (cb.board_type == "circle" ||
                         cb.board_type == "circles") {
                    allfound =
                        cv::findCirclesGrid(capture, cb.griddims, cb.corners);
                }
                else if (cb.board_type == "acircle" ||
                         cb.board_type == "acircles") {
                    // sometime cv::findCirclesGrid hangs
                    allfound =
                        cv::findCirclesGrid(
                            capture, cb.griddims, cb.corners,
                            cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
                }

                if(!allfound || cb.corners.size() != cb.grid3d.size())
                    break;

                // remove any corners that are close to the border
                const int borderthresh = 30;

                for(int j = 0; j < ncorners; ++j) {
                    int x = cb.corners[j].x;
                    int y = cb.corners[j].y;
                    if( x < borderthresh || x > capture.cols - borderthresh ||
                        y < borderthresh || y > capture.rows - borderthresh )
                    {
                        allfound = false;
                        break;
                    }
                }

                // mark out the image
                cv::Point upperleft, lowerright;
                upperleft.x = lowerright.x = cb.corners[0].x;
                upperleft.y = lowerright.y = cb.corners[0].y;
                for(size_t j = 1; j < cb.corners.size(); ++j) {
                    if( upperleft.x > cb.corners[j].x ) upperleft.x = cb.corners[j].x;
                    if( upperleft.y > cb.corners[j].y ) upperleft.y = cb.corners[j].y;
                    if( lowerright.x < cb.corners[j].x ) lowerright.x = cb.corners[j].x;
                    if( lowerright.y < cb.corners[j].y ) lowerright.y = cb.corners[j].y;
                }

                float step_size =
                  (double)( ((upperleft.x - lowerright.x) * (upperleft.x - lowerright.x)) +
                            ((upperleft.y - lowerright.y) * (upperleft.y - lowerright.y)) )
                  /
                  ( ((cb.griddims.width - 1) * (cb.griddims.width - 1)) +
                    ((cb.griddims.height - 1) * (cb.griddims.height - 1)) );
#if 0
                ROS_INFO("(%d %d) - (%d %d) -> %f ",
                         upperleft.x, upperleft.y, lowerright.x, lowerright.y, sqrt(step_size));
#endif
                int size = (int)(0.5*sqrt(step_size) + 0.5);

                if( allfound ) {
                    if (cb.board_type == "chess") { // subpixel only for chessboard
                        cv::cornerSubPix(capture, cb.corners,
                                         cv::Size(size,size), cv::Size(-1,-1),
                                         cv::TermCriteria(CV_TERMCRIT_ITER, 50, 1e-2));
                    }
                    objpose.pose = FindTransformation(cb.corners, cb.grid3d, cb.tlocaltrans, model, size);
                }

#pragma omp critical
                {
                    if( allfound ) {
                        vobjects.push_back(objpose);
                        vobjects.back().type = vstrtypes[i];
                    }
                    cv::rectangle(capture, upperleft, lowerright,
                                  cv::Scalar(0,0,0), CV_FILLED);
                }
            }

            //cvSaveImage("temp.jpg", pimggray);
        }

        objdetmsg.objects = vobjects;
        objdetmsg.header.stamp = imagemsg.header.stamp;
        if( frame_id.size() > 0 )
            objdetmsg.header.frame_id = frame_id;
        else
            objdetmsg.header.frame_id = imagemsg.header.frame_id;

        if( verbose > 0 )
            ROS_INFO("checkerboard: image: %ux%u (size=%u), num: %u, total: %.3fs",
                     imagemsg.width, imagemsg.height,
                     (unsigned int)imagemsg.data.size(), (unsigned int)objdetmsg.objects.size(),
                     (float)(ros::Time::now() - lasttime).toSec());
        lasttime = ros::Time::now();

        // draw each found checkerboard
        for(size_t i = 0; i < vobjects.size(); ++i) {
            int itype = maptypes[vobjects[i].type];
            CHECKERBOARD& cb = vcheckers[itype];
            Transform tglobal;
            tglobal.trans = Vector(vobjects[i].pose.position.x,vobjects[i].pose.position.y,vobjects[i].pose.position.z);
            tglobal.rot = Vector(vobjects[i].pose.orientation.w,vobjects[i].pose.orientation.x,vobjects[i].pose.orientation.y, vobjects[i].pose.orientation.z);
            Transform tlocal = tglobal * cb.tlocaltrans.inverse();

            // draw all the points
            int csize0 = std::max(circle_size_, 6);
            int cwidth0 = std::max(circle_size_/3, 2);
            int csize1 = std::max((2*circle_size_)/3, 4);
            int csize2 = std::max(circle_size_/4, 2);
            int cwidth1 = std::max(circle_size_/2, 4);
            int cwidth2 = std::max(circle_size_/2, 3);

            for(size_t i = 0; i < cb.grid3d.size(); ++i) {
                Vector grid3d_vec(cb.grid3d[i].x, cb.grid3d[i].y, cb.grid3d[i].z);
                Vector p = tlocal * grid3d_vec;
                dReal fx = p.x*camInfoMsg.P[0] + p.y*camInfoMsg.P[1] + p.z*camInfoMsg.P[2] + camInfoMsg.P[3];
                dReal fy = p.x*camInfoMsg.P[4] + p.y*camInfoMsg.P[5] + p.z*camInfoMsg.P[6] + camInfoMsg.P[7];
                dReal fz = p.x*camInfoMsg.P[8] + p.y*camInfoMsg.P[9] + p.z*camInfoMsg.P[10] + camInfoMsg.P[11];
                int x = (int)(fx/fz);
                int y = (int)(fy/fz);
                cv::circle(frame, cv::Point(x,y), csize0, cv::Scalar(0, 255, 0), cwidth0);
                cv::circle(frame, cv::Point(x,y), csize1, cv::Scalar(64*itype,128,128), cwidth1);
                cv::circle(frame, cv::Point(x,y), csize2, cv::Scalar(0,   0, 0), cwidth0);
            }

            cv::Point X[4];

            Vector vaxes[4];
            vaxes[0] = Vector(0,0,0);
            vaxes[1] = Vector(axis_size_,0,0);
            vaxes[2] = Vector(0,axis_size_,0);
            vaxes[3] = Vector(0,0,axis_size_);

            for(int i = 0; i < 4; ++i) {
                Vector p = tglobal*vaxes[i];
                dReal fx = p.x*camInfoMsg.P[0] + p.y*camInfoMsg.P[1] + p.z*camInfoMsg.P[2] + camInfoMsg.P[3];
                dReal fy = p.x*camInfoMsg.P[4] + p.y*camInfoMsg.P[5] + p.z*camInfoMsg.P[6] + camInfoMsg.P[7];
                dReal fz = p.x*camInfoMsg.P[8] + p.y*camInfoMsg.P[9] + p.z*camInfoMsg.P[10] + camInfoMsg.P[11];
                X[i].x = (int)(fx/fz);
                X[i].y = (int)(fy/fz);
            }

            cv::circle(frame, X[0], cwidth2, cv::Scalar(255,255,128), cwidth2);

            // draw three lines
            cv::Scalar col0(255,0,(64*itype)%256); // B
            cv::Scalar col1(0,255,(64*itype)%256); // G
            cv::Scalar col2((64*itype)%256,(64*itype)%256,255); // R
            int axis_width_ = floor(axis_size_ / 0.03);
            cv::line(frame, X[0], X[3], col0, axis_width_);
            cv::line(frame, X[0], X[2], col1, axis_width_);
            cv::line(frame, X[0], X[1], col2, axis_width_);

            // publish X[0]
            geometry_msgs::PointStamped point_msg;
            point_msg.header = imagemsg.header;
            point_msg.point.x = X[0].x;
            point_msg.point.y = X[0].y;
            point_msg.point.z = vobjects[vobjects.size() - 1].pose.position.z;
            _pubCornerPoint.publish(point_msg);
        }

        // publish debug image
        _pubDebugImage.publish(
          cv_bridge::CvImage(imagemsg.header, sensor_msgs::image_encodings::RGB8, frame).toImageMsg());

        if( display ) {
            cv::imshow("Checkerboard Detector",frame);
            cv::waitKey(1);
        }

        return true;
    }


    //////////////////////////////////////////////////////////////////////////////
    // FindTransformation
    geometry_msgs::Pose FindTransformation(
        const vector<cv::Point2f> &imgpts, const vector<cv::Point3f> &objpts,
        const Transform& tlocal,
        const image_geometry::PinholeCameraModel& model,
        double cell_size = 1.0)
    {
        geometry_msgs::Pose pose;
        Transform tchecker;
        cv::Mat R3_mat, T3_mat;
        cv::solvePnP(objpts, imgpts,
                     model.intrinsicMatrix(),
                     model.distortionCoeffs(),
                     R3_mat, T3_mat, false);
        double fR3[3];
        for (size_t i = 0; i < 3; i++) {
          fR3[i] = R3_mat.at<double>(i);
        }
        tchecker.trans.x = T3_mat.at<double>(0);
        tchecker.trans.y = T3_mat.at<double>(1);
        tchecker.trans.z = T3_mat.at<double>(2);
        double fang = sqrt(fR3[0]*fR3[0] + fR3[1]*fR3[1] + fR3[2]*fR3[2]);
        if( fang >= 1e-6 ) {
            double fmult = sin(fang/2)/fang;
            tchecker.rot = Vector(cos(fang/2),fR3[0]*fmult, fR3[1]*fmult, fR3[2]*fmult);
        }

        // check if points are in truth position
        cv::Mat projpoints(objpts.size(), 2, CV_32FC1);
        projectPoints(objpts, R3_mat, T3_mat, model.intrinsicMatrix(), model.distortionCoeffs(), projpoints);
        const cv::Point2f* projpoints_ptr = projpoints.ptr<cv::Point2f>();
        float max_diff_norm = -10;
        for ( size_t i = 0; i < objpts.size(); ++i){
            float diff_norm = (float)norm(imgpts[i] - projpoints_ptr[i]);
            if (diff_norm > max_diff_norm) {
                max_diff_norm = diff_norm;
            }
        }
        ROS_DEBUG("max error of cells: %f", max_diff_norm/cell_size);
        if (max_diff_norm/cell_size > 0.02) { // 2%
            ROS_WARN("Large error %f detected, please check (1): checker board is on plane, (2): you uses rectified image", max_diff_norm/cell_size);
        }

        Transform tglobal = tchecker*tlocal;
        pose.position.x = tglobal.trans.x;
        pose.position.y = tglobal.trans.y;
        pose.position.z = tglobal.trans.z;
        pose.orientation.x = tglobal.rot.y;
        pose.orientation.y = tglobal.rot.z;
        pose.orientation.z = tglobal.rot.w;
        pose.orientation.w = tglobal.rot.x;
        return pose;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    void configCallback(Config &config, uint32_t level)
    {
        boost::mutex::scoped_lock lock(this->mutex);
        adaptive_thresh_flag = config.adaptive_thresh;
        normalize_image_flag = config.normalize_image;
        filter_quads_flag = config.filter_quads;
        fast_check_flag = config.fast_check;
    }
};

////////////////////////////////////////////////////////////////////////////////
// MAIN
int main(int argc, char **argv)
{
    ros::init(argc,argv,"checkerboard_detector");
    if( !ros::master::check() )
        return 1;
  
    CheckerboardDetector cd;
    ros::spin();

    return 0;
}
