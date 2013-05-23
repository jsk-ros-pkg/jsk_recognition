// Software License Agreement (BSD License)
// Copyright (c) 2008, Willow Garage, Inc.
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
#include <cstdio>
#include <vector>
#include <sstream>
#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "posedetection_msgs/ObjectDetection.h"
#include "posedetection_msgs/Detect.h"
#include "math.h"

#include <sys/timeb.h>    // ftime(), struct timeb
#include <sys/time.h>

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
        vector<Vector> grid3d;
        vector<CvPoint2D32f> corners;
        TransformMatrix tlocaltrans;
    };

    posedetection_msgs::ObjectDetection _objdetmsg;
    cv_bridge::CvImagePtr capture;
    sensor_msgs::CameraInfo _camInfoMsg;

    ros::Subscriber camInfoSubscriber,camInfoSubscriber2;
    ros::Subscriber imageSubscriber,imageSubscriber2;
    ros::Publisher _pubDetection;
    ros::ServiceServer _srvDetect;

    string frame_id; // tf frame id

    int display, verbose, maxboard;
    vector<CHECKERBOARD> vcheckers; // grid points for every checkerboard
    vector< string > vstrtypes; // type names for every grid point
    map<string,int> maptypes;
    ros::Time lasttime;
    CvMat *intrinsic_matrix; // intrinsic matrices
    boost::mutex mutexcalib;
    IplImage* frame;

    ros::NodeHandle _node;

    //////////////////////////////////////////////////////////////////////////////
    // Constructor
    CheckerboardDetector() : intrinsic_matrix(NULL), frame(NULL)
    {
        _node.param("display", display, 0);
        _node.param("verbose", verbose, 1);
        _node.param("maxboard", maxboard, -1);
        
        char str[32];
        int index = 0;

        while(1) {
            int dimx, dimy;
            double fRectSize[2];
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

            string strtranslation,strrotation;
            sprintf(str,"translation%d",index);
            _node.param(str,strtranslation,string());

            vector<float> vtranslation = tokenizevector<float>(strtranslation);
            sprintf(str,"rotation%d",index);
            _node.param(str,strrotation,string());

            vector<float> vrotation = tokenizevector<float>(strrotation);

            CHECKERBOARD cb;
            cb.griddims = cvSize(dimx,dimy);

            cb.grid3d.resize(dimx*dimy);
            int j=0;
            for(int y=0; y<dimy; ++y)
                for(int x=0; x<dimx; ++x)
                    cb.grid3d[j++] = Vector(x*fRectSize[0], y*fRectSize[1], 0);

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
	  // enable to set other window flag // use display value if display != 1 because only CV_WINDOW_AUTOSIZE is supported officially
	  cvNamedWindow("Checkerboard Detector", (display == 1? CV_WINDOW_AUTOSIZE : display));
            cvStartWindowThread();
        }

        lasttime = ros::Time::now();

        ros::SubscriberStatusCallback connect_cb = boost::bind( &CheckerboardDetector::connectCb, this);
        _pubDetection =
          _node.advertise<posedetection_msgs::ObjectDetection> ("ObjectDetection", 1,
                                                                connect_cb, connect_cb);

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
        if( frame )
            cvReleaseImage(&frame);
        if( this->intrinsic_matrix )
            cvReleaseMat(&this->intrinsic_matrix);
        _srvDetect.shutdown();
        this->camInfoSubscriber.shutdown();
        this->imageSubscriber.shutdown();
        this->camInfoSubscriber2.shutdown();
        this->imageSubscriber2.shutdown();
    }

    //////////////////////////////////////////////////////////////////////////////
    // Camera info callback
    void caminfo_cb(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        boost::mutex::scoped_lock lock(this->mutexcalib);

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

    void image_cb2(const sensor_msgs::ImageConstPtr &msg)
    {
        ROS_WARN("The topic Image has been deprecated.  Please change your launch file to use image instead.");
        boost::mutex::scoped_lock lock(this->mutexcalib);
        if( Detect(_objdetmsg,*msg,this->_camInfoMsg) )
            _pubDetection.publish(_objdetmsg);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Image data callback
    void image_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        boost::mutex::scoped_lock lock(this->mutexcalib);
        if( Detect(_objdetmsg,*msg,this->_camInfoMsg) )
            _pubDetection.publish(_objdetmsg);
    }

    bool detect_cb(posedetection_msgs::Detect::Request& req, posedetection_msgs::Detect::Response& res)
    {
        return Detect(res.object_detection,req.image,req.camera_info);
    }

    //
    void connectCb( )
    {
      boost::mutex::scoped_lock lock(this->mutexcalib);
      if (_pubDetection.getNumSubscribers() == 0)
        {
          camInfoSubscriber.shutdown();
          camInfoSubscriber2.shutdown();
          imageSubscriber.shutdown();
          imageSubscriber2.shutdown();
        }
      else
        {
          if ( camInfoSubscriber == NULL )
            camInfoSubscriber = _node.subscribe("camera_info", 1, &CheckerboardDetector::caminfo_cb, this);
          if ( imageSubscriber == NULL )
            imageSubscriber = _node.subscribe("image", 1, &CheckerboardDetector::image_cb, this);
          if ( camInfoSubscriber2 == NULL )
            camInfoSubscriber2 = _node.subscribe("CameraInfo", 1, &CheckerboardDetector::caminfo_cb2, this);
          if ( imageSubscriber2 == NULL )
            imageSubscriber2 = _node.subscribe("Image",1, &CheckerboardDetector::image_cb2, this);
        }
    }

    bool Detect(posedetection_msgs::ObjectDetection& objdetmsg, const sensor_msgs::Image& imagemsg, const sensor_msgs::CameraInfo& camInfoMsg)
    {
        if( this->intrinsic_matrix == NULL )
            this->intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);

        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                this->intrinsic_matrix->data.fl[3*i+j] = camInfoMsg.P[4*i+j];

        try {
            capture = cv_bridge::toCvCopy(imagemsg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("failed to get image %s", e.what());
            return false;
        }

        IplImage imggray = capture->image;
        IplImage *pimggray = &imggray;
        if( display ) {
            // copy the raw image
            if( frame != NULL && (frame->width != (int)imagemsg.width || frame->height != (int)imagemsg.height) ) {
                cvReleaseImage(&frame);
                frame = NULL;
            }
            imggray = capture->image;
            pimggray = &imggray;

            if( frame == NULL ) 
                frame = cvCreateImage(cvSize(imagemsg.width,imagemsg.height),IPL_DEPTH_8U, 3);

            cvCvtColor(pimggray,frame,CV_GRAY2RGB);
        }

        vector<posedetection_msgs::Object6DPose> vobjects;

#pragma omp parallel for schedule(dynamic,1)
        for(int i = 0; i < (int)vcheckers.size(); ++i) {
            CHECKERBOARD& cb = vcheckers[i];
            int ncorners, board=0;
            posedetection_msgs::Object6DPose objpose;

            // do until no more checkerboards detected
            while((maxboard==-1)?1:((++board)<=maxboard)) {
                cb.corners.resize(200);
                int allfound = cvFindChessboardCorners( pimggray, cb.griddims, &cb.corners[0], &ncorners,
                                                        CV_CALIB_CB_ADAPTIVE_THRESH );
                cb.corners.resize(ncorners);

                //cvDrawChessboardCorners(pimgGray, itbox->second.griddims, &corners[0], ncorners, allfound);
                //cvSaveImage("temp.jpg", pimgGray);

                if(!allfound || ncorners != (int)cb.grid3d.size())
                    break;

                // remove any corners that are close to the border
                const int borderthresh = 30;

                for(int j = 0; j < ncorners; ++j) {
                    int x = cb.corners[j].x;
                    int y = cb.corners[j].y;
                    if( x < borderthresh || x > pimggray->width-borderthresh ||
                        y < borderthresh || y > pimggray->height-borderthresh )
                    {
                        allfound = 0;
                        break;
                    }
                }

                // mark out the image
                CvPoint upperleft, lowerright;
                upperleft.x = lowerright.x = cb.corners[0].x;
                upperleft.y = lowerright.y = cb.corners[0].y;
                for(int j = 1; j < (int)cb.corners.size(); ++j) {
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
                    cvFindCornerSubPix(pimggray, &cb.corners[0], cb.corners.size(), cvSize(size,size), cvSize(-1,-1),
                                       cvTermCriteria(CV_TERMCRIT_ITER, 50, 1e-2));
                    objpose.pose = FindTransformation(cb.corners, cb.grid3d, cb.tlocaltrans);
                }

#pragma omp critical
                {
                    if( allfound ) {
                        vobjects.push_back(objpose);
                        vobjects.back().type = vstrtypes[i];
                    }

                    cvRectangle(pimggray, upperleft, lowerright, CV_RGB(0,0,0),CV_FILLED);
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
            ROS_INFO("checkerboard: image: %ux%u (size=%u), num: %u, total: %.3fs",imagemsg.width,imagemsg.height,
                     (unsigned int)imagemsg.data.size(), (unsigned int)objdetmsg.objects.size(),
                     (float)(ros::Time::now()-lasttime).toSec());
        lasttime = ros::Time::now();

        if( display ) {
            // draw each found checkerboard
            for(size_t i = 0; i < vobjects.size(); ++i) {
                int itype = maptypes[vobjects[i].type];
                CHECKERBOARD& cb = vcheckers[itype];
                Transform tglobal;
                tglobal.trans = Vector(vobjects[i].pose.position.x,vobjects[i].pose.position.y,vobjects[i].pose.position.z);
                tglobal.rot = Vector(vobjects[i].pose.orientation.w,vobjects[i].pose.orientation.x,vobjects[i].pose.orientation.y, vobjects[i].pose.orientation.z);
                Transform tlocal = tglobal * cb.tlocaltrans.inverse();

                CvPoint X[4];

                Vector vaxes[4];
                vaxes[0] = Vector(0,0,0);
                vaxes[1] = Vector(0.05f,0,0);
                vaxes[2] = Vector(0,0.05f,0);
                vaxes[3] = Vector(0,0,0.05f);

                for(int i = 0; i < 4; ++i) {
                    Vector p = tglobal*vaxes[i];
                    dReal fx = p.x*camInfoMsg.P[0] + p.y*camInfoMsg.P[1] + p.z*camInfoMsg.P[2] + camInfoMsg.P[3];
                    dReal fy = p.x*camInfoMsg.P[4] + p.y*camInfoMsg.P[5] + p.z*camInfoMsg.P[6] + camInfoMsg.P[7];
                    dReal fz = p.x*camInfoMsg.P[8] + p.y*camInfoMsg.P[9] + p.z*camInfoMsg.P[10] + camInfoMsg.P[11];
                    X[i].x = (int)(fx/fz);
                    X[i].y = (int)(fy/fz);
                }

                // draw three lines
                CvScalar col0 = CV_RGB(255,0,(64*itype)%256);
                CvScalar col1 = CV_RGB(0,255,(64*itype)%256);
                CvScalar col2 = CV_RGB((64*itype)%256,(64*itype)%256,255);
                cvLine(frame, X[0], X[1], col0, 1);
                cvLine(frame, X[0], X[2], col1, 1);
                cvLine(frame, X[0], X[3], col2, 1);

                // draw all the points
                for(size_t i = 0; i < cb.grid3d.size(); ++i) {
                    Vector p = tlocal * cb.grid3d[i];
                    dReal fx = p.x*camInfoMsg.P[0] + p.y*camInfoMsg.P[1] + p.z*camInfoMsg.P[2] + camInfoMsg.P[3];
                    dReal fy = p.x*camInfoMsg.P[4] + p.y*camInfoMsg.P[5] + p.z*camInfoMsg.P[6] + camInfoMsg.P[7];
                    dReal fz = p.x*camInfoMsg.P[8] + p.y*camInfoMsg.P[9] + p.z*camInfoMsg.P[10] + camInfoMsg.P[11];
                    int x = (int)(fx/fz);
                    int y = (int)(fy/fz);
                    cvCircle(frame, cvPoint(x,y), 6, CV_RGB(0,0,0), 2);
                    cvCircle(frame, cvPoint(x,y), 2, CV_RGB(0,0,0), 2);
                    cvCircle(frame, cvPoint(x,y), 4, CV_RGB(128,128,64*itype), 3);
                }

                cvCircle(frame, X[0], 3, CV_RGB(255,255,128), 3);
            }

            cvShowImage("Checkerboard Detector",frame);
        }

        return true;
    }


    //////////////////////////////////////////////////////////////////////////////
    // FindTransformation
    geometry_msgs::Pose FindTransformation(const vector<CvPoint2D32f> &imgpts, const vector<Vector> &objpts, const Transform& tlocal)
    {
        CvMat *objpoints = cvCreateMat(3,objpts.size(),CV_32FC1);
        for(size_t i=0; i<objpts.size(); ++i) {
            cvSetReal2D(objpoints, 0,i, objpts[i].x);
            cvSetReal2D(objpoints, 1,i, objpts[i].y);
            cvSetReal2D(objpoints, 2,i, objpts[i].z);
        }

        geometry_msgs::Pose pose;
        Transform tchecker;
        assert(sizeof(tchecker.trans.x)==sizeof(float));
        float fR3[3];
        CvMat R3, T3;
        assert(sizeof(pose.position.x) == sizeof(double));
        cvInitMatHeader(&R3, 3, 1, CV_32FC1, fR3);
        cvInitMatHeader(&T3, 3, 1, CV_32FC1, &tchecker.trans.x);

        float kc[4] = {0};
        CvMat kcmat;
        cvInitMatHeader(&kcmat,1,4,CV_32FC1,kc);

        CvMat img_points;
        cvInitMatHeader(&img_points, 1,imgpts.size(), CV_32FC2, const_cast<CvPoint2D32f*>(&imgpts[0]));

        cvFindExtrinsicCameraParams2(objpoints, &img_points, this->intrinsic_matrix, &kcmat, &R3, &T3);
        cvReleaseMat(&objpoints);

        double fang = sqrt(fR3[0]*fR3[0] + fR3[1]*fR3[1] + fR3[2]*fR3[2]);
        if( fang >= 1e-6 ) {
            double fmult = sin(fang/2)/fang;
            tchecker.rot = Vector(cos(fang/2),fR3[0]*fmult, fR3[1]*fmult, fR3[2]*fmult);
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
