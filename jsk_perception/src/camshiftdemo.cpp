#include <ros/ros.h>
#include <jsk_topic_tools/log_utils.h>
#include <geometry_msgs/PolygonStamped.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/camshiftdemoConfig.h>
#include <jsk_recognition_msgs/RotatedRectStamped.h>
#include <sensor_msgs/SetCameraInfo.h>

// opencv/samples/cp/camshiftdemo.c

#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <ctype.h>

#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/highgui/highgui_c.h>
#endif
using namespace cv;
using namespace std;

class CamShiftDemo
{
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_, pub_hist_;
  sensor_msgs::Image img_;
  ros::Subscriber sub_rectangle_;
  ros::Publisher pub_result_;
  ros::ServiceServer roi_service_;
  int max_queue_size_;
  bool debug_;
 
  typedef jsk_perception::camshiftdemoConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  ReconfigureServer reconfigure_server_;
  Config config_;

  Mat image_;
  Mat hsv_, hue_, mask_, histimg_, backproj_;

  // opencv 2.1 uses MatND which is removed in 2.2
  // this code will be removend in D-Turtle
#if (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION < 2)
  MatND hist_;
#else
  Mat hist_;
#endif

  bool backprojMode_;
  bool selectObject_;
  int trackObject_;
  bool showHist_;
  Point origin_;
  Rect selection_;
  int vmin_, vmax_, smin_;

  Rect trackWindow_;
  RotatedRect trackBox_;
  int hsize_;
  float hranges_[2];
  const float* phranges_;

  typedef struct {
    Mat *image;
    bool *selectObject;
    int *trackObject;
    Rect *selection;
    Point *origin;
  } on_mouse_param_type;
  on_mouse_param_type on_mouse_param_;

public:
  CamShiftDemo(ros::NodeHandle nh)
    :  it_(nh)
  {
    // ros setup
    ros::NodeHandle local_nh("~");
    local_nh.param("max_queue_size", max_queue_size_, 3);
    local_nh.param("debug", debug_, true);

    image_transport::ImageTransport it(nh);
    sub_ = it.subscribe(nh.resolveName("image"), 1, &CamShiftDemo::imageCB, this);
    pub_ = it.advertise(local_nh.resolveName("image"), 1);
    pub_hist_ = it.advertise(local_nh.resolveName("histimg"), 1);

    sub_rectangle_ = nh.subscribe(nh.resolveName("screenrectangle"), 1, &CamShiftDemo::setRectangleCB, this);
    pub_result_ = nh.advertise<jsk_recognition_msgs::RotatedRectStamped>(local_nh.resolveName("result"), 1);

    //roi_service_ = local_nh.advertiseService("set_roi", sensor_msgs::SetCameraInfo);
    roi_service_ = local_nh.advertiseService("set_roi", &CamShiftDemo::setROICb, this);
    
    // camshiftdemo.cpp
    backprojMode_ = false;
    selectObject_ = false;
    trackObject_ = 0;
    showHist_ = true;
    vmin_ = 10; vmax_ = 256; smin_ = 30;

    hsize_ = 16;
    hranges_[0] = 0; hranges_[1] = 180;
    phranges_ = hranges_;

    // Set up reconfigure server
    config_.vmin = vmin_;
    config_.vmax = vmax_;
    config_.smin = smin_;
    ReconfigureServer::CallbackType f = boost::bind(&CamShiftDemo::configCb, this, _1, _2);
    reconfigure_server_.setCallback(f);


    on_mouse_param_.image = &image_;
    on_mouse_param_.selectObject = &selectObject_;
    on_mouse_param_.trackObject = &trackObject_;
    on_mouse_param_.selection = &selection_;
    on_mouse_param_.origin = &origin_;

    if ( debug_ )
      {
        namedWindow( "Histogram", 1 );
        namedWindow( "CamShift Demo", 1 );

        setMouseCallback( "CamShift Demo", onMouse, (void *)&on_mouse_param_ );
        createTrackbar( "Vmin", "CamShift Demo", &vmin_, 256, 0 );
        createTrackbar( "Vmax", "CamShift Demo", &vmax_, 256, 0 );
        createTrackbar( "Smin", "CamShift Demo", &smin_, 256, 0 );

        histimg_ = Mat::zeros(200, 320, CV_8UC3);
      }

    if ( ! debug_ )
      {
        startWindowThread();
      }
  }

  ~CamShiftDemo()
  {
  }

  bool setROICb(sensor_msgs::SetCameraInfo::Request &req,
                sensor_msgs::SetCameraInfo::Response &res)
  {

    onMouse(CV_EVENT_LBUTTONDOWN,
            req.camera_info.roi.x_offset,
            req.camera_info.roi.y_offset,
            0,
            &on_mouse_param_);
    onMouse(CV_EVENT_LBUTTONUP,
            req.camera_info.roi.x_offset + req.camera_info.roi.width,
            req.camera_info.roi.y_offset + req.camera_info.roi.height,
            0,
            &on_mouse_param_);
    res.success = true;
    return true;
  }

  void imageCB(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    cv_bridge::CvImagePtr cv_ptr;
    Mat frame;

    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");
        frame = cv_ptr->image;
      }
    catch (cv_bridge::Exception error)
      {
        ROS_ERROR("error");
      }

    frame.copyTo(image_);
    cvtColor(image_, hsv_, CV_BGR2HSV);

    if( trackObject_ )
      {
        int _vmin = vmin_, _vmax = vmax_;

        inRange(hsv_, Scalar(0, smin_, MIN(_vmin,_vmax)),
                Scalar(180, 256, MAX(_vmin, _vmax)), mask_);
        int ch[] = {0, 0};
        hue_.create(hsv_.size(), hsv_.depth());
        mixChannels(&hsv_, 1, &hue_, 1, ch, 1);

        if( trackObject_ < 0 )
          {
            Mat roi(hue_, selection_), maskroi(mask_, selection_);
            calcHist(&roi, 1, 0, maskroi, hist_, 1, &hsize_, &phranges_);
            normalize(hist_, hist_, 0, 255, CV_MINMAX);

            trackWindow_ = selection_;
            trackObject_ = 1;

            histimg_ = Scalar::all(0);
            int binW = histimg_.cols / hsize_;
            Mat buf(1, hsize_, CV_8UC3);
            for( int i = 0; i < hsize_; i++ )
              buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize_), 255, 255);
            cvtColor(buf, buf, CV_HSV2BGR);

            for( int i = 0; i < hsize_; i++ )
              {
                int val = saturate_cast<int>(hist_.at<float>(i)*histimg_.rows/255);
                rectangle( histimg_, Point(i*binW,histimg_.rows),
                           Point((i+1)*binW,histimg_.rows - val),
                           Scalar(buf.at<Vec3b>(i)), -1, 8 );
              }
          }

	try {
	  calcBackProject(&hue_, 1, 0, hist_, backproj_, &phranges_);
	  backproj_ &= mask_;
	  trackBox_ = CamShift(backproj_, trackWindow_,
					  TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	  if( backprojMode_ )
	    cvtColor( backproj_, image_, CV_GRAY2BGR );
	  ellipse( image_, trackBox_, Scalar(0,0,255), 3, CV_AA );

	  jsk_recognition_msgs::RotatedRectStamped result_msg;
	  result_msg.header = msg_ptr->header;
	  result_msg.rect.x = trackBox_.center.x;
	  result_msg.rect.y = trackBox_.center.y;
	  result_msg.rect.width = trackBox_.size.width;
	  result_msg.rect.height = trackBox_.size.height;
	  result_msg.rect.angle = trackBox_.angle; // degree->rad
	  pub_result_.publish(result_msg);

	} catch (...) {
	  ROS_WARN("illegal tracBox = x:%f y:%f width:%f height:%f angle:%f",
		   trackBox_.center.x, trackBox_.center.y,
		   trackBox_.size.width, trackBox_.size.height,
		   trackBox_.angle);
	}
      }

    if( selectObject_ && selection_.width > 0 && selection_.height > 0 )
      {
        Mat roi(image_, selection_);
        bitwise_not(roi, roi);
      }

    img_.header = msg_ptr->header;
    fillImage(img_, "bgr8", image_.rows, image_.cols, image_.step, const_cast<uint8_t*>(image_.data));
    pub_.publish(img_);
    fillImage(img_, "bgr8", histimg_.rows, histimg_.cols, histimg_.step, const_cast<uint8_t*>(histimg_.data));
    pub_hist_.publish(img_);

    if ( debug_ )
      {
        imshow( "CamShift Demo", image_ );
        imshow( "Histogram", histimg_ );

        char c = (char)waitKey(10);
        switch(c)
          {
          case 'b':
            backprojMode_ = !backprojMode_;
            break;
          case 'c':
            trackObject_ = 0;
            histimg_ = Scalar::all(0);
            break;
          case 'h':
            showHist_ = !showHist_;
            if( !showHist_ )
              destroyWindow( "Histogram" );
            else
              namedWindow( "Histogram", 1 );
            break;
          default:
            ;
          }
      }
  }

  void setRectangleCB(const geometry_msgs::PolygonStampedConstPtr& msg_ptr)
  {
    selection_.x = msg_ptr->polygon.points[0].x;
    selection_.y = msg_ptr->polygon.points[0].y;
    selection_.width  = msg_ptr->polygon.points[1].x - msg_ptr->polygon.points[0].x;
    selection_.height = msg_ptr->polygon.points[1].y - msg_ptr->polygon.points[0].y;
    selection_ &= Rect(0, 0, image_.cols, image_.rows);
    trackObject_ = -1;
  }

  void configCb(Config &config, uint32_t level)
  {
    config_ = config;
    vmin_ = config_.vmin;
    vmax_ = config_.vmax;
    smin_ = config_.smin;
    switch ( config_.display_mode )
      {
      case jsk_perception::camshiftdemo_RGB:
        backprojMode_ = false;
        break;
      case jsk_perception::camshiftdemo_Backproject:
        backprojMode_ = true;
        break;
      }
  }


  static void onMouse( int event, int x, int y, int flags, void* param)
  {
    on_mouse_param_type* on_mouse_param = (on_mouse_param_type *)param;
    Mat* image = on_mouse_param->image;
    bool *selectObject = on_mouse_param->selectObject;
    int *trackObject = on_mouse_param->trackObject;
    Rect *selection = on_mouse_param->selection;
    Point *origin = on_mouse_param->origin;

    if( *selectObject )
      {
        selection->x = MIN(x, origin->x);
        selection->y = MIN(y, origin->y);
        selection->width = std::abs(x - origin->x);
        selection->height = std::abs(y - origin->y);

        *selection &= Rect(0, 0, image->cols, image->rows);
      }

    switch( event )
      {
      case CV_EVENT_LBUTTONDOWN:
        *origin = Point(x,y);
        *selection = Rect(x,y,0,0);
        *selectObject = true;
        break;
      case CV_EVENT_LBUTTONUP:
        *selectObject = false;
        if( selection->width > 0 && selection->height > 0 )
          *trackObject = -1;
        break;
      }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camshiftdemo");
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN("%s: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./%s image:=<image topic>", argv[0], argv[0]);
  }

  CamShiftDemo c(n);

  ros::spin();

  return 0;
}
