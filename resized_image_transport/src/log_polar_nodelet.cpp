#include "resized_image_transport/log_polar_nodelet.h"

namespace resized_image_transport
{
  void LogPolar::onInit(){
    DiagnosticNodelet::onInit();
    initReconfigure();
    initParams();
    initPublishersAndSubscribers();
  }
  

  void LogPolar::initReconfigure(){
    reconfigure_server_ = boost::make_shared <dynamic_reconfigure::Server<LogPolarConfig> > (*pnh_);
    ReconfigureServer::CallbackType f
      = boost::bind(&LogPolar::config_cb, this, _1, _2);
    reconfigure_server_->setCallback(f);
  }

  void LogPolar::initParams(){
    ImageProcessing::initParams();
    period_ = ros::Duration(1.0);
    pnh_->param("log_polar_scale", log_polar_scale_, 100.0);
    NODELET_INFO("log polar scale : %f", log_polar_scale_);

    pnh_->param("inverse_log_polar", inverse_log_polar_, false);
    if (inverse_log_polar_){
      NODELET_INFO("log polar");
    }else{
      NODELET_INFO("inverse log polar");
    }
  }
  
  void LogPolar::config_cb ( LogPolarConfig &config, uint32_t level) {
    NODELET_INFO("config_cb");
    resize_x_ = config.resize_scale_x;
    resize_y_ = config.resize_scale_y;
    log_polar_scale_ = config.log_polar_scale;
    period_ = ros::Duration(1.0/config.msg_par_second);
    verbose_ = config.verbose;
    NODELET_DEBUG("resize_scale_x : %f", resize_x_);
    NODELET_DEBUG("resize_scale_y : %f", resize_y_);
    NODELET_DEBUG("log_polar_scale : %f", log_polar_scale_);
    NODELET_DEBUG("message period : %f", period_.toSec());
  }

  
  void LogPolar::process(const sensor_msgs::ImageConstPtr &src_img, const sensor_msgs::CameraInfoConstPtr &src_info,
                         sensor_msgs::ImagePtr &dst_img, sensor_msgs::CameraInfo &dst_info){
    int image_width, image_height;
    if(use_camera_info_){
      image_width = src_info->width;
      image_height = src_info->height;
    }else{
      image_width = src_img->width;
      image_height = src_img->height;
    }

    int width = dst_width_ ? dst_width_ : (resize_x_ * image_width);
    int height = dst_height_ ? dst_height_ : (resize_y_ * image_height);

    double scale_x = dst_width_ ? ((double)dst_width_)/image_width : resize_x_;
    double scale_y = dst_height_ ? ((double)dst_height_)/image_height : resize_y_;

    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(src_img);
#if ( CV_MAJOR_VERSION >= 4)
    IplImage src = cvIplImage(cv_img->image);
#else
    IplImage src = cv_img->image;
#endif
    IplImage* dst = cvCloneImage(&src);

    int log_polar_flags = CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS;
    if ( inverse_log_polar_ ){
      log_polar_flags += CV_WARP_INVERSE_MAP;
    }
    cvLogPolar( &src, dst, cvPoint2D32f(image_width/2, image_height/2), log_polar_scale_, log_polar_flags);
    // http://answers.opencv.org/question/23440/any-way-to-convert-iplimage-to-cvmat-in-opencv-300/
    cv_img->image = cv::cvarrToMat(dst);

    dst_img = cv_img->toImageMsg();
    if(use_camera_info_){
      dst_info = *src_info;
      dst_info.height = height;
      dst_info.width = width;
      dst_info.K[0] = dst_info.K[0] * scale_x; // fx
      dst_info.K[2] = dst_info.K[2] * scale_x; // cx
      dst_info.K[4] = dst_info.K[4] * scale_y; // fy
      dst_info.K[5] = dst_info.K[5] * scale_y; // cy

      dst_info.P[0] = dst_info.P[0] * scale_x; // fx
      dst_info.P[2] = dst_info.P[2] * scale_x; // cx
      dst_info.P[3] = dst_info.P[3] * scale_x; // T
      dst_info.P[5] = dst_info.P[5] * scale_y; // fy
      dst_info.P[6] = dst_info.P[6] * scale_y; // cy
    }
  }
}

#include <pluginlib/class_list_macros.h>
typedef resized_image_transport::LogPolar LogPolar;
PLUGINLIB_EXPORT_CLASS(LogPolar, nodelet::Nodelet);
