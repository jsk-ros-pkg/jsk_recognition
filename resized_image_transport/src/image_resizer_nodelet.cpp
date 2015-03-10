#include "resized_image_transport/image_resizer_nodelet.h"

namespace resized_image_transport
{
  void ImageResizer::onInit(){
    initNodeHandle();
    initParams();
    initReconfigure();
    initPublishersAndSubscribers();
  }

  void ImageResizer::initReconfigure(){
    ReconfigureServer::CallbackType f
      = boost::bind(&ImageResizer::config_cb, this, _1, _2);
    reconfigure_server_.setCallback(f);
  }

  void ImageResizer::initParams(){
    ImageProcessing::initParams();
    period_ = ros::Duration(1.0);
    std::string interpolation_method;
    pnh.param<std::string>("interpolation", interpolation_method, "LINEAR");
    if(interpolation_method == "NEAREST"){
      interpolation_ = cv::INTER_NEAREST;
    }else if(interpolation_method == "LINEAR"){
      interpolation_ = cv::INTER_LINEAR;
    }else if(interpolation_method == "AREA"){
      interpolation_ = cv::INTER_AREA;
    }else if(interpolation_method == "CUBIC"){
      interpolation_ = cv::INTER_CUBIC;
    }else if(interpolation_method == "LANCZOS4"){
      interpolation_ = cv::INTER_LANCZOS4;
    }else{
      ROS_ERROR("unknown interpolation method");
    }
  }

  void ImageResizer::config_cb ( ImageResizerConfig &config, uint32_t level) {
    NODELET_INFO("config_cb");
    resize_x_ = config.resize_scale_x;
    resize_y_ = config.resize_scale_y;
    period_ = ros::Duration(1.0/config.msg_par_second);
    verbose_ = config.verbose;
    NODELET_DEBUG("resize_scale_x : %f", resize_x_);
    NODELET_DEBUG("resize_scale_y : %f", resize_y_);
    NODELET_DEBUG("message period : %f", period_.toSec());
  }

  
  void ImageResizer::process(const sensor_msgs::ImageConstPtr &src_img, const sensor_msgs::CameraInfoConstPtr &src_info,
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

    cv::Mat tmpmat(height, width, cv_img->image.type());
    cv::resize(cv_img->image, tmpmat, cv::Size(width, height), 0, 0, interpolation_);
    cv_img->image = tmpmat;

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
typedef resized_image_transport::ImageResizer ImageResizer;
PLUGINLIB_EXPORT_CLASS(ImageResizer, nodelet::Nodelet);
