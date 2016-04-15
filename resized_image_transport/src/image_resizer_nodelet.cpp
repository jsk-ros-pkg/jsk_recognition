#include "resized_image_transport/image_resizer_nodelet.h"

namespace resized_image_transport
{
  void ImageResizer::onInit() {
    raw_width_ = 0;
    raw_height_ = 0;
    DiagnosticNodelet::onInit();
    initParams();
    initReconfigure();
    initPublishersAndSubscribers();
    onInitPostProcess();
  }

  void ImageResizer::subscribe() {
    ImageProcessing::subscribe();
    sub_ = pnh_->subscribe("input/mask", 1, &ImageResizer::mask_region_callback, this);
  }

  void ImageResizer::unsubscribe() {
    ImageProcessing::unsubscribe();
    sub_.shutdown();
  }

  void ImageResizer::initReconfigure() {
    reconfigure_server_ = boost::make_shared <dynamic_reconfigure::Server<ImageResizerConfig> > (*pnh_);
    ReconfigureServer::CallbackType f
      = boost::bind(&ImageResizer::config_cb, this, _1, _2);
    reconfigure_server_->setCallback(f);
  }

  void ImageResizer::initParams() {
    ImageProcessing::initParams();
    period_ = ros::Duration(1.0);
    std::string interpolation_method;
    pnh_->param<std::string>("interpolation", interpolation_method, "LINEAR");
    if(interpolation_method == "NEAREST"){
      interpolation_ = cv::INTER_NEAREST;
    }else if(interpolation_method == "LINEAR") {
      interpolation_ = cv::INTER_LINEAR;
    }else if(interpolation_method == "AREA") {
      interpolation_ = cv::INTER_AREA;
    }else if(interpolation_method == "CUBIC") {
      interpolation_ = cv::INTER_CUBIC;
    }else if(interpolation_method == "LANCZOS4") {
      interpolation_ = cv::INTER_LANCZOS4;
    }else{
      ROS_ERROR("unknown interpolation method");
    }
  }

  void ImageResizer::mask_region_callback(const sensor_msgs::Image::ConstPtr& msg) {
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy
      (msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat mask = cv_ptr->image;
    int step_x, step_y, ox, oy;
    int pixel_x = 0;
    int pixel_y = 0;
    int maskwidth = mask.cols;
    int maskheight = mask.rows;
    int cnt = 0;
    for (size_t j = 0; j < maskheight; j++) {
      for (size_t i = 0; i < maskwidth; i++) {
        if (mask.at<uchar>(j, i) != 0) {
          cnt++;
        }
      }
    }
    int surface_per = ((double) cnt) / (maskwidth * maskheight) * 100;
    // step_x = surface_per /10;
    step_x = sqrt (surface_per);
    if (step_x < 1) {
      step_x = 1;
    }
    step_y = step_x;

    //raw image wo step de bunkatu pixel dasu
    ox = step_x / 2;
    oy = step_y / 2;
    for (int i = ox; i < raw_width_; i += step_x) {
      pixel_x++;
    }
    for (int i = oy; i < raw_height_; i += step_y) {
      pixel_y++;
    }
    resize_x_ = ((double) pixel_x) / raw_width_;
    resize_y_ = ((double) pixel_y) / raw_height_;
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
    if(use_camera_info_) {
      image_width = src_info->width;
      image_height = src_info->height;
    }
    else {
      image_width = src_img->width;
      image_height = src_img->height;
    }

    int width = dst_width_ ? dst_width_ : (resize_x_ * image_width);
    int height = dst_height_ ? dst_height_ : (resize_y_ * image_height);

    double scale_x = dst_width_ ? ((double)dst_width_)/image_width : resize_x_;
    double scale_y = dst_height_ ? ((double)dst_height_)/image_height : resize_y_;

    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(src_img);

    cv::Mat tmpmat(height, width, cv_img->image.type());
    if (raw_width_ == 0) {
      raw_width_ = tmpmat.cols;
      raw_height_ = tmpmat.rows;
    }
    cv::resize(cv_img->image, tmpmat, cv::Size(width, height), 0, 0, interpolation_);
    NODELET_DEBUG("mat rows:%d cols:%d", tmpmat.rows, tmpmat.cols);
    cv_img->image = tmpmat;

    dst_img = cv_img->toImageMsg();
    if (use_camera_info_) {
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
