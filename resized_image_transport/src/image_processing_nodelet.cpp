#include <resized_image_transport/image_processing_nodelet.h>

#include <std_msgs/Float32.h>
#include <image_transport/camera_common.h>

namespace resized_image_transport
{
  void ImageProcessing::onInit() {
    DiagnosticNodelet::onInit();
    initReconfigure();
    initParams();
    initPublishersAndSubscribers();
  }

  void ImageProcessing::initReconfigure(){
  }

  void ImageProcessing::initParams(){
    publish_once_ = true;

    pnh_->param("resize_scale_x", resize_x_, 1.0);
    NODELET_INFO("resize_scale_x : %f", resize_x_);
    pnh_->param("resize_scale_y", resize_y_, 1.0);
    NODELET_INFO("resize_scale_y : %f", resize_y_);

    pnh_->param("width", dst_width_, 0);
    NODELET_INFO("width : %d", dst_width_);
    pnh_->param("height", dst_height_, 0);
    NODELET_INFO("height : %d", dst_height_);
    pnh_->param("use_camera_subscriber", use_camera_subscriber_, false);
    pnh_->param("max_queue_size", max_queue_size_, 5);
    pnh_->param("use_snapshot", use_snapshot_, false);
    pnh_->param("use_messages", use_messages_, true);
    if (use_messages_) {
      double d_period;
      pnh_->param("period", d_period, 1.0);
      period_ = ros::Duration(d_period);
      NODELET_INFO("use_messages : %d", use_messages_);
      NODELET_INFO("message period : %f", period_.toSec());
    }
    pnh_->param("use_bytes", use_bytes_, false);
    pnh_->param("use_camera_info", use_camera_info_, true);
  }

  void ImageProcessing::unsubscribe()
  {
    if (use_snapshot_) {
      sub_.shutdown();
    }
    if (use_camera_info_) {
      if (use_camera_subscriber_) {
        cs_.shutdown();
      }
      else {
        camera_info_sub_.shutdown();
        image_nonsync_sub_.shutdown();
      }
    }
    else {
      image_sub_.shutdown();
    }
  }
  
  void ImageProcessing::subscribe()
  {
    if (use_snapshot_) {
      sub_ = pnh_->subscribe("snapshot", 1, &ImageProcessing::snapshot_msg_cb, this);
    }
    if (use_camera_info_) {
      if (use_camera_subscriber_) {
        cs_ = it_->subscribeCamera("input/image", max_queue_size_,
                                   &ImageProcessing::callback, this);
      }
      else {
        camera_info_sub_ = nh_->subscribe(image_transport::getCameraInfoTopic(pnh_->resolveName("input/image")),
                                          1,
                                          &ImageProcessing::info_cb,
                                          this);
        image_nonsync_sub_ = it_->subscribe("input/image",
                                            1,
                                            &ImageProcessing::image_nonsync_cb,
                                            this);
      }
    }
    else {
      image_sub_ = pnh_->subscribe("input/image", max_queue_size_, &ImageProcessing::image_cb, this);
    }
  }


  void ImageProcessing::initPublishersAndSubscribers(){
    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    image_vital_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    info_vital_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    it_ = new image_transport::ImageTransport(*pnh_);
    std::string img = nh_->resolveName("image");
    std::string cam = nh_->resolveName("camera");
    if (img.at(0) == '/') {
      img.erase(0, 1);
    }
    NODELET_INFO("camera = %s", cam.c_str());
    NODELET_INFO("image = %s", img.c_str());
    
    width_scale_pub_ = advertise<std_msgs::Float32>(*pnh_, "output/width_scale", max_queue_size_);
    height_scale_pub_ = advertise<std_msgs::Float32>(*pnh_, "output/height_scale", max_queue_size_);
    
    if (use_snapshot_) {
      publish_once_ = false;
      srv_ = pnh_->advertiseService("snapshot", &ImageProcessing::snapshot_srv_cb, this);
    }

    if (use_camera_info_) {
#ifdef USE_2_0_9_ADVERTISE_CAMERA
      cp_ = advertiseCamera(*pnh_, *it_, "output/image", max_queue_size_);
#else
      cp_ = advertiseCamera(*pnh_, "output/image", max_queue_size_);
#endif
    }
    else {
      image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output/image", max_queue_size_);
    }
    
  }

  void ImageProcessing::snapshot_msg_cb (const std_msgs::EmptyConstPtr msg) {
    boost::mutex::scoped_lock lock(mutex_);
    publish_once_ = true;
  }

  bool ImageProcessing::snapshot_srv_cb (std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res) {
    boost::mutex::scoped_lock lock(mutex_);

    publish_once_ = true;
    return true;
  }

  void ImageProcessing::info_cb(const sensor_msgs::CameraInfoConstPtr &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    info_vital_->poke();
    info_msg_ = msg;
  }

  void ImageProcessing::image_nonsync_cb(const sensor_msgs::ImageConstPtr& msg) {
    {
      boost::mutex::scoped_lock lock(mutex_);
      image_vital_->poke();
      if (!info_msg_) {
        NODELET_WARN_THROTTLE(1, "camera info is not yet available");
        return;
      }
    }
    callback(msg, info_msg_);
  }
    
  void ImageProcessing::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(mutex_);
    
    if (!image_vital_ || !info_vital_) {
      // initialization is not finished
      return;
    }
    
    // common
    stat.add("use_camera_info", use_camera_info_);
    stat.add("use_snapshot", use_snapshot_);
    stat.add("input image", pnh_->resolveName("input/image"));
    stat.add("output image", pnh_->resolveName("output/image"));
    // summary
    if (!image_vital_->isAlive()) {
      stat.summary(diagnostic_error_level_,
                   "no image input. Is " + pnh_->resolveName("input/image") + " active?");
    }
    else {
      if (use_camera_info_) {
        if (!info_vital_->isAlive()) {
          stat.summary(diagnostic_error_level_,
                       "no info input. Is " + camera_info_sub_.getTopic() + " active?");
        }
        else {
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                       "running");
        }
        stat.add("input info", camera_info_sub_.getTopic());
        stat.add("info_last_received_time", info_vital_->lastAliveTimeRelative());
      }
      else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     "running");
      }
    }
    stat.add("image_last_received_time", image_vital_->lastAliveTimeRelative());
    ros::Time now = ros::Time::now();
    float duration =  (now - last_rosinfo_time_).toSec();
    int in_time_n = in_times.size();
    int out_time_n = out_times.size();
    double in_time_mean = 0, in_time_rate = 1.0, in_time_std_dev = 0.0, in_time_max_delta, in_time_min_delta;
    double out_time_mean = 0, out_time_rate = 1.0, out_time_std_dev = 0.0, out_time_max_delta, out_time_min_delta;

    std::for_each( in_times.begin(), in_times.end(), (in_time_mean += boost::lambda::_1) );
    in_time_mean /= in_time_n;
    in_time_rate /= in_time_mean;
    std::for_each( in_times.begin(), in_times.end(), (in_time_std_dev += (boost::lambda::_1 - in_time_mean)*(boost::lambda::_1 - in_time_mean) ) );
    in_time_std_dev = sqrt(in_time_std_dev/in_time_n);
    if (in_time_n > 1) {
      in_time_min_delta = *std::min_element(in_times.begin(), in_times.end());
      in_time_max_delta = *std::max_element(in_times.begin(), in_times.end());
    }

    std::for_each( out_times.begin(), out_times.end(), (out_time_mean += boost::lambda::_1) );
    out_time_mean /= out_time_n;
    out_time_rate /= out_time_mean;
    std::for_each( out_times.begin(), out_times.end(), (out_time_std_dev += (boost::lambda::_1 - out_time_mean)*(boost::lambda::_1 - out_time_mean) ) );
    out_time_std_dev = sqrt(out_time_std_dev/out_time_n);
    if (out_time_n > 1) {
      out_time_min_delta = *std::min_element(out_times.begin(), out_times.end());
      out_time_max_delta = *std::max_element(out_times.begin(), out_times.end());
    }

    double in_byte_mean = 0, out_byte_mean = 0;
    std::for_each( in_bytes.begin(), in_bytes.end(), (in_byte_mean += boost::lambda::_1) );
    std::for_each( out_bytes.begin(), out_bytes.end(), (out_byte_mean += boost::lambda::_1) );
    in_byte_mean /= duration;
    out_byte_mean /= duration;
    stat.add("compressed rate", in_byte_mean / out_byte_mean);
    stat.add("input bandwidth (Kbps)", in_byte_mean / 1000 * 8);
    stat.add("input bandwidth (Mbps)", in_byte_mean / 1000 / 1000 * 8);
    stat.add("input rate (hz)", in_time_rate);
    stat.add("input min delta (s)", in_time_min_delta);
    stat.add("input max delta (s)", in_time_max_delta);
    stat.add("input std_dev delta (s)", in_time_std_dev);
    stat.add("input times (n)", in_time_n);
    stat.add("output bandwidth (Kbps)", out_byte_mean / 1000 * 8);
    stat.add("output bandwidth (Mbps)", out_byte_mean / 1000 / 1000 * 8);
    stat.add("output rate (hz)", out_time_rate);
    stat.add("output min delta (s)", out_time_min_delta);
    stat.add("output max delta (s)", out_time_max_delta);
    stat.add("output std_dev delta (s)", out_time_std_dev);
    stat.add("output times (n)", out_time_n);
    in_times.clear();
    in_bytes.clear();
    out_times.clear();
    out_bytes.clear();
    last_rosinfo_time_ = now;

  }
  
  void ImageProcessing::image_cb(const sensor_msgs::ImageConstPtr &img){
    image_vital_->poke();
    callback(img, sensor_msgs::CameraInfo::ConstPtr());
  }

  void ImageProcessing::callback(const sensor_msgs::ImageConstPtr &img,
                                 const sensor_msgs::CameraInfoConstPtr &info) {
    boost::mutex::scoped_lock lock(mutex_);
    ros::Time now = ros::Time::now();
    ROS_DEBUG("image processing callback");
    if ( !publish_once_ || (cp_.getNumSubscribers () == 0 && image_pub_.getNumSubscribers () == 0 )) {
      ROS_DEBUG("number of subscribers is 0, ignoring image");
      return;
    }
    if (use_messages_ && now - last_publish_time_ < period_) {
      ROS_DEBUG("to reduce load, ignoring image");
      return;
    }
    in_times.push_front((now - last_subscribe_time_).toSec());
    in_bytes.push_front(img->data.size());

    try {
      sensor_msgs::ImagePtr dst_img;
      sensor_msgs::CameraInfo dst_info;
      process(img, info, dst_img, dst_info);
  

      if (use_camera_info_) {
        cp_.publish(dst_img,
                    boost::make_shared<sensor_msgs::CameraInfo> (dst_info));
      }
      else {
        image_pub_.publish(dst_img);
      }

      // TODO: it does not support dst_width_ and dst_height_
      std_msgs::Float32 width_scale;
      width_scale.data = resize_x_;
      std_msgs::Float32 height_scale;
      height_scale.data = resize_y_;
      width_scale_pub_.publish(width_scale);
      height_scale_pub_.publish(height_scale);
      
      out_times.push_front((now - last_publish_time_).toSec());
      out_bytes.push_front(dst_img->step * dst_img->height);
      
      last_publish_time_ = now;
    }
    catch (cv::Exception& e) {
      ROS_ERROR("%s", e.what());
    }

    last_subscribe_time_ = now;

    if (use_snapshot_) {
      publish_once_ = false;
    }
  }
}

