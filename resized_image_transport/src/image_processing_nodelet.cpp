#include <resized_image_transport/image_processing_nodelet.h>

namespace resized_image_transport
{
  void ImageProcessing::onInit() {
    initNodeHandle();
    initReconfigure();
    initParams();
    initPublishersAndSubscribers();
  }

  void ImageProcessing::initNodeHandle(){
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();
  }

  void ImageProcessing::initReconfigure(){
  }

  void ImageProcessing::initParams(){
    publish_once_ = true;

    pnh.param("resize_scale_x", resize_x_, 1.0);
    NODELET_INFO("resize_scale_x : %f", resize_x_);
    pnh.param("resize_scale_y", resize_y_, 1.0);
    NODELET_INFO("resize_scale_y : %f", resize_y_);

    pnh.param("width", dst_width_, 0);
    NODELET_INFO("width : %d", dst_width_);
    pnh.param("height", dst_height_, 0);
    NODELET_INFO("height : %d", dst_height_);

    pnh.param("max_queue_size", max_queue_size_, 5);

    pnh.param("use_snapshot", use_snapshot_, false);
    pnh.param("use_messages", use_messages_, true);
    if (use_messages_) {
      double d_period;
      pnh.param("period", d_period, 1.0);
      period_ = ros::Duration(d_period);
      NODELET_INFO("use_messages : %d", use_messages_);
      NODELET_INFO("message period : %f", period_.toSec());
    }
    pnh.param("use_bytes", use_bytes_, false);
    pnh.param("use_camera_info", use_camera_info_, true);
  }

  void ImageProcessing::initPublishersAndSubscribers(){
    it_ = new image_transport::ImageTransport(pnh);
    std::string img = nh.resolveName("image");
    std::string cam = nh.resolveName("camera");
    if (img.at(0) == '/') {
      img.erase(0, 1);
    }
    NODELET_INFO("camera = %s", cam.c_str());
    NODELET_INFO("image = %s", img.c_str());

    if (use_snapshot_) {
      publish_once_ = false;
      srv_ = pnh.advertiseService("snapshot", &ImageProcessing::snapshot_srv_cb, this);
      sub_ = pnh.subscribe("snapshot", 1, &ImageProcessing::snapshot_msg_cb, this);
    }

    if ( use_camera_info_ ){
      cp_ = it_->advertiseCamera("output/image", max_queue_size_);

      cs_ = it_->subscribeCamera("input/image", max_queue_size_,
				 &ImageProcessing::callback, this);
    }else{
      image_pub_ = pnh.advertise<sensor_msgs::Image>("output/image", max_queue_size_);
      image_sub_ = pnh.subscribe("input/image", max_queue_size_, &ImageProcessing::image_cb, this);
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

  void ImageProcessing::image_cb(const sensor_msgs::ImageConstPtr &img){
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
    if ( use_messages_ && now - last_publish_time_ < period_ ) {
      ROS_DEBUG("to reduce load, ignoring image");
      return;
    }
    in_times.push_front((now - last_subscribe_time_).toSec());
    in_bytes.push_front(img->data.size());

    try {
      sensor_msgs::ImagePtr dst_img;
      sensor_msgs::CameraInfo dst_info;
      process(img, info, dst_img, dst_info);
	

      if(use_camera_info_){
	cp_.publish(dst_img,
		    boost::make_shared<sensor_msgs::CameraInfo> (dst_info));
      }else{
	image_pub_.publish(dst_img);
      }
	  
      out_times.push_front((now - last_publish_time_).toSec());
      out_bytes.push_front(dst_img->step * dst_img->height);

      last_publish_time_ = now;
    } catch( cv::Exception& e ) {
      ROS_ERROR("%s", e.what());
    }


    float duration =  (now - last_rosinfo_time_).toSec();
    if ( duration > 2 ) {
      int in_time_n = in_times.size();
      int out_time_n = out_times.size();
      double in_time_mean = 0, in_time_rate = 1.0, in_time_std_dev = 0.0, in_time_max_delta, in_time_min_delta;
      double out_time_mean = 0, out_time_rate = 1.0, out_time_std_dev = 0.0, out_time_max_delta, out_time_min_delta;

      std::for_each( in_times.begin(), in_times.end(), (in_time_mean += boost::lambda::_1) );
      in_time_mean /= in_time_n;
      in_time_rate /= in_time_mean;
      std::for_each( in_times.begin(), in_times.end(), (in_time_std_dev += (boost::lambda::_1 - in_time_mean)*(boost::lambda::_1 - in_time_mean) ) );
      in_time_std_dev = sqrt(in_time_std_dev/in_time_n);
      if ( in_time_n > 1 ) {
	in_time_min_delta = *std::min_element(in_times.begin(), in_times.end());
	in_time_max_delta = *std::max_element(in_times.begin(), in_times.end());
      }

      std::for_each( out_times.begin(), out_times.end(), (out_time_mean += boost::lambda::_1) );
      out_time_mean /= out_time_n;
      out_time_rate /= out_time_mean;
      std::for_each( out_times.begin(), out_times.end(), (out_time_std_dev += (boost::lambda::_1 - out_time_mean)*(boost::lambda::_1 - out_time_mean) ) );
      out_time_std_dev = sqrt(out_time_std_dev/out_time_n);
      if ( out_time_n > 1 ) {
	out_time_min_delta = *std::min_element(out_times.begin(), out_times.end());
	out_time_max_delta = *std::max_element(out_times.begin(), out_times.end());
      }

      double in_byte_mean = 0, out_byte_mean = 0;
      std::for_each( in_bytes.begin(), in_bytes.end(), (in_byte_mean += boost::lambda::_1) );
      std::for_each( out_bytes.begin(), out_bytes.end(), (out_byte_mean += boost::lambda::_1) );
      in_byte_mean /= duration;
      out_byte_mean /= duration;

      if (verbose_) {
	NODELET_INFO_STREAM(" in  bandwidth: " << std::fixed << std::setw(11) << std::setprecision(3)  << in_byte_mean/1000*8
			    << " Kbps rate:"   << std::fixed << std::setw(7) << std::setprecision(3) << in_time_rate
			    << " hz min:"      << std::fixed << std::setw(7) << std::setprecision(3) << in_time_min_delta
			    << " s max: "    << std::fixed << std::setw(7) << std::setprecision(3) << in_time_max_delta
			    << " s std_dev: "<< std::fixed << std::setw(7) << std::setprecision(3) << in_time_std_dev << "s n: " << in_time_n);
	NODELET_INFO_STREAM(" out bandwidth: " << std::fixed << std::setw(11) << std::setprecision(3)  << out_byte_mean/1000*8
			    << " kbps rate:"   << std::fixed << std::setw(7) << std::setprecision(3) << out_time_rate
			    << " hz min:"      << std::fixed << std::setw(7) << std::setprecision(3) << out_time_min_delta
			    << " s max: "    << std::fixed << std::setw(7) << std::setprecision(3) << out_time_max_delta
			    << " s std_dev: "<< std::fixed << std::setw(7) << std::setprecision(3) << out_time_std_dev << "s n: " << out_time_n);
      }
      in_times.clear();
      in_bytes.clear();
      out_times.clear();
      out_bytes.clear();
      last_rosinfo_time_ = now;
    }

    last_subscribe_time_ = now;

    if(use_snapshot_) {
      publish_once_ = false;
    }
  }
}

