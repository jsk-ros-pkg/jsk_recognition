#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lambda/lambda.hpp>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "resized_image_transport/ImageResizerConfig.h"

namespace resized_image_transport
{
class ImageResizer: public nodelet::Nodelet
{
  // variables
public:
protected:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  // dynamic reconfigure
  typedef dynamic_reconfigure::Server<resized_image_transport::ImageResizerConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

  image_transport::CameraSubscriber cs_;
  image_transport::CameraPublisher cp_;
  image_transport::ImageTransport *it_;
  ros::ServiceServer srv_;
  ros::Subscriber sub_;

  double resize_x_, resize_y_;
  int dst_width_, dst_height_;
  int max_queue_size_;
  bool use_snapshot_;
  bool publish_once_;
  bool use_messages_;
  bool use_bytes_;
  ros::Time last_rosinfo_time_, last_subscribe_time_, last_publish_time_;
  ros::Duration period_;
  boost::mutex mutex_;

private:
  
  
public:
protected:
  void onInit() {
    nh = getNodeHandle();
    pnh = getPrivateNodeHandle();
    publish_once_ = true;
    ReconfigureServer::CallbackType f
      = boost::bind(&ImageResizer::config_cb, this, _1, _2);
    reconfigure_server_.setCallback(f);

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
    pnh.param("use_messages", use_messages_, false);
    if (use_messages_) {
        double d_period;
        pnh.param("period", d_period, 1.0);
        period_ = ros::Duration(d_period);
        NODELET_INFO("use_messages : %d", use_messages_);
        NODELET_INFO("message period : %f", period_.toSec());
    }
    pnh.param("use_bytes", use_bytes_, false);

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
      srv_ = pnh.advertiseService("snapshot", &ImageResizer::snapshot_srv_cb, this);
      sub_ = pnh.subscribe("snapshot", 1, &ImageResizer::snapshot_msg_cb, this);
    }

    cs_ = it_->subscribeCamera("input/image", max_queue_size_,
                               &ImageResizer::callback, this);

    cp_ = it_->advertiseCamera("output/image", max_queue_size_);

  }

  ~ImageResizer() { }

protected:

  void config_cb (resized_image_transport::ImageResizerConfig &config, uint32_t level) {
    NODELET_INFO("config_cb");
    resize_x_ = config.resize_scale_x;
    resize_y_ = config.resize_scale_y;
    period_ = ros::Duration(1.0/config.msg_par_second);

    NODELET_INFO("resize_scale_x : %f", resize_x_);
    NODELET_INFO("resize_scale_y : %f", resize_y_);
    NODELET_INFO("message period : %f", period_.toSec());
  }

  void snapshot_msg_cb (const std_msgs::EmptyConstPtr msg) {
    boost::mutex::scoped_lock lock(mutex_);

    publish_once_ = true;
  }

  bool snapshot_srv_cb (std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res) {
    boost::mutex::scoped_lock lock(mutex_);

    publish_once_ = true;
    return true;
  }

  void callback(const sensor_msgs::ImageConstPtr &img,
                const sensor_msgs::CameraInfoConstPtr &info) {
    boost::mutex::scoped_lock lock(mutex_);
    ros::Time now = ros::Time::now();

    static boost::circular_buffer<double> in_times(100);
    static boost::circular_buffer<double> out_times(100);
    static boost::circular_buffer<double> in_bytes(100);
    static boost::circular_buffer<double> out_bytes(100);

    ROS_DEBUG("resize: callback");
    if ( !publish_once_ || cp_.getNumSubscribers () == 0 ) {
      ROS_DEBUG("resize: number of subscribers is 0, ignoring image");
      return;
    }

    in_times.push_front((now - last_subscribe_time_).toSec());
    in_bytes.push_front(img->data.size());
    //
    try {
        int width = dst_width_ ? dst_width_ : (resize_x_ * info->width);
        int height = dst_height_ ? dst_height_ : (resize_y_ * info->height);
        double scale_x = dst_width_ ? ((double)dst_width_)/info->width : resize_x_;
        double scale_y = dst_height_ ? ((double)dst_height_)/info->height : resize_y_;

        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img);

        cv::Mat tmpmat(height, width, cv_img->image.type());
        cv::resize(cv_img->image, tmpmat, cv::Size(width, height));
        cv_img->image = tmpmat;

        sensor_msgs::CameraInfo tinfo = *info;
        tinfo.height = height;
        tinfo.width = width;
        tinfo.K[0] = tinfo.K[0] * scale_x; // fx
        tinfo.K[2] = tinfo.K[2] * scale_x; // cx
        tinfo.K[4] = tinfo.K[4] * scale_y; // fy
        tinfo.K[5] = tinfo.K[5] * scale_y; // cy

        tinfo.P[0] = tinfo.P[0] * scale_x; // fx
        tinfo.P[2] = tinfo.P[2] * scale_x; // cx
        tinfo.P[3] = tinfo.P[3] * scale_x; // T
        tinfo.P[5] = tinfo.P[5] * scale_y; // fy
        tinfo.P[6] = tinfo.P[6] * scale_y; // cy

        if ( !use_messages_ || now - last_publish_time_  > period_ ) {
            cp_.publish(cv_img->toImageMsg(),
                        boost::make_shared<sensor_msgs::CameraInfo> (tinfo));

            out_times.push_front((now - last_publish_time_).toSec());
            out_bytes.push_front(cv_img->image.total()*cv_img->image.elemSize());

            last_publish_time_ = now;
        }
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
};
}

#include <pluginlib/class_list_macros.h>
typedef resized_image_transport::ImageResizer ImageResizer;
PLUGINLIB_DECLARE_CLASS (resized_image_transport, ImageResizer, ImageResizer, nodelet::Nodelet);
