#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <boost/thread/mutex.hpp>

class ImageResizer
{
public:
  ImageResizer () : nh(), pnh("~"), publish_once_(true) {

    pnh.param("resize_scale_x", resize_x_, 1.0);
    ROS_INFO("resize_scale_x : %f", resize_x_);
    pnh.param("resize_scale_y", resize_y_, 1.0);
    ROS_INFO("resize_scale_y : %f", resize_y_);

    pnh.param("width", dst_width_, 0);
    ROS_INFO("width : %d", dst_width_);
    pnh.param("height", dst_height_, 0);
    ROS_INFO("height : %d", dst_height_);

    pnh.param("max_queue_size", max_queue_size_, 5);

    pnh.param("use_snapshot", use_snapshot_, false);

    it_ = new image_transport::ImageTransport(pnh);
    std::string img = nh.resolveName("image_type");
    std::string cam = nh.resolveName("camera");
    if (img.at(0) == '/') {
      img.erase(0, 1);
    }
    ROS_INFO("camera = %s", cam.c_str());
    ROS_INFO("image = %s", img.c_str());

    if (use_snapshot_) {
      publish_once_ = false;
      srv_ = pnh.advertiseService("snapshot", &ImageResizer::snapshot_cb, this);
    }

    cs_ = it_->subscribeCamera(cam + "/" + img, max_queue_size_,
                               &ImageResizer::callback, this);

    cp_ = it_->advertiseCamera(img, max_queue_size_);
  }

  ~ImageResizer() { }

protected:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  image_transport::CameraSubscriber cs_;
  image_transport::CameraPublisher cp_;
  image_transport::ImageTransport *it_;
  ros::ServiceServer srv_;

  double resize_x_, resize_y_;
  int dst_width_, dst_height_;
  int max_queue_size_;
  bool use_snapshot_;
  bool publish_once_;
  boost::mutex mutex_;

  bool snapshot_cb (std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &res) {
    boost::mutex::scoped_lock lock(mutex_);

    publish_once_ = true;
    return true;
  }

  void callback(const sensor_msgs::ImageConstPtr &img,
                const sensor_msgs::CameraInfoConstPtr &info) {
    boost::mutex::scoped_lock lock(mutex_);

    ROS_DEBUG("resize: callback");
    if ( !publish_once_ || cp_.getNumSubscribers () == 0 ) {
      ROS_DEBUG("resize: number of subscribers is 0, ignoring image");
      return;
    }
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

    cp_.publish(cv_img->toImageMsg(),
                boost::make_shared<sensor_msgs::CameraInfo> (tinfo));
    if(use_snapshot_) {
      publish_once_ = false;
    }
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_resizer");

  ImageResizer ir;
  ros::spin();

  return 0;
}
