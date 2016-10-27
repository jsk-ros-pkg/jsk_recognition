// @brief sample node to use oriented gradient
// @author Hiroaki Yaguchi, JSK

#include <vector>
#include <string>

#include <ros/ros.h>
#include <jsk_topic_tools/log_utils.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "jsk_perception/oriented_gradient.hpp"

namespace jsk_perception {

class OrientedGradientNode {
 public:
  explicit OrientedGradientNode(const ros::NodeHandle& nh) :
      handle_(nh), image_transport_(nh) {
    image_pub_ = image_transport_.advertise("image", 1);
    image_sub_ = image_transport_.subscribe(
        "/camera/rgb/image_raw", 1,
        &OrientedGradientNode::imageCallback, this);
    cv::namedWindow("OrinetedGradient");
  }

  ~OrientedGradientNode() {
  }

 private:
  ros::NodeHandle handle_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat cv_img = cv_ptr->image;
    cv::Mat cv_og_img;
    cv::Mat cv_out_img;

    // calcOrientedGradient() will write oriented gradient to
    // 2nd arg image as HSV format,
    // H is orientation and V is intensity
    calcOrientedGradient(cv_img, cv_og_img);
    cv::cvtColor(cv_og_img, cv_out_img, CV_HSV2BGR);
    // cv::imshow("OrinetedGradient", cv_out_img);
    // cv::waitKey(1);
    sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(
      msg->header,
      sensor_msgs::image_encodings::BGR8,
      cv_out_img).toImageMsg();
    image_pub_.publish(out_img);
  }
};

}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "oriented_gradient");
  ros::NodeHandle handle("~");
  jsk_perception::OrientedGradientNode ognode(handle);
  ros::spin();
  return 0;
}


