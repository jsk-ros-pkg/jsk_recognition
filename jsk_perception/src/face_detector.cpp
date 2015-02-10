/// @file face_detector.cpp
/// @author Hiroaki Yaguchi
/// @brief face detector
///
/// @par Publish:
/// - face_detector/image (sensor_msgs::Image)
/// - faces (jsk_recognition_msgs::RectArray)
/// @par Subscribe:
/// - image (sensor_msgs::Image)
///

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "jsk_recognition_msgs/Rect.h"
#include "jsk_recognition_msgs/RectArray.h"

namespace enc = sensor_msgs::image_encodings;

class FaceDetector
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  ros::Publisher rect_pub_;

  image_transport::ImageTransport it_;
  ros::NodeHandle nh_;
  int subscriber_count_;

  cv::CascadeClassifier cascade_;
  double scale_factor_;
  int min_neighbors_;
  jsk_recognition_msgs::RectArray rect_array_msg_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  void do_work(const sensor_msgs::ImageConstPtr& msg,
               const std::string input_frame_from_msg)
  {
    // Transform the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

      // Do the work
      cv::Mat in_gimage;
      cv::Mat out_image;

      if(in_image.channels() >= 3) {
        cv::cvtColor(in_image, in_gimage, CV_BGR2GRAY);
        out_image = in_image;
      } else {
        in_gimage = in_image;
        cv::cvtColor(in_gimage, out_image, CV_GRAY2BGR);
      }

      // Face detection
      std::vector<cv::Rect> faces;
      cascade_.detectMultiScale(in_gimage, faces,
                                scale_factor_,
                                min_neighbors_,
                                CV_HAAR_SCALE_IMAGE);

      // Convert result from CV to ROS
      rect_array_msg_.rectangles.clear();
      for(int i = 0; i < faces.size(); i++) {
        const cv::Rect& face = faces[i];
        cv::rectangle(out_image,
                      face.tl(), face.br(),
                      cv::Scalar(80, 80, 255), 2, CV_AA);
        jsk_recognition_msgs::Rect rect;
        rect.x = face.x;
        rect.y = face.y;
        rect.width = face.width;
        rect.height = face.height;
        rect_array_msg_.rectangles.push_back(rect);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img =
          cv_bridge::CvImage(msg->header, enc::RGB8, out_image).toImageMsg();
      img_pub_.publish(out_img);

      rect_array_msg_.header = msg->header;
      rect_pub_.publish(rect_array_msg_);
    }
    catch (cv::Exception &e)
    {
      ROS_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  }

  void subscribe()
  {
    ROS_DEBUG("Subscribing to image topic.");
    img_sub_ = it_.subscribe("image", 3, &FaceDetector::imageCallback, this);
  }

  void unsubscribe()
  {
    ROS_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      subscribe();
    }
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      unsubscribe();
    }
  }

 public:
  FaceDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), nh_(nh), subscriber_count_(0)
  {
    std::string cascade_file =
        "/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml";
    if (!cascade_.load(cascade_file)) {
      ROS_ERROR("CANNOT LOAD FILE %s", cascade_file.c_str());
      return;
    }

    // params for face detection
    scale_factor_ = 1.1;
    min_neighbors_ = 3;

    image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&FaceDetector::connectCb, this, _1);
    image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&FaceDetector::disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "face_detector")).advertise("image", 1, connect_cb, disconnect_cb);
    rect_pub_ = nh.advertise<jsk_recognition_msgs::RectArray>(nh_.resolveName("faces"), 1);

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_detection", ros::init_options::AnonymousName);

  FaceDetector face_detection;
  ros::spin();
}

