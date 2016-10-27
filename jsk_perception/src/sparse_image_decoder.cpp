#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <vector>
#include <iostream>

#include <nodelet/nodelet.h>
#include <jsk_topic_tools/log_utils.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/SparseImage.h>


namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception {
class SparseImageDecoder: public nodelet::Nodelet
{
  image_transport::Publisher _img_pub;
  ros::Subscriber _spr_img_sub;

  sensor_msgs::ImagePtr _img_ptr;

  boost::shared_ptr<image_transport::ImageTransport> _it;
  ros::NodeHandle _nh;
  int _subscriber_count;

  void imageCallback(const jsk_recognition_msgs::SparseImageConstPtr& msg){
    do_work(msg, msg->header.frame_id);
  }

  void do_work(const jsk_recognition_msgs::SparseImageConstPtr& msg, const std::string input_frame_from_msg){
    try {

      _img_ptr->header.stamp = msg->header.stamp;
      _img_ptr->header.frame_id = input_frame_from_msg;
      _img_ptr->width  = msg->width;
      _img_ptr->height = msg->height;
      _img_ptr->step = msg->width;
      _img_ptr->encoding = enc::MONO8;
      if (!_img_ptr->data.empty()) _img_ptr->data.clear();

      // check if uint16 array needed
      bool useData32 = false;
      int length = msg->data16.size();
      if (length <= 0) {
        useData32 = true;
        length = msg->data32.size();
        NODELET_DEBUG("use data32 array");
      }
      _img_ptr->data.resize(_img_ptr->width * _img_ptr->height);
      // decode sparse image -> image
      for (int i = 0; i < length; ++i){
        uint16_t x, y;
        if (useData32) {
          uint32_t pos = msg->data32[i];
          x = (uint16_t)(pos >> 16);
          y = (uint16_t)pos;
        } else {
          uint16_t pos = msg->data16[i];
          x = (uint16_t)(pos >> 8);
          y = (uint16_t)(pos & (uint8_t)-1);
        }
        _img_ptr->data[y * _img_ptr->width + x] = 255;
      }

      // publish image message
      _img_pub.publish(*_img_ptr);
    } // end of try
    catch (...) {
      NODELET_ERROR("making sparse image error");
    }
  } // end of do_work function

  void subscribe() {
    NODELET_DEBUG("Subscribing to image topic.");
    _spr_img_sub = _nh.subscribe("sparse_image", 3, &SparseImageDecoder::imageCallback, this);
    ros::V_string names = boost::assign::list_of("sparse_image");
    jsk_topic_tools::warnNoRemap(names);
  }

  void unsubscribe() {
    NODELET_DEBUG("Unsubscribing from image topic.");
    _spr_img_sub.shutdown();
  }

  void connectCb(const image_transport::SingleSubscriberPublisher& ssp) {
    if (_subscriber_count++ == 0) {
      subscribe();
    }
  }

  void disconnectCb(const image_transport::SingleSubscriberPublisher&) {
    _subscriber_count--;
    if (_subscriber_count == 0) {
      unsubscribe();
    }
  }

public:
  void onInit() {
    _nh = getNodeHandle();
    _img_ptr.reset(new sensor_msgs::Image());
    _it.reset(new image_transport::ImageTransport(_nh));
    _subscriber_count = 0;
    image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&SparseImageDecoder::connectCb, this, _1);
    image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&SparseImageDecoder::disconnectCb, this, _1);
    _img_pub = image_transport::ImageTransport(ros::NodeHandle(_nh, "sparse")).advertise("image_decoded", 1, connect_cb, disconnect_cb);
  } // end of onInit function
}; // end of SparseImageDecoder class definition
} // end of jsk_perception namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::SparseImageDecoder, nodelet::Nodelet);
