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
class SparseImageEncoder: public nodelet::Nodelet
{
  ros::Publisher _spr_img_pub;
  image_transport::Subscriber _img_sub;

  jsk_recognition_msgs::SparseImagePtr _spr_img_ptr;

  boost::shared_ptr<image_transport::ImageTransport> _it;
  ros::NodeHandle _nh;
  ros::NodeHandle _ln;
  int _subscriber_count;
  double _rate;
  bool _print_point_num;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    do_work(msg, msg->header.frame_id);
  }

  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg){
    try {
      int channel = enc::numChannels(msg->encoding);

      // check if uint16 array needed
      bool useData32 = false;
      if (std::max(msg->width, msg->height) > 256){
        useData32 = true;
        NODELET_DEBUG("use data32 array");
      }

      _spr_img_ptr->header.stamp = msg->header.stamp;
      _spr_img_ptr->header.frame_id = input_frame_from_msg;
      _spr_img_ptr->width  = msg->width;
      _spr_img_ptr->height = msg->height;
      if (!_spr_img_ptr->data16.empty()) _spr_img_ptr->data16.clear();
      if (!_spr_img_ptr->data32.empty()) _spr_img_ptr->data32.clear();

      // make sparse image
      for (uint32_t y = 0; y < msg->height; ++y){
        for (uint32_t x = 0; x < msg->width; ++x){
          if(msg->data[x*channel+y*msg->step] > 125){
            if (useData32) _spr_img_ptr->data32.push_back( (x << 16) | y );
            else           _spr_img_ptr->data16.push_back( (x << 8)  | y );
          }
        }
      }

      // print number of point if enabled
      if (_print_point_num) {
        int size = 0;
        if (useData32) size = _spr_img_ptr->data32.size();
        else size = _spr_img_ptr->data16.size();
        NODELET_INFO("%d point encoded. encoded area per is %lf%%.", size, (100 * ((double) size)/(msg->height* msg->width)));
      }

      // publish sparse image message
      _spr_img_pub.publish(*_spr_img_ptr);
    } // end of try
    catch (...) {
      NODELET_ERROR("making sparse image error");
    }

    ros::Rate pubRate(_rate); // hz
    pubRate.sleep();
  } // end of do_work function

  void subscribe() {
    NODELET_DEBUG("Subscribing to image topic.");
    _img_sub = _it->subscribe("image", 3, &SparseImageEncoder::imageCallback, this);
    ros::V_string names = boost::assign::list_of("image");
    jsk_topic_tools::warnNoRemap(names);
  }

  void unsubscribe() {
    NODELET_DEBUG("Unsubscribing from image topic.");
    _img_sub.shutdown();
  }

  void connectCb(const ros::SingleSubscriberPublisher& ssp) {
    if (_subscriber_count++ == 0) {
      subscribe();
    }
  }

  void disconnectCb(const ros::SingleSubscriberPublisher&) {
    _subscriber_count--;
    if (_subscriber_count == 0) {
      unsubscribe();
    }
  }

public:
  void onInit() {
    _nh = getNodeHandle();
    _ln = ros::NodeHandle("~");
    _it.reset(new image_transport::ImageTransport(_nh));
    _subscriber_count = 0;
    ros::SubscriberStatusCallback connect_cb    = boost::bind(&SparseImageEncoder::connectCb, this, _1);
    ros::SubscriberStatusCallback disconnect_cb = boost::bind(&SparseImageEncoder::disconnectCb, this, _1);
    _spr_img_pub = _nh.advertise<jsk_recognition_msgs::SparseImage>("sparse_image", 10, connect_cb, disconnect_cb);
    _spr_img_ptr = boost::make_shared<jsk_recognition_msgs::SparseImage>();
    _ln.param("rate", _rate, 3.0);
    _ln.param("print_point_num", _print_point_num, false);
  } // end of onInit function
}; // end of SparseImageEncoder class definition
} // end of jsk_perception namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::SparseImageEncoder, nodelet::Nodelet);
