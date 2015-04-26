// -*- mode: C++ -*-

#ifndef IMAGE_PROCESSING_NODELET_H_
#define IMAGE_PROCESSING_NODELET_H_

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

namespace resized_image_transport
{
  class ImageProcessing : public nodelet::Nodelet
  {
  public:
  protected:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    
    //publishser and subscriber
    image_transport::CameraSubscriber cs_;
    image_transport::CameraPublisher cp_;
    image_transport::ImageTransport *it_;
    ros::ServiceServer srv_;
    ros::Subscriber sub_;

    ros::Publisher image_pub_;
    ros::Subscriber image_sub_;
    
    ros::Publisher width_scale_pub_;
    ros::Publisher height_scale_pub_;

    double resize_x_, resize_y_;
    int dst_width_, dst_height_;
    int max_queue_size_;
    bool use_snapshot_;
    bool publish_once_;
    bool use_messages_;
    bool use_bytes_;
    bool use_camera_info_;
    bool verbose_;
    ros::Time last_rosinfo_time_, last_subscribe_time_, last_publish_time_;
    ros::Duration period_;
    boost::mutex mutex_;

    boost::circular_buffer<double> in_times;
    boost::circular_buffer<double> out_times;
    boost::circular_buffer<double> in_bytes;
    boost::circular_buffer<double> out_bytes;

    void onInit();
    void initNodeHandle();
    void initReconfigure();
    void initParams();
    void initPublishersAndSubscribers();

  public:
    ImageProcessing():
      in_times(boost::circular_buffer<double>(100)),
      out_times(boost::circular_buffer<double>(100)),
      in_bytes(boost::circular_buffer<double>(100)),
      out_bytes(boost::circular_buffer<double>(100))
	{ }
      ~ImageProcessing() { }
    
  protected:
    virtual void process(const sensor_msgs::ImageConstPtr &src_img, const sensor_msgs::CameraInfoConstPtr &src_info,
			 sensor_msgs::ImagePtr &dst_img, sensor_msgs::CameraInfo &dst_info) = 0;

    void snapshot_msg_cb (const std_msgs::EmptyConstPtr msg);

    bool snapshot_srv_cb (std_srvs::Empty::Request &req,
			  std_srvs::Empty::Response &res);
		    
    void image_cb(const sensor_msgs::ImageConstPtr &img);


    void callback(const sensor_msgs::ImageConstPtr &img,
		  const sensor_msgs::CameraInfoConstPtr &info);
  };
}

#endif
