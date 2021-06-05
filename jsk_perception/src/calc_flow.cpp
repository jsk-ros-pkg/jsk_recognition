//
// JSK calc_flow
//
#include <ros/ros.h>
#include <ros/names.h>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/opencv.hpp>
#else
#include <cv.hpp>
#endif

#include <stdlib.h>

class calc_flow_node {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  ros::Publisher result_pub_;
  std::string namespace_;
  cv::Mat prevImg;
  cv::Mat flow;
  //cv::Mat *nextImg;
public:
  calc_flow_node () : nh_(), it_(nh_), flow(0, 0, CV_8UC1), prevImg(0, 0, CV_8UC1) {
    //flow = new cv::Mat(0, 0, CV_8UC1);

    namespace_ = nh_.resolveName("camera");
    result_pub_ = nh_.advertise<sensor_msgs::Image> ("flow_image", 1);
    sub_ = it_.subscribe(namespace_ + "/image", 10, &calc_flow_node::imageCB, this);
  }

  void imageCB(const sensor_msgs::ImageConstPtr &img) {
    //IplImage *ipl_ = new IplImage();
    //ipl_ = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 1);
    //sensor_msgs::CvBridge bridge;
    cv_bridge::CvImagePtr cv_ptr;
    //cvResize(bridge.imgMsgToCv(img, "mono8"), ipl_);
    //IplImage *ipl_ = bridge.imgMsgToCv(img, "mono8");
    //cv_ptr = cv_bridge::toCvCopy(img, "mono8");
    cv_ptr = cv_bridge::toCvCopy(img, "mono8");
    bool prevImg_update_required = false;
    if((flow.cols != (int)img->width) ||
       (flow.rows != (int)img->height)) {
      ROS_INFO("make flow");
      cv_ptr->image.copyTo(flow);
      prevImg_update_required = true;
    }
    if(prevImg_update_required) {
      cv_ptr->image.copyTo(prevImg);
      prevImg_update_required = false;
      ROS_INFO("return");
      return;
    }
    //
    //ROS_INFO("subscribe image");
    //prevImg.
    //cv::Mat *nextImg = new cv::Mat(ipl_);
    cv::Mat nextImg(img->height, img->width, CV_8UC1);
    //memcpy(nextImg->data, ipl_->imageData, img->height*img->width);
    cv_ptr->image.copyTo(nextImg);

    cv::calcOpticalFlowFarneback(prevImg, nextImg, flow,
                                 0.5, 3, 15, 3, 5, 1.2, 0 );
                                 // 0.5, 2,
                                 // 16, 4,
                                 // 5, 1.1, 0);
    //cv::OPTFLOW_USE_INITIAL_FLOW);
    nextImg.copyTo(prevImg);

    sensor_msgs::Image result;
    result.header = img->header;
    result.width  = flow.cols;
    result.height = flow.rows;
    result.encoding = "mono8";
    result.step   = flow.cols;
    result.data.resize(flow.cols * flow.rows);
    CvPoint2D32f *ptr = (CvPoint2D32f *)flow.data;
    for(int i = 0; i<result.data.size(); i++) {
      // copy flow -> result
      //result.data[i] = ;
      int val = 10 * sqrt(ptr[i].x * ptr[i].x + ptr[i].y * ptr[i].y);
      result.data[i] = 255>val?val:255;
    }
    result_pub_.publish(result);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calc_flow");
  //cv::namedWindow(std::string("window"), );
  calc_flow_node flow_node;

  ros::spin();
  return 0;
}
