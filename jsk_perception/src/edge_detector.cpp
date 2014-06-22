#include <nodelet/nodelet.h>
#include <jsk_perception/EdgeDetectorConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <dynamic_reconfigure/server.h>

#include <pluginlib/class_list_macros.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception {
class EdgeDetector: public nodelet::Nodelet
{
    jsk_perception::EdgeDetectorConfig config_;
    dynamic_reconfigure::Server<jsk_perception::EdgeDetectorConfig> srv;

    image_transport::Publisher img_pub_;
    image_transport::Subscriber img_sub_;

    boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::NodeHandle nh_;
    int subscriber_count_;

    double _threshold1;
    double _threshold2;
    int _apertureSize;
    bool _L2gradient;

    void reconfigureCallback(jsk_perception::EdgeDetectorConfig &new_config, uint32_t level)
    {
        config_ = new_config;
        _threshold1 = config_.threshold1;
        _threshold2 = config_.threshold2;
        _apertureSize = config_.apertureSize;
        _L2gradient = config_.L2gradient;
        if (subscriber_count_)
            { // @todo Could do this without an interruption at some point.
                unsubscribe();
                subscribe();
            }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        do_work(msg, msg->header.frame_id);
    }

    void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
    {
        // Transform the image.
        try
            {
                // Convert the image into something opencv can handle.
                cv::Mat in_image = cv_bridge::toCvShare(msg, msg->encoding)->image;

                // Do the work
                cv::Mat out_image;
		if(in_image.channels() >= 3){
		  cv::cvtColor(in_image, out_image, CV_BGR2GRAY);
		}else{
		  out_image = in_image;
		}
                cv::blur(out_image, out_image, cv::Size(_apertureSize,_apertureSize));
                cv::Canny(out_image, out_image, _threshold1, _threshold2, _apertureSize, _L2gradient);

                // Publish the image.
                sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, enc::MONO8, out_image).toImageMsg();
                img_pub_.publish(out_img);
            }
        catch (cv::Exception &e)
            {
                NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
            }
    }

    void subscribe()
    {
        NODELET_DEBUG("Subscribing to image topic.");
        img_sub_ = it_->subscribe("image", 3, &EdgeDetector::imageCallback, this);
    }

    void unsubscribe()
    {
        NODELET_DEBUG("Unsubscribing from image topic.");
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
    void onInit() {
      nh_ = getNodeHandle();
      it_.reset(new image_transport::ImageTransport(nh_));
      subscriber_count_ = 0;
      image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&EdgeDetector::connectCb, this, _1);
      image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&EdgeDetector::disconnectCb, this, _1);
      img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "edge")).advertise("image", 1, connect_cb, disconnect_cb);

      dynamic_reconfigure::Server<jsk_perception::EdgeDetectorConfig>::CallbackType f =
        boost::bind(&EdgeDetector::reconfigureCallback, this, _1, _2);
      srv.setCallback(f);
    }
  
};
}

typedef jsk_perception::EdgeDetector EdgeDetector;
PLUGINLIB_DECLARE_CLASS (jsk_perception, EdgeDetector, EdgeDetector, nodelet::Nodelet);
