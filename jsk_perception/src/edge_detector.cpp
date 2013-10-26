#include <ros/ros.h>
#include <jsk_perception/EdgeDetectorConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <dynamic_reconfigure/server.h>

namespace enc = sensor_msgs::image_encodings;

class EdgeDetector
{
    jsk_perception::EdgeDetectorConfig config_;
    dynamic_reconfigure::Server<jsk_perception::EdgeDetectorConfig> srv;

    image_transport::Publisher img_pub_;
    image_transport::Subscriber img_sub_;

    image_transport::ImageTransport it_;
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
                cv::cvtColor(in_image, out_image, CV_BGR2GRAY);
                cv::blur(out_image, out_image, cv::Size(_apertureSize,_apertureSize));
                cv::Canny(out_image, out_image, _threshold1, _threshold2, _apertureSize, _L2gradient);

                // Publish the image.
                sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, enc::MONO8, out_image).toImageMsg();
                img_pub_.publish(out_img);
            }
        catch (cv::Exception &e)
            {
                ROS_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
            }
    }

    void subscribe()
    {
        ROS_DEBUG("Subscribing to image topic.");
        img_sub_ = it_.subscribe("image", 3, &EdgeDetector::imageCallback, this);
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
    EdgeDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), nh_(nh), subscriber_count_(0)
    {
        image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&EdgeDetector::connectCb, this, _1);
        image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&EdgeDetector::disconnectCb, this, _1);
        img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "edge")).advertise("image", 1, connect_cb, disconnect_cb);

        dynamic_reconfigure::Server<jsk_perception::EdgeDetectorConfig>::CallbackType f =
            boost::bind(&EdgeDetector::reconfigureCallback, this, _1, _2);
        srv.setCallback(f);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edge_detector", ros::init_options::AnonymousName);

    EdgeDetector ed;
    ros::spin();
}
