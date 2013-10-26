#include <ros/ros.h>
#include <jsk_perception/HoughLinesConfig.h>
#include <jsk_perception/LineArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <dynamic_reconfigure/server.h>

namespace enc = sensor_msgs::image_encodings;

class HoughLines
{
    jsk_perception::HoughLinesConfig config_;
    dynamic_reconfigure::Server<jsk_perception::HoughLinesConfig> srv;

    image_transport::Publisher img_pub_;
    image_transport::Subscriber img_sub_;
    ros::Publisher out_pub_;

    image_transport::ImageTransport it_;
    ros::NodeHandle nh_;
    int subscriber_count_;

    double _rho;
    double _theta;
    int    _threshold;
    double _minLineLength;
    double _maxLineGap;

    void reconfigureCallback(jsk_perception::HoughLinesConfig &new_config, uint32_t level)
    {
        config_ = new_config;
        _rho            = config_.rho;
        _theta          = config_.theta;
        _threshold       = config_.threshold;
        _minLineLength  = config_.minLineLength;
        _maxLineGap     = config_.maxLineGap;
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

                if (msg->encoding != enc::MONO8) {
                    ROS_ERROR("Hough Lines assume monochrome image for input");
                    return;
                }

                cv::Mat out_image;
                cvtColor(in_image, out_image, CV_GRAY2BGR);

                // Do the work
                ROS_INFO_STREAM("Hough Lines : rho:" << _rho << ", tehta:" << _theta << ", threshold:" <<  _threshold << ", minLineLength:" <<  _minLineLength << ", maxLineGap" <<  _maxLineGap);
#if 1
                std::vector<cv::Vec4i> lines;
                cv::HoughLinesP( in_image, lines, _rho, _theta*CV_PI/180 , _threshold, _minLineLength, _maxLineGap );
                jsk_perception::LineArray out_lines;
                out_lines.header = msg->header;
                out_lines.lines.resize(lines.size());
                for( size_t i = 0; i < lines.size(); i++ )
                    {
                        out_lines.lines[i].x1 = lines[i][0];
                        out_lines.lines[i].y1 = lines[i][1];
                        out_lines.lines[i].x2 = lines[i][2];
                        out_lines.lines[i].y2 = lines[i][3];
                        cv::line( out_image, cv::Point(lines[i][0], lines[i][1]),
                                  cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255,0,0), 3, 8 );
                    }
#else
                std::vector<cv::Vec2f> lines;
                cv::HoughLines( in_image, lines, _rho, _theta, _threshold*CV_PI/180, _minLineLength, _maxLineGap );
                jsk_perception::LineArray out_lines;
                out_lines.header = msg->header;
                out_lines.lines.resize(lines.size());
                for( size_t i = 0; i < lines.size(); i++ )
                    {
                        float rho = lines[i][0];
                        float theta = lines[i][1];
                        double a = cos(theta), b = sin(theta);
                        double x0 = a*rho, y0 = b*rho;
                        cv::Point pt1(cvRound(x0 + 1000*(-b)),
                                      cvRound(y0 + 1000*(a)));
                        cv::Point pt2(cvRound(x0 - 1000*(-b)),
                                      cvRound(y0 - 1000*(a)));
                        cv::line( out_image, pt1, pt2, cv::Scalar(0,0,255), 3, 8 );
                    }
#endif

                // Publish the image.
                sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, enc::RGB8, out_image).toImageMsg();
                out_pub_.publish(out_lines);
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
        img_sub_ = it_.subscribe("image", 3, &HoughLines::imageCallback, this);
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
    HoughLines(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), nh_(nh), subscriber_count_(0)
    {
        image_transport::SubscriberStatusCallback connect_cb    = boost::bind(&HoughLines::connectCb, this, _1);
        image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&HoughLines::disconnectCb, this, _1);
        img_pub_ = image_transport::ImageTransport(ros::NodeHandle(nh_, "hough")).advertise("image", 1, connect_cb, disconnect_cb);

        out_pub_ = nh.advertise<jsk_perception::LineArray>(nh_.resolveName("lines"), 1);


        dynamic_reconfigure::Server<jsk_perception::HoughLinesConfig>::CallbackType f =
            boost::bind(&HoughLines::reconfigureCallback, this, _1, _2);
        srv.setCallback(f);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hough_lines", ros::init_options::AnonymousName);

    HoughLines hl;
    ros::spin();
}
