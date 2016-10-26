/**
 * Automatic perspective correction for quadrilateral objects. See the tutorial at
 * http://opencv-code.com/tutorials/automatic-perspective-correction-for-quadrilateral-objects/
 */

// https://raw.github.com/bsdnoobz/opencv-code/master/quad-segmentation.cpp

#include <ros/ros.h>
#include <jsk_topic_tools/log_utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <jsk_recognition_msgs/LineArray.h>
#include <jsk_perception/RectangleDetectorConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <dynamic_reconfigure/server.h>

namespace enc = sensor_msgs::image_encodings;

using namespace message_filters;

class RectangleDetector
{
    jsk_perception::RectangleDetectorConfig config_;
    dynamic_reconfigure::Server<jsk_perception::RectangleDetectorConfig> srv;

    //image_transport::Publisher image_pub_;
    ros::Publisher image_pub_;
    image_transport::ImageTransport it_;
    ros::NodeHandle nh_;
    //int subscriber_count_;

    message_filters::Subscriber<sensor_msgs::Image> *image_sub;
    message_filters::Subscriber<jsk_recognition_msgs::LineArray> *line_sub;
    TimeSynchronizer<sensor_msgs::Image, jsk_recognition_msgs::LineArray> *sync;

    double _threshold1;
    double _threshold2;
    int _apertureSize;
    bool _L2gradient;

    void reconfigureCallback(jsk_perception::RectangleDetectorConfig &new_config, uint32_t level)
    {
        config_ = new_config;
        _threshold1 = config_.threshold1;
        _threshold2 = config_.threshold2;
        _apertureSize = config_.apertureSize;
        _L2gradient = config_.L2gradient;
    }

    cv::Point2f computeIntersect(cv::Vec4i a,
                                 cv::Vec4i b)
    {
        int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];
        float denom;

        if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))
            {
                cv::Point2f pt;
                pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
                pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;
                return pt;
            }
        else
            return cv::Point2f(-1, -1);
    }

    void sortCorners(std::vector<cv::Point2f>& corners, 
                     cv::Point2f center)
    {
        std::vector<cv::Point2f> top, bot;

        for (int i = 0; i < corners.size(); i++)
            {
                if (corners[i].y < center.y)
                    top.push_back(corners[i]);
                else
                    bot.push_back(corners[i]);
            }

        cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
        cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
        cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
        cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

        corners.clear();
        corners.push_back(tl);
        corners.push_back(tr);
        corners.push_back(br);
        corners.push_back(bl);
    }

    void callback(const sensor_msgs::ImageConstPtr& image,
                  const jsk_recognition_msgs::LineArrayConstPtr& line)
    {
        // Transform the image.
        try
            {
                if ( line->lines.size() < 4 ) return;

                // Convert the image into something opencv can handle.
                cv::Mat src = cv_bridge::toCvShare(image, image->encoding)->image;
                std::vector<cv::Vec4i> lines;
                lines.resize(line->lines.size());

                // Expand the lines
                for (int i = 0; i < lines.size(); i++)
                    {
                        cv::Vec4i v;
                        v[0] = line->lines[i].x1; v[1] = line->lines[i].y1;
                        v[2] = line->lines[i].x2; v[3] = line->lines[i].y2;
                        lines[i][0] = 0;
                        lines[i][1] = ((float)v[1] - v[3]) / (v[0] - v[2]) * -v[0] + v[1];
                        lines[i][2] = src.cols;
                        lines[i][3] = ((float)v[1] - v[3]) / (v[0] - v[2]) * (src.cols - v[2]) + v[3];
                    }

                std::vector<cv::Point2f> corners;
                for (int i = 0; i < lines.size(); i++)
                    {
                        for (int j = i+1; j < lines.size(); j++)
                            {
                                cv::Point2f pt = computeIntersect(lines[i], lines[j]);
                                if (pt.x >= 0 && pt.y >= 0)
                                    corners.push_back(pt);
                            }
                    }

                std::vector<cv::Point2f> approx;
                cv::approxPolyDP(cv::Mat(corners), approx, cv::arcLength(cv::Mat(corners), true) * 0.02, true);

                if (approx.size() != 4)
                    {
                        ROS_ERROR("The object is not quadrilateral!");
                        return ;
                    }

                // Get mass center
                cv::Point2f center(0,0);
                for (int i = 0; i < corners.size(); i++)
                    center += corners[i];
                center *= (1. / corners.size());

                sortCorners(corners, center);

                cv::Mat dst = src.clone();

                // Draw lines
                for (int i = 0; i < lines.size(); i++)
                    {
                        cv::Vec4i v = lines[i];
                        cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0,255,0));
                    }

                // Draw corner points
                cv::circle(dst, corners[0], 3, CV_RGB(255,0,0), 2);
                cv::circle(dst, corners[1], 3, CV_RGB(0,255,0), 2);
                cv::circle(dst, corners[2], 3, CV_RGB(0,0,255), 2);
                cv::circle(dst, corners[3], 3, CV_RGB(255,255,255), 2);

                // Draw mass center
                cv::circle(dst, center, 3, CV_RGB(255,255,0), 2);

                cv::Mat quad = cv::Mat::zeros(300, 220, CV_8UC3);

                std::vector<cv::Point2f> quad_pts;
                quad_pts.push_back(cv::Point2f(0, 0));
                quad_pts.push_back(cv::Point2f(quad.cols, 0));
                quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));
                quad_pts.push_back(cv::Point2f(0, quad.rows));

                cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);
                cv::warpPerspective(src, quad, transmtx, quad.size());

                sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(image->header, enc::RGB8, dst).toImageMsg();
                //sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(image->header, enc::RGB8, quad).toImageMsg();
                image_pub_.publish(out_img);
            }
        catch (cv::Exception &e)
            {
                ROS_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
            }
    }

public:
    RectangleDetector(ros::NodeHandle nh = ros::NodeHandle()) : it_(nh), nh_(nh)
    {

        image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "image", 1);
        line_sub = new message_filters::Subscriber<jsk_recognition_msgs::LineArray>(nh_, "lines", 1);
        sync = new TimeSynchronizer<sensor_msgs::Image, jsk_recognition_msgs::LineArray>(*image_sub, *line_sub, 10);
        sync->registerCallback(boost::bind(&RectangleDetector::callback, this, _1, _2));
        image_pub_ = nh_.advertise<sensor_msgs::Image>("rectangle/image", 1);

        dynamic_reconfigure::Server<jsk_perception::RectangleDetectorConfig>::CallbackType f =
            boost::bind(&RectangleDetector::reconfigureCallback, this, _1, _2);
        srv.setCallback(f);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "edge_detector", ros::init_options::AnonymousName);

    RectangleDetector rd;
    ros::spin();
}
