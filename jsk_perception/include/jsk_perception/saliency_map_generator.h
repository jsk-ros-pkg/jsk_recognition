
#ifndef _JSK_PERCEPTION_SALIENCY_MAP_GENERATOR_H_
#define _JSK_PERCEPTION_SALIENCY_MAP_GENERATOR_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/mutex.hpp>

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <omp.h>
#include <time.h>
#include <sys/time.h>

namespace jsk_perception
{
   class SaliencyMapGenerator: public jsk_topic_tools::DiagnosticNodelet
   {
    private:
      void calcIntensityChannel(cv::Mat, cv::Mat);
      void copyImage(cv::Mat, cv::Mat);
      void getIntensityScaled(cv::Mat, cv::Mat, cv::Mat, cv::Mat, int);
      float getMean(cv::Mat, cv::Point2i, int, int);
      void mixScales(cv::Mat *, cv::Mat, cv::Mat *, cv::Mat, const int);
      void mixOnOff(cv::Mat intensityOn, cv::Mat intensityOff, cv::Mat intensity);
      void getIntensity(cv::Mat, cv::Mat, cv::Mat, cv::Mat, bool);
      
      int num_threads_;
      bool print_fps_;
      double start_;
      int counter_;
      
    protected:
      boost::mutex lock_;
      ros::Subscriber sub_image_;
      ros::Publisher pub_image_;

      void onInit();
      void subscribe();
      void unsubscribe();

    public:
      SaliencyMapGenerator(): DiagnosticNodelet("SaliencyMapGenerator"),
                              counter_(0), num_threads_(2) {}
      bool computeSaliencyImpl(cv::Mat, cv::Mat &);
      void setNumThreads(int);
      void callback(const sensor_msgs::Image::ConstPtr &);
   };
}

#endif	 // _JSK_PERCEPTION_SALIENCY_MAP_GENERATOR_H_


