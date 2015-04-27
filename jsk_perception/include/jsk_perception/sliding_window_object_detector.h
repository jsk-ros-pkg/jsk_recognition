
#ifndef JSK_PERCEPTION_SLIDING_WINDOW_OBJECT_DETECTOR_H
#define JSK_PERCEPTION_SLIDING_WINDOW_OBJECT_DETECTOR_H

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_perception/histogram_of_oriented_gradients.h>
#include <jsk_recognition_msgs/RectArray.h>

#include <jsk_recognition_msgs/ClusterPointIndices.h>
// #include <jsk_pcl_ros/pcl_conversion_util.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_perception/SlidingWindowObjectDetectorConfig.h>

#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <algorithm>
#include <utility>

namespace jsk_perception
{
   class SlidingWindowObjectDetector: public jsk_topic_tools::DiagnosticNodelet,
                                      public HOGFeatureDescriptor
   {
    public:
      SlidingWindowObjectDetector(): DiagnosticNodelet("SlidingWindowObjectDetector")
      {
         // loadTrainedDetectorModel();
      }

      virtual void loadTrainedDetectorModel();
      virtual void trainObjectClassifier();
      virtual void readDataset(
         std::string, std::vector<cv::Mat> &,
         cv::Mat &, bool = false, const int = 0);
      virtual void extractFeatures(
         const std::vector<cv::Mat> &, cv::Mat &);
      virtual void trainBinaryClassSVM(
         const cv::Mat &, const cv::Mat &);
      virtual std::vector<cv::Rect_<int> > runObjectRecognizer(
         cv::Mat &, const cv::Size, const float, const int, const int);
      virtual void objectRecognizer(
         const cv::Mat &, std::multimap<float, cv::Rect_<int> > &,
         const cv::Size, const int = 16);
      virtual void pyramidialScaling(
         cv::Size &, const float);
      virtual std::vector<cv::Rect_<int> > nonMaximumSuppression(
         std::multimap<float, cv::Rect_<int> > &, const float);

      void convertCvRectToJSKRectArray(
         const std::vector<cv::Rect_<int> > &,
         jsk_recognition_msgs::RectArray &, const int, const cv::Size );
      void concatenateCVMat(
         const cv::Mat &, const cv::Mat &, cv::Mat &, bool = true);
      virtual void configCallback(
         jsk_perception::SlidingWindowObjectDetectorConfig &, uint32_t);
      
    protected:
      virtual void imageCb(
       const sensor_msgs::ImageConstPtr&);

      virtual void onInit();
      virtual void subscribe();
      virtual void unsubscribe();

      boost::mutex mutex_;
      ros::Subscriber sub_;
      ros::Publisher pub_image_;
      ros::Publisher pub_rects_;
      ros::ServiceClient nms_client_;

      int swindow_x;
      int swindow_y;
      float scale_;
      int stack_size_;
      int incrementor_;
      int downsize_;
      
      std::string model_name_;
      std::string dataset_path_;
      boost::shared_ptr<cv::SVM> supportVectorMachine_;
      boost::shared_ptr<dynamic_reconfigure::Server<
         jsk_perception::SlidingWindowObjectDetectorConfig> > srv_;

   };

} // namespace jsk_perception


#endif  // JSK_PERCEPTION_SLIDING_WINDOW_OBJECT_DETECTOR_H
