// @author Krishneel Chaudhary, JSK

#ifndef JSK_PERCEPTION_SLIDING_WINDOW_OBJECT_DETECTOR_H
#define JSK_PERCEPTION_SLIDING_WINDOW_OBJECT_DETECTOR_H

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_perception/histogram_of_oriented_gradients.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_perception/SlidingWindowObjectDetectorConfig.h>

#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

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
      SlidingWindowObjectDetector():
         DiagnosticNodelet("SlidingWindowObjectDetector") {}
      
      virtual void readTrainingManifestFromDirectory();
      virtual void loadTrainedDetectorModel();
      virtual std::multimap<float, cv::Rect_<int> > runSlidingWindowDetector(
         const cv::Mat &, const cv::Size, const float, const int, const int);
      virtual void objectRecognizer(
         const cv::Mat &, std::multimap<float, cv::Rect_<int> > &,
         const cv::Size, const int = 16);
      virtual void pyramidialScaling(
         cv::Size &, const float);
      virtual std::vector<cv::Rect_<int> > nonMaximumSuppression(
         std::multimap<float, cv::Rect_<int> > &, const float);
      void convertCvRectToJSKRectArray(
         const std::vector<cv::Rect_<int> > &,
         jsk_recognition_msgs::RectArray &, const int, const cv::Size);
      virtual void configCallback(
         jsk_perception::SlidingWindowObjectDetectorConfig &, uint32_t);
      virtual void concatenateCVMat(
       const cv::Mat &, const cv::Mat &, cv::Mat &, bool = true);
      virtual void setBoundingBoxLabel(
         cv::Mat&, cv::Rect_<int>, const std::string = "object");
      
      // temp placed here
      virtual void computeHSHistogram(
         cv::Mat &, cv::Mat &, const int = 64, const int = 32, bool = true);
      
    protected:
      virtual void imageCb(
       const sensor_msgs::ImageConstPtr&);

      virtual void onInit();
      virtual void subscribe();
      virtual void unsubscribe();

      boost::mutex mutex_;
      ros::Subscriber sub_image_;
      ros::Publisher pub_image_;
      ros::Publisher pub_rects_;
      ros::ServiceClient nms_client_;

      int swindow_x;
      int swindow_y;
      float scale_;
      int stack_size_;
      int incrementor_;
      int downsize_;

      std::string run_type_;
      std::string trainer_manifest_filename_;
      std::string object_dataset_filename_;
      std::string nonobject_dataset_filename_;
      std::string ndataset_path_;
      
      std::string model_name_;
      std::string dataset_path_;

      bool override_manifest_;

#if CV_MAJOR_VERSION >= 3 // http://answers.opencv.org/question/46770/cvknearest-missing-in-300-cvmlknearest-abstract/
      cv::Ptr<cv::ml::SVM> supportVectorMachine_;
#else
      boost::shared_ptr<cv::SVM> supportVectorMachine_;
#endif
      boost::shared_ptr<dynamic_reconfigure::Server<
         jsk_perception::SlidingWindowObjectDetectorConfig> > srv_;
      boost::shared_ptr<rosbag::Bag> rosbag_;
   };

} // namespace jsk_perception


#endif  // JSK_PERCEPTION_SLIDING_WINDOW_OBJECT_DETECTOR_H
