#ifndef _SLIDING_WINDOW_OBJECT_DETECTOR_TRAINER_H_
#define _SLIDING_WINDOW_OBJECT_DETECTOR_TRAINER_H_

#include <jsk_perception/histogram_of_oriented_gradients.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <string>
#include <fstream>

namespace jsk_perception
{
   class SlidingWindowObjectDetectorTrainer: public HOGFeatureDescriptor
                                             // public LocalBinaryPatterns
   {
    private:
      ros::NodeHandle nh_;
      
      int swindow_x_;
      int swindow_y_;
      int hist_hbin_;
      int hist_sbin_;
      
      std::string dataset_path_;

      std::string object_dataset_filename_;
      std::string object_dataset_topic_;
      std::string nonobject_dataset_filename_;
      std::string nonobject_dataset_topic_;
      std::string trained_classifier_name_;
      std::string manifest_filename_;

      boost::shared_ptr<rosbag::Bag> rosbag_;
#if CV_MAJOR_VERSION >= 3 // http://answers.opencv.org/question/46770/cvknearest-missing-in-300-cvmlknearest-abstract/
      cv::Ptr<cv::ml::SVM> supportVectorMachine_;
#else
      boost::shared_ptr<cv::SVM> supportVectorMachine_;
#endif
      void writeTrainingManifestToDirectory(cv::FileStorage &);
      virtual void concatenateCVMat(
         const cv::Mat &, const cv::Mat &, cv::Mat &, bool = true);

    public:
      SlidingWindowObjectDetectorTrainer();
      // :DiagnosticNodelet("SlidingWindowObjectDetectorTrainer");
      
      virtual void trainObjectClassifier(
         std::string, std::string);
      virtual void readDataset(
         std::string, std::string, cv::Mat &,
         cv::Mat &, bool = false, const int = 0);
      virtual void extractFeatures(
         cv::Mat &, cv::Mat &);
      virtual void trainBinaryClassSVM(
         const cv::Mat &, const cv::Mat &);

      // temp placed here
      virtual void computeHSHistogram(
         cv::Mat &, cv::Mat &, const int = 64, const int = 32, bool = true);
  
   };
}

#endif  // _SLIDING_WINDOW_OBJECT_DETECTOR_TRAINER_H_
