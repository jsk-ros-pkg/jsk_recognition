
#ifndef _TRAINED_OBJECT_DETECTOR_H_
#define _TRAINED_OBJECT_DETECTOR_H_

#include <jsk_perception/histogram_of_oriented_gradients.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_perception/TrainedObjectDetectorConfig.h>

#include <pcl/point_types.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <map>
#include <vector>
#include <string>

namespace jsk_perception
{
   class TrainedObjectDetector: public HOGFeatureDescriptor,
                                public jsk_topic_tools::DiagnosticNodelet
   {
    private:
   
      image_transport::ImageTransport it_;
      image_transport::Subscriber image_sub_;
      image_transport::Publisher image_pub_;
      ros::Publisher pub_indices_;   
      ros::ServiceClient nms_client_;
      
      boost::shared_ptr<cv::SVM> supportVectorMachine_;
      void concatenateCVMat(
         const cv::Mat &, const cv::Mat &, cv::Mat &, bool = true);
   
    public:
      TrainedObjectDetector();
      virtual void trainObjectClassifier();
      virtual void readDataset(
         std::string, std::vector<cv::Mat> &,
         cv::Mat &, bool = false, const int = 0);
      virtual void extractFeatures(
         const std::vector<cv::Mat> &, cv::Mat &);
      virtual void trainBinaryClassSVM(
         const cv::Mat &, const cv::Mat &);
      virtual void imageCb(
         const sensor_msgs::ImageConstPtr&);
      virtual std::vector<cv::Rect_<int> > runObjectRecognizer(
         const cv::Mat &, const cv::Size, const float, const int, const int);
      virtual void objectRecognizer(
         const cv::Mat &, std::multimap<float, cv::Rect_<int> > &,
         const cv::Size, const int = 16);
      virtual void pyramidialScaling(
         cv::Size &, const float);
      virtual std::vector<cv::Rect_<int> > nonMaximumSuppression(
         std::multimap<float, cv::Rect_<int> > &, const float);
      virtual void objectBoundingBoxPointCloudIndices(
         const std::vector<cv::Rect_<int> > &,
         std::vector<pcl::PointIndices> &,
         const int,
         const cv::Size);

    protected:
       virtual void onInit();
       virtual void subscribe();
       virtual void unsubscribe();
       virtual void updateDiagnostic(
          diagnostic_updater::DiagnosticStatusWrapper &);
       virtual void configCallback(
          jsk_perception::TrainedObjectDetectorConfig &, uint32_t);

       boost::shared_ptr<dynamic_reconfigure::Server<
          jsk_perception::TrainedObjectDetectorConfig> > srv_;
       boost::mutex mutex_;
       float scale_;
       int stack_size_;
       int incrementor_;
       cv::Size swindow_;
       std::string dataset_path_;
   };
}  // namespace jsk_perception

#endif  // _TRAINED_OBJECT_DETECTOR_H_
