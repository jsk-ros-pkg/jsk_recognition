
#include <jsk_perception/trained_object_detector.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_perception/NonMaximumSuppression.h>

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <algorithm>
#include <utility>

namespace jsk_perception
{
   TrainedObjectDetector::TrainedObjectDetector() :
      it_(*pnh_),
      supportVectorMachine_(new cv::SVM),
      swindow_(64/2, 128/2), DiagnosticNodelet("TrainedObjectDetector")
   {
      this->nms_client_ = pnh_->serviceClient<
         jsk_perception::NonMaximumSuppression>("non_maximum_suppression");
    
      bool isTrain = false;
      if (isTrain) {
         ROS_INFO("--Training Classifier");
         trainObjectClassifier();
         exit(-1);
      } else {
         ROS_INFO("--Loading Trained SVM Classifier");
         // this->supportVectorMachine_->load("svm.xml");
      }
   }

   void TrainedObjectDetector::onInit()
   {
      DiagnosticNodelet::onInit();
      
      this->srv_ = boost::make_shared<dynamic_reconfigure::Server<
         jsk_perception::TrainedObjectDetectorConfig> >(*pnh_);
      dynamic_reconfigure::Server<
         jsk_perception::TrainedObjectDetectorConfig>::CallbackType f =
         boost::bind(&TrainedObjectDetector::configCallback, this, _1, _2);
      this->srv_->setCallback(f);
      
      this->pub_indices_ = this->pnh_->advertise<
         jsk_recognition_msgs::ClusterPointIndices>(
            "output/indices", 1);
      this->image_pub_ = this->it_.advertise(
         "output/image", 1);
   }
   
   void TrainedObjectDetector::subscribe()
   {
      this->image_sub_ = this->it_.subscribe(
         "input", 1, &TrainedObjectDetector::imageCb, this);
   }

   void TrainedObjectDetector::updateDiagnostic(
       diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                 "TrainedObjectDetector Running");
  }
   
   void TrainedObjectDetector::imageCb(
      const sensor_msgs::ImageConstPtr& msg)
   {
      boost::mutex::scoped_lock lock(this->mutex_);
      cv_bridge::CvImagePtr cv_ptr;
      try {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
      cv::Mat image;
      cv::Size isize = cv_ptr->image.size();
      // control params
      const int downsize = 2;
      const float scale = this->scale_;
      const int img_stack = this->stack_size_;
      const int incrementor = this->incrementor_;

      /*
      cv::resize(cv_ptr->image, image, cv::Size(
                    isize.width/downsize, isize.height/downsize));
      std::vector<cv::Rect_<int> > bb_rects = this->runObjectRecognizer(
         image, this->swindow_, scale, img_stack, incrementor);

      
      std::vector<pcl::PointIndices> bb_cluster_indices;
      this->objectBoundingBoxPointCloudIndices(
         bb_rects, bb_cluster_indices, downsize, isize);
      jsk_recognition_msgs::ClusterPointIndices ros_indices;
      ros_indices.cluster_indices = pcl_conversions::convertToROSPointIndices(
         bb_cluster_indices, msg->header);
      ros_indices.header = msg->header;
      this->pub_indices_.publish(ros_indices);
      */
      this->image_pub_.publish(cv_ptr->toImageMsg());
   }
   
   std::vector<cv::Rect_<int> > TrainedObjectDetector::runObjectRecognizer(
      const cv::Mat &image, const cv::Size wsize,
      const float scale, const int scale_counter, const int incrementor)
   {
      if (image.empty()) {
         ROS_ERROR("--INPUT IMAGE IS EMPTY");
      }
      cv::Size nwsize = wsize;
      int scounter = 0;
      std::multimap<float, cv::Rect_<int> > detection_info;
      while (scounter++ < scale_counter) {
         this->objectRecognizer(image, detection_info, nwsize, incrementor);
         this->pyramidialScaling(nwsize, scale);
      }
      cv::Mat dimg = image.clone();
      std::cout << "Info Size: " << detection_info.size()  << std::endl;
      for (std::multimap<float, cv::Rect_<int> >::iterator
              it = detection_info.begin(); it != detection_info.end(); it++) {
         cv::rectangle(dimg, it->second, cv::Scalar(0, 255, 0), 2);
      }
      const float nms_threshold = 0.05;
      std::vector<cv::Rect_<int> > object_rects = this->nonMaximumSuppression(
         detection_info, nms_threshold);
      cv::Mat bimg = image.clone();
      for (std::vector<cv::Rect_<int> >::iterator it = object_rects.begin();
           it != object_rects.end(); it++) {
         cv::rectangle(bimg, *it, cv::Scalar(0, 0, 255), 2);
      }
      cv::imshow("Initial Detection", dimg);
      cv::imshow("Final Detection", bimg);
      cv::waitKey(3);

      return object_rects;
   }

   void TrainedObjectDetector::objectRecognizer(
      const cv::Mat &image, std::multimap<float, cv::Rect_<int> > &detection_info,
      const cv::Size wsize, const int incrementor)
   {
      for (int j = 0; j < image.rows; j += incrementor) {
         for (int i = 0; i < image.cols; i += incrementor) {
            cv::Rect_<int> rect = cv::Rect_<int>(i, j, wsize.width, wsize.height);
            if ((rect.x + rect.width <= image.cols) &&
                (rect.y + rect.height <= image.rows)) {
               cv::Mat roi = image(rect).clone();
               cv::GaussianBlur(roi, roi, cv::Size(3, 3), 1.0);
             
               cv::resize(roi, roi, this->swindow_);
               cv::Mat roi_feature = this->computeHOG(roi);
               float response = this->supportVectorMachine_->predict(
                  roi_feature, false);             
               if (response == 1) {
                  detection_info.insert(std::make_pair(response, rect));
               } else {
                  continue;
               }
            }
         }
      }
   }

   void TrainedObjectDetector::pyramidialScaling(
      cv::Size &wsize, const float scale)
   {
      float nwidth = wsize.width + (wsize.width * scale);
      float nheight = wsize.height + (wsize.height * scale);
      const int min_swindow_size = 16;
      nwidth = (nwidth < min_swindow_size) ? min_swindow_size : nwidth;
      nheight = (nheight < min_swindow_size) ? min_swindow_size : nheight;
      wsize = cv::Size(std::abs(nwidth), std::abs(nheight));
   }

   std::vector<cv::Rect_<int> > TrainedObjectDetector::nonMaximumSuppression(
      std::multimap<float, cv::Rect_<int> > &detection_info,
      const float nms_threshold) {
      if (detection_info.empty()) {
         return std::vector<cv::Rect_<int> >();
      }
      jsk_perception::NonMaximumSuppression srv_nms;
      std::vector<jsk_recognition_msgs::Rect> rect_msg;
      for (std::multimap<float, cv::Rect_<int> >::iterator
              it = detection_info.begin(); it != detection_info.end(); it++)
      {
         cv::Rect_<int> cv_rect = it->second;
         jsk_recognition_msgs::Rect jsk_rect;
         jsk_rect.x = cv_rect.x;
         jsk_rect.y = cv_rect.y;
         jsk_rect.width = cv_rect.width;
         jsk_rect.height = cv_rect.height;
         srv_nms.request.rect.push_back(jsk_rect);
      }
      srv_nms.request.threshold = nms_threshold;;
      std::vector<cv::Rect_<int> > bbox;
      if (this->nms_client_.call(srv_nms)) {
         for (int i = 0; i < srv_nms.response.bbox_count; i++) {
            cv::Rect_<int> brect = cv::Rect_<int>(
               srv_nms.response.bbox[i].x,
               srv_nms.response.bbox[i].y,
               srv_nms.response.bbox[i].width,
               srv_nms.response.bbox[i].height);
            bbox.push_back(brect);
         }
      } else {
         ROS_ERROR("Failed to call service add_two_ints");
         return std::vector<cv::Rect_<int> >();
      }
      return bbox;
   }

   void TrainedObjectDetector::trainObjectClassifier() {
      // reading the positive training image
      std::string pfilename = dataset_path_ + "dataset/train.txt";
      std::vector<cv::Mat> pdataset_img;
      cv::Mat labelMD;
      this->readDataset(pfilename, pdataset_img, labelMD, true, 1);

      // reading the negative training image
      std::string nfilename = dataset_path_ + "dataset/negative.txt";
      std::vector<cv::Mat> ndataset_img;
      this->readDataset(nfilename, ndataset_img, labelMD, true, -1);

      pdataset_img.insert(
         pdataset_img.end(), ndataset_img.begin(), ndataset_img.end());
       
      cv::Mat featureMD;
      this->extractFeatures(pdataset_img, featureMD);
      std::cout << featureMD.size() << std::endl;
    
      try {
         this->trainBinaryClassSVM(featureMD, labelMD);
         this->supportVectorMachine_->save(
            (this->dataset_path_ + "dataset/svm.xml").c_str());
      } catch(std::exception &e) {
         ROS_ERROR("--ERROR: %s", e.what());
      }
   }

   void TrainedObjectDetector::readDataset(
      std::string filename, std::vector<cv::Mat> &dataset_img, cv::Mat &labelMD,
      bool is_usr_label, const int usr_label)
   {
      ROS_INFO("--READING DATASET IMAGE");
      std::ifstream infile;
      infile.open(filename.c_str(), std::ios::in);
      char buffer[255];
      if (!infile.eof()) {
         while (infile.good()) {
            infile.getline(buffer, 255);
            std::string _line(buffer);
            if (!_line.empty()) {
               std::istringstream iss(_line);
               std::string _path;
               iss >> _path;
               cv::Mat img = cv::imread(this->dataset_path_+ _path,
                                        CV_LOAD_IMAGE_COLOR);
               float label;
               if (!is_usr_label) {
                  std::string _label;
                  iss >> _label;
                  label = std::atoi(_label.c_str());
               } else {
                  label = static_cast<float>(usr_label);
               }
               if (img.data) {
                  labelMD.push_back(label);
                  dataset_img.push_back(img);
               }
            }
         }
      }
   }

/**
 * currently programmed using fixed sized image
 */
   void TrainedObjectDetector::extractFeatures(
      const std::vector<cv::Mat> &dataset_img, cv::Mat &featureMD)
   {
      ROS_INFO("--EXTRACTING IMAGE FEATURES.");
      for (std::vector<cv::Mat>::const_iterator it = dataset_img.begin();
           it != dataset_img.end(); it++) {
         cv::Mat img = *it;
         cv::resize(img, img, this->swindow_);
         if (img.data) {
            cv::Mat _feature = this->computeHOG(img);
            featureMD.push_back(_feature);
            // std::cout << featureMD << std::endl;
         }
       
         cv::imshow("image", img);
         cv::waitKey(3);
      }
   }

   void TrainedObjectDetector::trainBinaryClassSVM(
      const cv::Mat &featureMD, const cv::Mat &labelMD)
   {
      std::cout << featureMD.size() << labelMD.size() << std::endl;
      ROS_INFO("--TRAINING CLASSIFIER");
      cv::SVMParams svm_param = cv::SVMParams();
      svm_param.svm_type = cv::SVM::NU_SVC;
      svm_param.kernel_type = cv::SVM::RBF;
      svm_param.degree = 0.0;
      svm_param.gamma = 0.90;
      svm_param.coef0 = 0.50;
      svm_param.C = 100;
      svm_param.nu = 0.70;
      svm_param.p = 1.0;
      svm_param.class_weights = NULL;
      svm_param.term_crit.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
      svm_param.term_crit.max_iter = 1e6;
      svm_param.term_crit.epsilon = 1e-6;
      cv::ParamGrid paramGrid = cv::ParamGrid();
      paramGrid.min_val = 0;
      paramGrid.max_val = 0;
      paramGrid.step = 1;

      /*this->supportVectorMachine_->train(
        featureMD, labelMD, cv::Mat(), cv::Mat(), svm_param);*/
      this->supportVectorMachine_->train_auto
         (featureMD, labelMD, cv::Mat(), cv::Mat(), svm_param, 10,
          paramGrid, cv::SVM::get_default_grid(cv::SVM::GAMMA),
          cv::SVM::get_default_grid(cv::SVM::P),
          cv::SVM::get_default_grid(cv::SVM::NU),
          cv::SVM::get_default_grid(cv::SVM::COEF),
          cv::SVM::get_default_grid(cv::SVM::DEGREE),
          true);
   }

   void TrainedObjectDetector::concatenateCVMat(
      const cv::Mat &mat_1, const cv::Mat &mat_2,
      cv::Mat &featureMD, bool iscolwise)
   {
      if (iscolwise) {
         featureMD = cv::Mat(mat_1.rows, (mat_1.cols + mat_2.cols), CV_32F);
         for (int i = 0; i < featureMD.rows; i++) {
            for (int j = 0; j < mat_1.cols; j++) {
               featureMD.at<float>(i, j) = mat_1.at<float>(i, j);
            }
            for (int j = mat_1.cols; j < featureMD.cols; j++) {
               featureMD.at<float>(i, j) = mat_2.at<float>(i, j - mat_1.cols);
            }
         }
      } else {
         featureMD = cv::Mat((mat_1.rows + mat_2.rows), mat_1.cols, CV_32F);
         for (int i = 0; i < featureMD.cols; i++) {
            for (int j = 0; j < mat_1.rows; j++) {
               featureMD.at<float>(j, i) = mat_1.at<float>(j, i);
            }
            for (int j = mat_1.rows; j < featureMD.rows; j++) {
               featureMD.at<float>(j, i) = mat_2.at<float>(j - mat_1.rows, i);
            }
         }
      }
   }

   void TrainedObjectDetector::objectBoundingBoxPointCloudIndices(
      const std::vector<cv::Rect_<int> > &bounding_boxes,
      std::vector<pcl::PointIndices> &cluster_indices,
      const int downsize, const cv::Size img_sz)
   {
      cluster_indices.clear();
      for (std::vector<cv::Rect_<int> >::const_iterator it =
              bounding_boxes.begin(); it != bounding_boxes.end(); it++) {
         int x = it->x * downsize;
         int y = it->y * downsize;
         int w = it->width * downsize;
         int h = it->height * downsize;
         pcl::PointIndices _indices;
         for (int j = y; j < (y + h); j++) {
            for (int i = x; i < (x + w); i++) {
               int _index = (i + (j * img_sz.width));
               _indices.indices.push_back(_index);
            }
         }
         cluster_indices.push_back(_indices);
         _indices.indices.clear();
      }
   }

   void TrainedObjectDetector::configCallback(
      jsk_perception::TrainedObjectDetectorConfig &config, uint32_t level)
   {
      boost::mutex::scoped_lock lock(mutex_);
      this->scale_ = static_cast<float>(config.scaling_factor);
      this->stack_size_ = static_cast<int>(config.stack_size);
      this->incrementor_ = config.sliding_window_increment;
   }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::TrainedObjectDetector, nodelet::Nodelet);
