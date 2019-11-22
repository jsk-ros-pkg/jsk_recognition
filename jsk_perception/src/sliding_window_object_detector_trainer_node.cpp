#include <jsk_topic_tools/log_utils.h>
#include <jsk_perception/sliding_window_object_detector_trainer.h>

#include <iostream>
namespace jsk_perception
{
   SlidingWindowObjectDetectorTrainer::SlidingWindowObjectDetectorTrainer()
#if CV_MAJOR_VERSION < 3
     : supportVectorMachine_(new cv::SVM)
#endif
   {
#if CV_MAJOR_VERSION >= 3
      this->supportVectorMachine_ = cv::ml::SVM::create();
#endif
      nh_.getParam("dataset_path", this->dataset_path_);
      nh_.getParam("object_dataset_filename", this->object_dataset_filename_);
      nh_.param<std::string>("object_dataset_topic", this->object_dataset_topic_, "/dataset/roi");
      nh_.getParam("nonobject_dataset_filename", this->nonobject_dataset_filename_);
      nh_.param<std::string>("nonobject_dataset_topic", this->nonobject_dataset_topic_, "/dataset/background/roi");
      nh_.getParam("classifier_name", this->trained_classifier_name_);
      nh_.getParam("swindow_x", this->swindow_x_);
      nh_.getParam("swindow_y", this->swindow_y_);
      nh_.param<std::string>("manifest_filename", this->manifest_filename_, "sliding_window_trainer_manifest.xml");

      ROS_INFO("--Training Classifier");
      std::string pfilename = dataset_path_ + this->object_dataset_filename_;
      std::string nfilename = dataset_path_ + this->nonobject_dataset_filename_;
      trainObjectClassifier(pfilename, nfilename);
      ROS_INFO("--Trained Successfully..");

      /*write the training manifest*/
      std::string manifest_filename = this->manifest_filename_;
      cv::FileStorage fs(manifest_filename, cv::FileStorage::WRITE);
      this->writeTrainingManifestToDirectory(fs);
      fs.release();

      cv::destroyAllWindows();
      nh_.shutdown();
   }
   
   void SlidingWindowObjectDetectorTrainer::trainObjectClassifier(
      std::string pfilename, std::string nfilename)
   {
      cv::Mat featureMD;
      cv::Mat labelMD;
      std::string topic_name = this->object_dataset_topic_;
      this->readDataset(pfilename, topic_name, featureMD, labelMD, true, 1);
      ROS_INFO("Info: Total Object Sample: %d", featureMD.rows);
      
      topic_name = this->nonobject_dataset_topic_;
      this->readDataset(nfilename, topic_name, featureMD, labelMD, true, -1);
      ROS_INFO("Info: Total Training Features: %d", featureMD.rows);
    
      try {
         this->trainBinaryClassSVM(featureMD, labelMD);
         this->supportVectorMachine_->save(
            this->trained_classifier_name_.c_str());
      } catch(std::exception &e) {
         ROS_ERROR("--ERROR: PLEASE CHECK YOUR DATA \n%s", e.what());
         std::_Exit(EXIT_FAILURE);
      }
   }

   void SlidingWindowObjectDetectorTrainer::readDataset(
      std::string filename, std::string topic_name, cv::Mat &featureMD,
      cv::Mat &labelMD, bool is_usr_label, const int usr_label) {
      ROS_INFO("--READING DATASET IMAGE");
      try {
         rosbag_ = boost::shared_ptr<rosbag::Bag>(new rosbag::Bag);
         this->rosbag_->open(filename, rosbag::bagmode::Read);
         ROS_INFO("Bag Found and Opened Successfully...");
         std::vector<std::string> topics;
         topics.push_back(std::string(topic_name));
         rosbag::View view(*rosbag_, rosbag::TopicQuery(topics));
         BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            sensor_msgs::Image::ConstPtr img_msg = m.instantiate<
               sensor_msgs::Image>();
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
               img_msg, sensor_msgs::image_encodings::BGR8);
            if (cv_ptr->image.data) {
               cv::Mat image = cv_ptr->image.clone();
               float label = static_cast<float>(usr_label);
               labelMD.push_back(label);
               this->extractFeatures(image, featureMD);
               cv::imshow("image", image);
               cv::waitKey(3);
            } else {
               ROS_WARN("-> NO IMAGE");
            }
         }
         this->rosbag_->close();
      } catch (ros::Exception &e) {
         ROS_ERROR("ERROR: Bag File:%s not found..\n%s",
                   filename.c_str(), e.what());
         std::_Exit(EXIT_FAILURE);
      }
   }

/**
 * currently programmed using fixed sized image
 */
   void SlidingWindowObjectDetectorTrainer::extractFeatures(
      cv::Mat &img, cv::Mat &featureMD) {
      ROS_INFO("--EXTRACTING IMAGE FEATURES.");
      if (img.data) {
         cv::resize(img, img, cv::Size(this->swindow_x_, this->swindow_y_));
         cv::Mat hog_feature = this->computeHOG(img);
         cv::Mat hsv_feature;
         this->computeHSHistogram(img, hsv_feature, 16, 16, true);
         hsv_feature = hsv_feature.reshape(1, 1);
         cv::Mat _feature;
         this->concatenateCVMat(hog_feature, hsv_feature, _feature, true);
         featureMD.push_back(_feature);
      }
      cv::imshow("image", img);
      cv::waitKey(3);
   }

   void SlidingWindowObjectDetectorTrainer::trainBinaryClassSVM(
      const cv::Mat &featureMD, const cv::Mat &labelMD)
   {
      ROS_INFO("--TRAINING CLASSIFIER");
#if CV_MAJOR_VERSION >= 3
      this->supportVectorMachine_->setType(cv::ml::SVM::NU_SVC);
      //this->supportVectorMachine_->setKernelType(cv::ml::SVM::RBF);
      this->supportVectorMachine_->setDegree(0.0);
      this->supportVectorMachine_->setGamma(0.90);
      this->supportVectorMachine_->setCoef0(0.50);
      this->supportVectorMachine_->setC(1);
      this->supportVectorMachine_->setNu(0.70);
      this->supportVectorMachine_->setP(1.0);
      //this->supportVectorMachine_->setClassWeights(NULL);
      cv::TermCriteria term_crit;
      term_crit.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
      term_crit.maxCount = 1e6;
      term_crit.epsilon = 1e-6;
      this->supportVectorMachine_->setTermCriteria(term_crit);
      cv::ml::ParamGrid paramGrid = cv::ml::ParamGrid();
      paramGrid.minVal = 0;
      paramGrid.maxVal = 0;
      paramGrid.logStep = 1;

      cv::Ptr<cv::ml::TrainData> train = cv::ml::TrainData::create(featureMD, cv::ml::ROW_SAMPLE, labelMD, cv::Mat(), cv::Mat()); // ROW_SAMPLE ? COL_SAMPLE ?
      this->supportVectorMachine_->trainAuto
        (train, 10,
         paramGrid, cv::ml::SVM::getDefaultGrid(cv::ml::SVM::GAMMA),
         cv::ml::SVM::getDefaultGrid(cv::ml::SVM::P),
         cv::ml::SVM::getDefaultGrid(cv::ml::SVM::NU),
         cv::ml::SVM::getDefaultGrid(cv::ml::SVM::COEF),
         cv::ml::SVM::getDefaultGrid(cv::ml::SVM::DEGREE),
          true);
#else
      cv::SVMParams svm_param = cv::SVMParams();
      svm_param.svm_type = cv::SVM::NU_SVC;
      svm_param.kernel_type = cv::SVM::RBF;
      svm_param.degree = 0.0;
      svm_param.gamma = 0.90;
      svm_param.coef0 = 0.50;
      svm_param.C = 1;
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
#endif
   }

   void SlidingWindowObjectDetectorTrainer::concatenateCVMat(
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
   
   void SlidingWindowObjectDetectorTrainer::writeTrainingManifestToDirectory(
      cv::FileStorage &fs)
   {
      fs <<  "TrainerInfo" << "{";
      fs << "trainer_type" << "cv::SVM";
      fs << "trainer_path" << this->trained_classifier_name_;
      fs << "}";

      fs <<  "FeatureInfo" << "{";
      fs << "HOG" << 1;
      // fs << "LBP" << 0;
      // fs << "SIFT" << 0;
      // fs << "SURF" << 0;
      fs << "COLOR_HISTOGRAM" << 1;
      fs << "}";
    
      fs <<  "SlidingWindowInfo" << "{";
      fs << "swindow_x" << this->swindow_x_;
      fs << "swindow_y" << this->swindow_y_;
      fs << "}";
    
      fs << "TrainingDatasetDirectoryInfo" << "{";
      fs << "object_dataset_filename" << this->object_dataset_filename_;
      fs << "nonobject_dataset_filename" << this->nonobject_dataset_filename_;
      fs << "dataset_path" << this->dataset_path_;  // only path to neg
      fs << "}";
   }
   
   void SlidingWindowObjectDetectorTrainer::computeHSHistogram(
      cv::Mat &src, cv::Mat &hist, const int hBin, const int sBin, bool is_norm)
   {
      if (src.empty()) {
         return;
      }
      cv::Mat hsv;
      cv::cvtColor(src, hsv, CV_BGR2HSV);
      int histSize[] = {hBin, sBin};
      float h_ranges[] = {0, 180};
      float s_ranges[] = {0, 256};
      const float* ranges[] = {h_ranges, s_ranges};
      int channels[] = {0, 1};
      cv::calcHist(
         &hsv, 1, channels, cv::Mat(), hist, 2, histSize, ranges, true, false);
      if (is_norm) {
         cv::normalize(hist, hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
      }
   }
}  // namespace jsk_perception

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sliding_window_object_detector_trainer_node");
    ROS_INFO("RUNNING NODELET %s", "sliding_window_object_detector_trainer");
    jsk_perception::SlidingWindowObjectDetectorTrainer run_trainer;
    ros::spin();
    return 0;    
}
