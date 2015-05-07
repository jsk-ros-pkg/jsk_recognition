
#include <jsk_perception/sliding_window_object_detector_trainer.h>

#include <iostream>
namespace jsk_perception
{
   SlidingWindowObjectDetectorTrainer::SlidingWindowObjectDetectorTrainer() :
      supportVectorMachine_(new cv::SVM)
   {
      nh_.getParam("dataset_path", this->dataset_path_);
      nh_.getParam("object_dataset_filename", this->object_dataset_filename_);
      nh_.getParam("nonobject_dataset_filename", this->nonobject_dataset_filename_);
      nh_.getParam("classifier_name", this->trained_classifier_name_);
      nh_.getParam("swindow_x", this->swindow_x_);
      nh_.getParam("swindow_y", this->swindow_y_);

      ROS_INFO("--Training Classifier");
      std::string pfilename = dataset_path_ + this->object_dataset_filename_;
      std::string nfilename = dataset_path_ + this->nonobject_dataset_filename_;
      trainObjectClassifier(pfilename, nfilename);
      ROS_INFO("--Trained Successfully..");

      /*write the training manifest*/
      std::string mainfest_filename = "sliding_window_trainer_manifest.xml";
      cv::FileStorage fs = cv::FileStorage(
         mainfest_filename, cv::FileStorage::WRITE);
      this->writeTrainingManifestToDirectory(fs);
      fs.release();

      cv::destroyAllWindows();
      nh_.shutdown();
   }
   
   void SlidingWindowObjectDetectorTrainer::trainObjectClassifier(
      std::string pfilename, std::string nfilename)
   {
      // reading the positive training image
      std::vector<cv::Mat> pdataset_img;
      cv::Mat featureMD;
      cv::Mat labelMD;
      this->readDataset(pfilename, featureMD, labelMD, true, 1);
      ROS_INFO("Info: Total Object Sample: %d", featureMD.rows);
    
      // reading the negative training image
      std::vector<cv::Mat> ndataset_img;
      this->readDataset(nfilename, featureMD, labelMD, true, -1);
      ROS_INFO("Info: Total Training Features: %d", featureMD.rows);
    
      try {
         this->trainBinaryClassSVM(featureMD, labelMD);
         this->supportVectorMachine_->save(
            this->trained_classifier_name_.c_str());
      } catch(std::exception &e) {
         ROS_ERROR("--ERROR: %s", e.what());
      }
   }

   void SlidingWindowObjectDetectorTrainer::readDataset(
      std::string filename, cv::Mat &featureMD, cv::Mat &labelMD,
      bool is_usr_label, const int usr_label) {
      ROS_INFO("--READING DATASET IMAGE");
      std::ifstream infile;
      infile.open(filename.c_str(), std::ios::in);
      char buffer[255];

      std::cout << "FileName: " << filename << std::endl;
      
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
                  this->extractFeatures(img, featureMD);
               }
            }
         }
      } else {
         ROS_ERROR("INPUT FILE NOT FOUND");
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
         // cv::Mat lbp_feature = this->computeLBP(
         //   img, cv::Size(8, 8), 10, false, true);
         cv::Mat _feature = hog_feature;
         // this->concatenateCVMat(hog_feature, lbp_feature, _feature, true);
         featureMD.push_back(_feature);
      }
      cv::imshow("image", img);
      cv::waitKey(3);
   }

   void SlidingWindowObjectDetectorTrainer::trainBinaryClassSVM(
      const cv::Mat &featureMD, const cv::Mat &labelMD)
   {
      ROS_INFO("--TRAINING CLASSIFIER");
      cv::SVMParams svm_param = cv::SVMParams();
      svm_param.svm_type = cv::SVM::NU_SVC;
      svm_param.kernel_type = cv::SVM::LINEAR;
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
      fs << "LBP" << 0;
      // fs << "SIFT" << 0;
      // fs << "SURF" << 0;
      fs << "COLOR_HISTOGRAM" << 0;
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
}  // namespace jsk_perception

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sliding_window_object_detector_trainer_node");
    ROS_INFO("RUNNING NODELET %s", "sliding_window_object_detector_trainer");
    jsk_perception::SlidingWindowObjectDetectorTrainer run_trainer;
    ros::spin();
    return 0;    
}
