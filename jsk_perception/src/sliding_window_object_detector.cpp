
#include <jsk_perception/sliding_window_object_detector.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_perception/NonMaximumSuppression.h>

namespace jsk_perception
{
   void SlidingWindowObjectDetector::onInit()
   {
      DiagnosticNodelet::onInit();

      this->nms_client_ = pnh_->serviceClient<
         jsk_perception::NonMaximumSuppression>("non_maximum_suppression");
      this->srv_ = boost::make_shared<dynamic_reconfigure::Server<
         jsk_perception::SlidingWindowObjectDetectorConfig> >(*pnh_);
      dynamic_reconfigure::Server<
         jsk_perception::SlidingWindowObjectDetectorConfig>::CallbackType f =
         boost::bind(&SlidingWindowObjectDetector::configCallback, this, _1, _2);
      this->srv_->setCallback(f);

      loadTrainedDetectorModel();

      this->pub_rects_ = advertise<jsk_recognition_msgs::RectArray>(
         *pnh_, "output/rects", 1);
      this->pub_image_ = advertise<sensor_msgs::Image>(
         *pnh_, "output/image", 1);
   }

   void SlidingWindowObjectDetector::subscribe()
   {      
      this->sub_ = pnh_->subscribe(
         "/camera/rgb/image_rect_color", 1,
         &SlidingWindowObjectDetector::imageCb, this);
   }
   
   void SlidingWindowObjectDetector::unsubscribe()
   {
      NODELET_DEBUG("Unsubscribing from ROS topic.");
      this->sub_.shutdown();
   }

   void SlidingWindowObjectDetector::imageCb(
      const sensor_msgs::ImageConstPtr& msg)
   {
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
      const int downsize = this->downsize_;
      const float scale = this->scale_;
      const int img_stack = this->stack_size_;
      const int incrementor = this->incrementor_;
      cv::resize(cv_ptr->image, image, cv::Size(
                    isize.width/downsize, isize.height/downsize));
      std::vector<cv::Rect_<int> > bb_rects = this->runObjectRecognizer(
         image, cv::Size(this->swindow_x, this->swindow_y),
         scale, img_stack, incrementor);

      jsk_recognition_msgs::RectArray jsk_rect_array;
      this->convertCvRectToJSKRectArray(
         bb_rects, jsk_rect_array, downsize, isize);
      jsk_rect_array.header = msg->header;
      cv_bridge::CvImage img_bridge(
         msg->header, sensor_msgs::image_encodings::BGR8, image);
      this->pub_rects_.publish(jsk_rect_array);
      this->pub_image_.publish(img_bridge.toImageMsg());
   }
   
   std::vector<cv::Rect_<int> >  SlidingWindowObjectDetector::runObjectRecognizer(
      cv::Mat &image, const cv::Size wsize,
      const float scale, const int scale_counter, const int incrementor)
   {
      if (image.empty()) {
         ROS_ERROR("--INPUT IMAGE IS EMPTY");
         return image;
      }
      cv::Size nwsize = wsize;
      int scounter = 0;
      std::multimap<float, cv::Rect_<int> > detection_info;
      while (scounter++ < scale_counter) {
         this->objectRecognizer(image, detection_info, nwsize, incrementor);
         this->pyramidialScaling(nwsize, scale);
      }
      cv::Mat dimg = image.clone();
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
      image = bimg.clone();
      // cv::imshow("Initial Detection", dimg);
      // cv::imshow("Final Detection", bimg);
      // cv::waitKey(3);
      return object_rects;
   }

   void SlidingWindowObjectDetector::objectRecognizer(
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
             
               cv::resize(roi, roi, cv::Size(this->swindow_x, this->swindow_y));
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

   void SlidingWindowObjectDetector::pyramidialScaling(
      cv::Size &wsize, const float scale)
   {
      float nwidth = wsize.width + (wsize.width * scale);
      float nheight = wsize.height + (wsize.height * scale);
      const int min_swindow_size = 16;
      nwidth = (nwidth < min_swindow_size) ? min_swindow_size : nwidth;
      nheight = (nheight < min_swindow_size) ? min_swindow_size : nheight;
      wsize = cv::Size(std::abs(nwidth), std::abs(nheight));
   }

   std::vector<cv::Rect_<int> > SlidingWindowObjectDetector::nonMaximumSuppression(
      std::multimap<float, cv::Rect_<int> > &detection_info,
      const float nms_threshold)
   {
      if (detection_info.empty()) {
         return std::vector<cv::Rect_<int> >();
      }
      jsk_perception::NonMaximumSuppression srv_nms;
      std::vector<jsk_recognition_msgs::Rect> rect_msg;
      for (std::multimap<float, cv::Rect_<int> >::iterator
              it = detection_info.begin(); it != detection_info.end(); it++) {
         cv::Rect_<int> cv_rect = it->second;
         jsk_recognition_msgs::Rect jsk_rect;
         jsk_rect.x = cv_rect.x;
         jsk_rect.y = cv_rect.y;
         jsk_rect.width = cv_rect.width;
         jsk_rect.height = cv_rect.height;
         srv_nms.request.rect.push_back(jsk_rect);
      }
      srv_nms.request.threshold = nms_threshold;
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
         ROS_ERROR("Failed to call NonMaximumSuppression Module");
         return std::vector<cv::Rect_<int> >();
      }
      return bbox;
   }

   void SlidingWindowObjectDetector::trainObjectClassifier()
   {
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
      
      try {
         this->trainBinaryClassSVM(featureMD, labelMD);
         this->supportVectorMachine_->save(
            (this->model_name_).c_str());
      } catch(std::exception &e) {
         ROS_ERROR("--ERROR: %s", e.what());
      }
   }

   void SlidingWindowObjectDetector::readDataset(
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

   void SlidingWindowObjectDetector::loadTrainedDetectorModel()
   {
      this->supportVectorMachine_ = boost::shared_ptr<cv::SVM>(new cv::SVM);
      this->supportVectorMachine_->load(this->model_name_.c_str());
   }

/**
 * currently programmed using fixed sized image
 */
   void SlidingWindowObjectDetector::extractFeatures(
      const std::vector<cv::Mat> &dataset_img, cv::Mat &featureMD)
   {
      ROS_INFO("--EXTRACTING IMAGE FEATURES");
      for (std::vector<cv::Mat>::const_iterator it = dataset_img.begin();
           it != dataset_img.end(); it++) {
         cv::Mat img = *it;
         cv::resize(img, img, cv::Size(this->swindow_x, this->swindow_y));
         if (img.data) {
            cv::Mat _feature = this->computeHOG(img);
            featureMD.push_back(_feature);
         }
         cv::imshow("image", img);
         cv::waitKey(3);
      }
   }

   void SlidingWindowObjectDetector::trainBinaryClassSVM(
      const cv::Mat &featureMD, const cv::Mat &labelMD)
   {
      // std::cout << featureMD.size() << labelMD.size() << std::endl;
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

      this->supportVectorMachine_->train_auto
         (featureMD, labelMD, cv::Mat(), cv::Mat(), svm_param, 10,
          paramGrid, cv::SVM::get_default_grid(cv::SVM::GAMMA),
          cv::SVM::get_default_grid(cv::SVM::P),
          cv::SVM::get_default_grid(cv::SVM::NU),
          cv::SVM::get_default_grid(cv::SVM::COEF),
          cv::SVM::get_default_grid(cv::SVM::DEGREE),
          true);
   }

   void SlidingWindowObjectDetector::concatenateCVMat(
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

   void SlidingWindowObjectDetector::convertCvRectToJSKRectArray(
      const std::vector<cv::Rect_<int> > &bounding_boxes,
      jsk_recognition_msgs::RectArray &jsk_rects,
      const int downsize, const cv::Size img_sz)
   {
      for (std::vector<cv::Rect_<int> >::const_iterator it =
              bounding_boxes.begin(); it != bounding_boxes.end(); it++) {
         jsk_recognition_msgs::Rect j_r;
         j_r.x = it->x * downsize;
         j_r.y = it->y * downsize;
         j_r.width = it->width * downsize;
         j_r.height = it->height * downsize;
         jsk_rects.rects.push_back(j_r);
      }
   }

   void SlidingWindowObjectDetector::configCallback(
      jsk_perception::SlidingWindowObjectDetectorConfig &config, uint32_t level)
   {
      boost::mutex::scoped_lock lock(mutex_);
      this->scale_ = static_cast<float>(config.scaling_factor);
      this->stack_size_ = static_cast<int>(config.stack_size);
      this->incrementor_ = config.sliding_window_increment;
      this->model_name_ = config.svm_model_name;
      this->dataset_path_ = config.dataset_directory;
      
      // currently fixed variables
      this->swindow_x = config.sliding_window_width;
      this->swindow_y = config.sliding_window_height;
      this->downsize_ = config.image_downsize;
   }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::SlidingWindowObjectDetector, nodelet::Nodelet);
