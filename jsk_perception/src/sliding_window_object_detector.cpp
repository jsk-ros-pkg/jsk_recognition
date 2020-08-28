// @author Krishneel Chaudhary, JSK

#include <jsk_perception/sliding_window_object_detector.h>
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_msgs/Rect.h>
#include <jsk_perception/NonMaximumSuppression.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif

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
      
      pnh_->getParam("run_type", this->run_type_);
      pnh_->getParam("trainer_manifest", this->trainer_manifest_filename_);
      pnh_->param<bool>("override_manifest", this->override_manifest_, false);

      ROS_INFO("RUN TYPE: %s", run_type_.c_str());
      ROS_INFO("LOADED TRAINER MANIFEST: %s", trainer_manifest_filename_.c_str());

      this->readTrainingManifestFromDirectory();
      loadTrainedDetectorModel();
      if (this->run_type_.compare("BOOTSTRAPER") == 0) {
         try {
            std::string _topic = "/dataset/background/roi";
            boost::shared_ptr<rosbag::Bag> tmp_bag(new rosbag::Bag);
            tmp_bag->open(this->ndataset_path_, rosbag::bagmode::Read);
            ROS_INFO("Bag Found and Opened Successfully ...");
            std::vector<std::string> topics;
            topics.push_back(_topic);
            rosbag::View view(*tmp_bag, rosbag::TopicQuery(topics));
            std::vector<sensor_msgs::Image> tmp_imgs;
            BOOST_FOREACH(rosbag::MessageInstance const m, view) {
               sensor_msgs::Image::ConstPtr img_msg = m.instantiate<
               sensor_msgs::Image>();
               tmp_imgs.push_back(*img_msg);
            }
            tmp_bag->close();
            this->rosbag_ = boost::shared_ptr<rosbag::Bag>(new rosbag::Bag);
            this->rosbag_->open(this->ndataset_path_, rosbag::bagmode::Write);
            for (std::vector<sensor_msgs::Image>::iterator it = tmp_imgs.begin();
                 it != tmp_imgs.end(); it++) {
               this->rosbag_->write(_topic, ros::Time::now(), *it);
            }
         } catch (ros::Exception &e) {
            ROS_ERROR("ERROR: Bag File not found..\n%s", e.what());
            std::_Exit(EXIT_FAILURE);
         }         
      }
      this->pub_rects_ = advertise<jsk_recognition_msgs::RectArray>(
         *pnh_, "output/rects", 1);
      this->pub_image_ = advertise<sensor_msgs::Image>(
         *pnh_, "output/image", 1);
      onInitPostProcess();
   }

   void SlidingWindowObjectDetector::subscribe()
   {
      ROS_INFO("Subscribing...");
      this->sub_image_ = pnh_->subscribe(
         "input", 1, &SlidingWindowObjectDetector::imageCb, this);
      ros::V_string names = boost::assign::list_of("~input");
      jsk_topic_tools::warnNoRemap(names);
   }
   
   void SlidingWindowObjectDetector::unsubscribe()
   {
      NODELET_DEBUG("Unsubscribing from ROS topic.");
      this->sub_image_.shutdown();
   }
   
   void SlidingWindowObjectDetector::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
      std::multimap<float, cv::Rect_<int> > detection_info =
         this->runSlidingWindowDetector(image, cv::Size(
                                      this->swindow_x, this->swindow_y),
                                   scale, img_stack, incrementor);
      cv::Mat dimg = image.clone();
      ROS_INFO("--Info Size: %ld", detection_info.size());
      for (std::multimap<float, cv::Rect_<int> >::iterator
              it = detection_info.begin(); it != detection_info.end(); it++) {
         cv::rectangle(dimg, it->second, cv::Scalar(0, 0, 255), 2);
      }
      if (this->run_type_.compare("DETECTOR") == 0) {
         const float nms_threshold = 0.01;
         std::vector<cv::Rect_<int> > object_rects = this->nonMaximumSuppression(
            detection_info, nms_threshold);
         cv::Mat bimg = image.clone();
         for (std::vector<cv::Rect_<int> >::iterator it = object_rects.begin();
              it != object_rects.end(); it++) {
            this->setBoundingBoxLabel(bimg, *it);
            cv::rectangle(bimg, *it, cv::Scalar(0, 255, 0), 1);
         }
         jsk_recognition_msgs::RectArray jsk_rect_array;
         this->convertCvRectToJSKRectArray(
            object_rects, jsk_rect_array, downsize, isize);
         jsk_rect_array.header = msg->header;
         cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);
         out_msg->header = msg->header;
         out_msg->encoding = sensor_msgs::image_encodings::BGR8;
         out_msg->image = bimg.clone();
         this->pub_rects_.publish(jsk_rect_array);
         this->pub_image_.publish(out_msg->toImageMsg());
      } else if (this->run_type_.compare("BOOTSTRAPER") == 0) {
         for (std::multimap<float, cv::Rect_<int> >::const_iterator
                 it = detection_info.begin(); it != detection_info.end(); it++) {
            cv::Mat roi = image(it->second).clone();
            cv::resize(
               roi, roi, cv::Size(
                  roi.cols * this->downsize_, roi.rows * this->downsize_));
            if (roi.data) {
               ROS_INFO("Writing to bag file");
               cv_bridge::CvImagePtr write_roi(new cv_bridge::CvImage);
               write_roi->header = msg->header;
               write_roi->encoding = sensor_msgs::image_encodings::BGR8;
               write_roi->image = roi.clone();
               this->rosbag_->write("/dataset/background/roi",
                                    ros::Time::now(), write_roi->toImageMsg());
               cv::imshow("write_roi", roi);
               cv::waitKey(3);
            }
         }
      } else {
         this->pub_image_.publish(cv_ptr->toImageMsg());
         ROS_ERROR("NODELET RUNTYPE IS NOT SET.");
         std::_Exit(EXIT_FAILURE);
      }
   }

   std::multimap<float, cv::Rect_<int> >
   SlidingWindowObjectDetector::runSlidingWindowDetector(
      const cv::Mat &image, const cv::Size wsize, const float scale,
      const int scale_counter, const int incrementor)
   {
      if (image.empty()) {
         ROS_ERROR("--INPUT IMAGE IS EMPTY");
         return std::multimap<float, cv::Rect_<int> >();
      }
      cv::Size nwsize = wsize;
      int scounter = 0;
      std::multimap<float, cv::Rect_<int> > detection_info;
      int sw_incrementor = incrementor;
      while (scounter++ < scale_counter) {
         this->objectRecognizer(image, detection_info, nwsize, sw_incrementor);
         this->pyramidialScaling(nwsize, scale);
         sw_incrementor += (sw_incrementor * scale);
      }
      return detection_info;
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
               cv::Mat hog_feature = this->computeHOG(roi);
               cv::Mat hsv_feature;
               this->computeHSHistogram(roi, hsv_feature, 16, 16, true);
               hsv_feature = hsv_feature.reshape(1, 1);
               cv::Mat _feature = hog_feature;
               this->concatenateCVMat(hog_feature, hsv_feature, _feature);
#if CV_MAJOR_VERSION >= 3
               cv::Mat _ret;
               float response = this->supportVectorMachine_->predict(
                                                                     _feature, _ret, false);
#else
               float response = this->supportVectorMachine_->predict(
                  _feature, false);
#endif
               if (response == 1) {
                  detection_info.insert(std::make_pair(response, rect));
               } else {
                  continue;
               }
            }
         }
      }
   }

   /**
    * color histogram temp placed here
    */
   void SlidingWindowObjectDetector::computeHSHistogram(
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
   
   void SlidingWindowObjectDetector::loadTrainedDetectorModel()
   {
      try {
         ROS_INFO("--Loading Trained SVM Classifier");
#if CV_MAJOR_VERSION >= 3 // http://docs.opencv.org/master/d3/d46/classcv_1_1Algorithm.html
         this->supportVectorMachine_ = cv::ml::SVM::create();
         this->supportVectorMachine_ = cv::Algorithm::load<cv::ml::SVM>(this->model_name_);
#else
         this->supportVectorMachine_ = boost::shared_ptr<cv::SVM>(new cv::SVM);
         this->supportVectorMachine_->load(this->model_name_.c_str());
#endif
         ROS_INFO("--Classifier Loaded Successfully");
      } catch(cv::Exception &e) {
         ROS_ERROR("--ERROR: Fail to load Classifier \n%s", e.what());
         std::_Exit(EXIT_FAILURE);
      }
   }
   
   void SlidingWindowObjectDetector::readTrainingManifestFromDirectory()
   {
      cv::FileStorage fs = cv::FileStorage(
         this->trainer_manifest_filename_, cv::FileStorage::READ);
      if (!fs.isOpened()) {
         ROS_ERROR("TRAINER MANIFEST NOT FOUND..");
         std::_Exit(EXIT_FAILURE);
      }
      cv::FileNode n = fs["TrainerInfo"];
      std::string ttype = n["trainer_type"];
      std::string tpath = n["trainer_path"];

      n = fs["FeatureInfo"];       // features used
      int hog = static_cast<int>(n["HOG"]);
      int lbp = static_cast<int>(n["LBP"]);

      n = fs["SlidingWindowInfo"];  // window size
      int sw_x = static_cast<int>(n["swindow_x"]);
      int sw_y = static_cast<int>(n["swindow_y"]);

      n = fs["TrainingDatasetDirectoryInfo"];
      std::string pfile = n["object_dataset_filename"];
      std::string nfile = n["nonobject_dataset_filename"];
      std::string dataset_path = n["dataset_path"];

      if (this->override_manifest_)
      {
        pnh_->param<std::string>("trainer_path", tpath, tpath);
        pnh_->param<int>("swindow_x", sw_x, sw_x);
        pnh_->param<int>("swindow_y", sw_y, sw_y);
        pnh_->param<std::string>("object_dataset_filename", pfile, pfile);
        pnh_->param<std::string>("nonobject_dataset_filename", nfile, nfile);
        pnh_->param<std::string>("dataset_path", dataset_path, dataset_path);
      }

      this->model_name_ = tpath;   // classifier path
      this->swindow_x = sw_x;
      this->swindow_y = sw_y;
      this->object_dataset_filename_ = pfile;  // object dataset
      this->nonobject_dataset_filename_ = nfile;
      this->ndataset_path_ = dataset_path + nfile;  // ~/.ros/dir/negative/x.bag
}

   void SlidingWindowObjectDetector::setBoundingBoxLabel(
      cv::Mat& im, cv::Rect_<int> rect, const std::string label)
   {
      int fontface = cv::FONT_HERSHEY_SIMPLEX;
      double scale = 0.2;
      int thickness = 1;
      int baseline = 0;
      cv::Size text = cv::getTextSize(
         label, fontface, scale, thickness, &baseline);
      cv::Point pt = cv::Point(
         rect.x + (rect.width-text.width), rect.y + (rect.height+text.height));
      cv::rectangle(im, pt + cv::Point(0, baseline),
                    pt + cv::Point(text.width, -text.height),
                    CV_RGB(0, 0, 255), CV_FILLED);
      cv::putText(im, label, pt,
                  fontface, scale, CV_RGB(255, 0, 0), thickness, 8);
   }
   
   void SlidingWindowObjectDetector::configCallback(
      jsk_perception::SlidingWindowObjectDetectorConfig &config, uint32_t level)
   {
      boost::mutex::scoped_lock lock(mutex_);
      this->scale_ = static_cast<float>(config.scaling_factor);
      this->stack_size_ = static_cast<int>(config.stack_size);
      this->incrementor_ = config.sliding_window_increment;
      this->downsize_ = config.image_downsize;
      
      // currently fixed variables
      // this->swindow_x = config.sliding_window_width;
      // this->swindow_y = config.sliding_window_height;
   }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::SlidingWindowObjectDetector, nodelet::Nodelet);
