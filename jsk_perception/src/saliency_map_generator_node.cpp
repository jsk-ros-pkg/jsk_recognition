
#include <jsk_perception/saliency_map_generator.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif

namespace jsk_perception
{
   void SaliencyMapGenerator::onInit()
   {
      DiagnosticNodelet::onInit();
      pnh_->getParam("num_threads", this->num_threads_);
      pnh_->getParam("fps", this->print_fps_);
      this->pub_image_ = advertise<sensor_msgs::Image>(*pnh_,
         "/saliency_map_generator/output/saliency_map", 1);
      onInitPostProcess();
   }

   void SaliencyMapGenerator::subscribe()
   {
      this->sub_image_ = pnh_->subscribe(
         "input", 1, &SaliencyMapGenerator::callback, this);
   }

   void SaliencyMapGenerator::unsubscribe()
   {
      NODELET_DEBUG("Unsubscribing from ROS topic.");
      this->sub_image_.shutdown();
   }
   
   void SaliencyMapGenerator::callback(
      const sensor_msgs::Image::ConstPtr &image_msg)
   {      
      cv::Mat image = cv_bridge::toCvShare(
         image_msg, image_msg->encoding)->image;
      if (image.empty()) {
         return;
      }
      if (image.channels() == 3) {
         cv::cvtColor(image, image, CV_BGR2GRAY);
      }
      if (this->counter_ == 0) {
         this->start_ = omp_get_wtime();
      }
      
      cv::Mat saliency_map;
      this->computeSaliencyImpl(image, saliency_map);
      cv::cvtColor(saliency_map, saliency_map, CV_GRAY2BGR);

      if (this->print_fps_) {
         this->counter_++;
         double sec = (omp_get_wtime() - this->start_) /
            static_cast<double>(this->num_threads_);
         double fps = static_cast<double>(this->counter_) / sec;
         fps = ceil(fps * 100) / 100;      
         if (this->counter_ == (INT_MAX)) {
            this->counter_ = 0;
         }
         cv::putText(saliency_map, "FPS: " + boost::lexical_cast<std::string>(fps),
                     cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
                     cv::Scalar(200, 200, 250), 1, CV_AA);
      }
      cv_bridge::CvImage pub_img(image_msg->header,
                                 image_msg->encoding,
                                 saliency_map);
      this->pub_image_.publish(pub_img.toImageMsg());
   }

   bool SaliencyMapGenerator::computeSaliencyImpl(
      cv::Mat image, cv::Mat &saliencyMap)
   {
      if (image.empty()) {
         return false;
      }
      cv::Mat dst(cv::Size(image.cols, image.rows), CV_8UC1);
      calcIntensityChannel(image, dst);
      saliencyMap = cv::Mat::zeros(image.size(), CV_8UC1);
      dst.copyTo(saliencyMap);
      return true;
   }

   void SaliencyMapGenerator::setNumThreads(int num_threads)
   {
      this->num_threads_ = num_threads;
   }

   void SaliencyMapGenerator::copyImage(cv::Mat srcArg, cv::Mat dstArg)
   {
      srcArg.copyTo(dstArg);
   }

   void SaliencyMapGenerator::calcIntensityChannel(
      cv::Mat srcArg, cv::Mat dstArg)
   {
      if (dstArg.channels() > 1) {
         return;
      }
      const int numScales = 6;
      cv::Mat intensityScaledOn[numScales];
      cv::Mat intensityScaledOff[numScales];
      cv::Mat gray = cv::Mat::zeros(cv::Size(srcArg.cols, srcArg.rows), CV_8UC1);
      cv::Mat integralImage(cv::Size(srcArg.cols + 1, srcArg.rows + 1), CV_32FC1);
      cv::Mat intensity(cv::Size(srcArg.cols, srcArg.rows), CV_8UC1);
      cv::Mat intensityOn(cv::Size(srcArg.cols, srcArg.rows), CV_8UC1);
      cv::Mat intensityOff(cv::Size(srcArg.cols, srcArg.rows), CV_8UC1);

      int i;
      int neighborhood;
      int neighborhoods[] = {3*4, 3*4*2, 3*4*2*2, 7*4, 7*4*2, 7*4*2*2};

      for (i = 0; i < numScales; i++) {
         intensityScaledOn[i] = cv::Mat(cv::Size(
                                           srcArg.cols,
                                           srcArg.rows), CV_8UC1);
         intensityScaledOff[i] = cv::Mat(cv::Size(
                                            srcArg.cols, srcArg.rows), CV_8UC1);
      }
      if (srcArg.channels() == 3) {
         cv::cvtColor(srcArg, gray, cv::COLOR_BGR2GRAY);
      } else {
         srcArg.copyTo(gray);
      }
      cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0);
      cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0, 0);
      cv::integral(gray, integralImage, CV_32F);
      for (i=0; i < numScales; i++) {
         neighborhood = neighborhoods[i];
         getIntensityScaled(integralImage, gray,
                            intensityScaledOn[i],
                            intensityScaledOff[i],
                            neighborhood);
      }
      mixScales(intensityScaledOn, intensityOn,
                intensityScaledOff,
                intensityOff,
                numScales);
      mixOnOff(intensityOn, intensityOff, intensity);
      intensity.copyTo(dstArg);
   }

   void SaliencyMapGenerator::getIntensityScaled(
      cv::Mat integralImage, cv::Mat gray, cv::Mat intensityScaledOn,
      cv::Mat intensityScaledOff, int neighborhood)
   {
      float value, meanOn, meanOff;
      intensityScaledOn.setTo(cv::Scalar::all(0));
      intensityScaledOff.setTo(cv::Scalar::all(0));
#ifdef _OPENMP
#pragma omp parallel for collapse(2)            \
   private(value, meanOn, meanOff)              \
   num_threads(num_threads_)
#endif
      for (int y = 0; y < gray.rows; y++) {
         for (int x = 0; x < gray.cols; x++) {
            cv::Point2i point;
            point.x = x;
            point.y = y;
            value = getMean(integralImage,
                            point, neighborhood, gray.at<uchar>(y, x));
            meanOn = gray.at<uchar>(y, x) - value;
            meanOff = value - gray.at<uchar>(y, x);
            if (meanOn > 0)
               intensityScaledOn.at<uchar>(y, x) = (uchar)meanOn;
            else
               intensityScaledOn.at<uchar>(y, x) = 0;

            if (meanOff > 0)
               intensityScaledOff.at<uchar>(y, x) = (uchar)meanOff;
            else
               intensityScaledOff.at<uchar>(y, x) = 0;
         }
      }
   }

   float SaliencyMapGenerator::getMean(
      cv::Mat srcArg, cv::Point2i PixArg, int neighbourhood, int centerVal)
   {
      cv::Point2i P1, P2;
      float value;

      P1.x = PixArg.x - neighbourhood + 1;
      P1.y = PixArg.y - neighbourhood + 1;
      P2.x = PixArg.x + neighbourhood + 1;
      P2.y = PixArg.y + neighbourhood + 1;

      if (P1.x < 0)
         P1.x = 0;
      else if (P1.x > srcArg.cols - 1)
         P1.x = srcArg.cols - 1;
      if (P2.x < 0)
         P2.x = 0;
      else if (P2.x > srcArg.cols - 1)
         P2.x = srcArg.cols - 1;
      if (P1.y < 0)
         P1.y = 0;
      else if (P1.y > srcArg.rows - 1)
         P1.y = srcArg.rows - 1;
      if (P2.y < 0)
         P2.y = 0;
      else if (P2.y > srcArg.rows - 1)
         P2.y = srcArg.rows - 1;

      // we use the integral image to compute fast features
      value = static_cast<float> (
         (srcArg.at<float>(P2.y, P2.x)) +
         (srcArg.at<float>(P1.y, P1.x)) -
         (srcArg.at<float>(P2.y, P1.x)) -
         (srcArg.at<float>(P1.y, P2.x)));
      value = (value - centerVal)/  (( (P2.x - P1.x) * (P2.y - P1.y))-1);
      return value;
   }

   void SaliencyMapGenerator::mixScales(
      cv::Mat *intensityScaledOn, cv::Mat intensityOn,
      cv::Mat *intensityScaledOff, cv::Mat intensityOff, const int numScales)
   {
      int i = 0;
      int width = intensityScaledOn[0].cols;
      int height = intensityScaledOn[0].rows;
      short int maxValOn = 0, currValOn = 0;
      short int maxValOff = 0, currValOff = 0;
      int maxValSumOff = 0, maxValSumOn = 0;
      cv::Mat mixedValuesOn(cv::Size(width, height), CV_16UC1);
      cv::Mat mixedValuesOff(cv::Size(width, height), CV_16UC1);
      mixedValuesOn.setTo(cv::Scalar::all(0));
      mixedValuesOff.setTo(cv::Scalar::all(0));
#ifdef _OPENMP
#pragma omp parallel for collapse(3)                                    \
   private(i, maxValOn, currValOn, maxValOff, currValOff, maxValSumOn, maxValSumOff) \
   num_threads(num_threads_) 
#endif
      for (i = 0; i < numScales; i++) {
         for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
               currValOn = intensityScaledOn[i].at<uchar>(y, x);
               if (currValOn > maxValOn)
                  maxValOn = currValOn;

               currValOff = intensityScaledOff[i].at<uchar>(y, x);
               if (currValOff > maxValOff)
                  maxValOff = currValOff;
               mixedValuesOn.at<unsigned short>(y, x) += currValOn;
               mixedValuesOff.at<unsigned short>(y, x) += currValOff;
            }
      }
#ifdef _OPENMP
#pragma omp parallel for collapse(2)                        \
   shared(currValOn, currValOff, maxValSumOn, maxValSumOff) \
   num_threads(num_threads_) 
#endif
      for (int y = 0; y < height; y++)
         for (int x = 0; x < width; x++) {
            currValOn = mixedValuesOn.at<unsigned short>(y, x);
            currValOff = mixedValuesOff.at<unsigned short>(y, x);
            if (currValOff > maxValSumOff)
               maxValSumOff = currValOff;
            if (currValOn > maxValSumOn)
               maxValSumOn = currValOn;
         }
#ifdef _OPENMP
#pragma omp parallel for collapse(2) num_threads(num_threads_) 
#endif
      for (int y = 0; y < height; y++)
         for (int x = 0; x < width; x++) {
            intensityOn.at<uchar>(y, x) = (uchar)(
               255.*((float)(mixedValuesOn.at<unsigned short>(y, x) / (float)maxValSumOn)));
            intensityOff.at<uchar>(y, x) = (uchar)(
               255.*((float)(mixedValuesOff.at<unsigned short>(y, x) / (float)maxValSumOff)));
         }

   }

   void SaliencyMapGenerator::mixOnOff(
      cv::Mat intensityOn, cv::Mat intensityOff, cv::Mat intensityArg)
   {
      int width = intensityOn.cols;
      int height = intensityOn.rows;
      int maxVal = 0;
      int currValOn, currValOff, maxValSumOff, maxValSumOn;
      cv::Mat intensity(cv::Size(width, height), CV_8UC1);
      maxValSumOff = 0;
      maxValSumOn = 0;
#ifdef _OPENMP
#pragma omp parallel for collapse(2)                                \
   shared(currValOn, currValOff, maxValSumOn, maxValSumOff, maxVal) \
   num_threads(num_threads_) 
#endif
      for (int y = 0; y < height; y++) {
         for (int x = 0; x < width; x++) {
            currValOn = intensityOn.at<uchar>(y, x);
            currValOff = intensityOff.at<uchar>(y, x);
            if (currValOff > maxValSumOff) {
               maxValSumOff = currValOff;
            }
            if (currValOn > maxValSumOn) {
               maxValSumOn = currValOn;
            }
         }
      }
      if (maxValSumOn > maxValSumOff) {
         maxVal = maxValSumOn;
      } else {
         maxVal = maxValSumOff;
      }
#ifdef _OPENMP
#pragma omp parallel for collapse(2) num_threads(num_threads_) 
#endif
      for (int y = 0; y < height; y++) {
         for (int x = 0; x < width; x++) {
            intensity.at<uchar>(y, x) = (uchar) (
               255. * (float) (intensityOn.at<uchar>(y, x) +
                               intensityOff.at<uchar>(y, x)) / (float)maxVal);
         }
      }
      intensity.copyTo(intensityArg);
   }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::SaliencyMapGenerator, nodelet::Nodelet);
