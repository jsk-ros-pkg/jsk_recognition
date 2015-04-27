
#ifndef _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_
#define _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_

// OpenCV Header Directives
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>

class HOGFeatureDescriptor {

    //  HOG Configuration Params
#define N_BINS 9
#define ANGLE 180.0
#define BINS_ANGLE (ANGLE / N_BINS)
#define CELL 8
#define BLOCK 2

 private:
    virtual void bilinearBinVoting(
       const float &, int &, int &);
    virtual void imageGradient(
       const cv::Mat &, cv::Mat &);
    virtual cv::Mat blockGradient(
       const int, const int, cv::Mat &);
    virtual cv::Mat orientationistogram(
       const cv::Mat&, const int &, const int &, bool = false);
    virtual void getHOG(
       const cv::Mat &, cv::Mat &, cv::Mat &);
    template<typename T>
    T computeHOGHistogramDistances(
       const cv::Mat &, std::vector<cv::Mat> &,
       const int = CV_COMP_BHATTACHARYYA);
   
 public:
    HOGFeatureDescriptor();
    virtual cv::Mat computeHOG(
       const cv::Mat &);
};

#endif  // _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_
