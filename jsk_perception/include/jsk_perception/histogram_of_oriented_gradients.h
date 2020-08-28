
#ifndef _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_
#define _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_

// OpenCV Header Directives
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/types_c.h>
#endif

#include <string>
#include <vector>

class HOGFeatureDescriptor {

    //  HOG Configuration Params
 protected:
    int N_BINS;
    int ANGLE;
    int BINS_ANGLE;
    int CELL;
    int BLOCK;

 private:
    virtual void bilinearBinVoting(
       const float &, int &, int &);
    virtual void imageGradient(
       const cv::Mat &, cv::Mat &);
    virtual cv::Mat blockGradient(
       const int, const int, cv::Mat &);
    virtual void getHOG(
       const cv::Mat &, cv::Mat &, cv::Mat &);
    template<typename T>
    T computeHOGHistogramDistances(
       const cv::Mat &, std::vector<cv::Mat> &,
       const int = CV_COMP_BHATTACHARYYA);
   
 public:
    HOGFeatureDescriptor(
       const int = 8, const int = 2, const int = 9,
       const float = 180.0f);
    virtual cv::Mat computeHOG(
       const cv::Mat &);
};

#endif  // _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_
