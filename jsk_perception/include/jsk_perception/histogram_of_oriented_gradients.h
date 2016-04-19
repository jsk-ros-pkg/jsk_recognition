// Copyright (C) 2016 by Krishneel Chaudhary @ JSK Lab, The University
// of Tokyo

#ifndef _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_
#define _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

class HOGFeatureDescriptor {

 protected:
    int N_BINS;
    int ANGLE;
    int BINS_ANGLE;
    int CELL;
    int BLOCK;
    int num_threads_;
    
 private:
    virtual void histogramBinVoting(
	const float &, int &, int &);
    virtual void imageGradient(
	const cv::Mat &, cv::Mat &);
    virtual cv::Mat blockGradient(
        const int, const int, const int, cv::Mat &);
    virtual void getHOG(
	const cv::Mat &, cv::Mat &, cv::Mat &);
   
 public:
    HOGFeatureDescriptor(
        const int = 8, const int = 2, const int = 9,
        const float = 180.0f, const int = 1);
    virtual cv::Mat computeHOG(const cv::Mat &);
    template<typename T>
    T computeHOGHistogramDistances(
        const cv::Mat &, std::vector<cv::Mat> &,
        const int = CV_COMP_BHATTACHARYYA);
    cv::Mat visualizeHOG(
        cv::Mat &, cv::Mat &, cv::Size, int, double);
};
#endif  // _HISTOGRAM_OF_ORIENTED_GRADIENTS_H_
