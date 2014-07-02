// @file oriented_gradient.hpp
// @brief calc oriented gradient
// @author Hiroaki Yaguchi, JSK

#ifndef JSK_PERCEPTION_ORIENTEDGRADIENT_HPP_
#define JSK_PERCEPTION_ORIENTEDGRADIENT_HPP_

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace jsk_perception {

void calcOrientedGradient(cv::Mat& src, cv::Mat& dst);
void calcOGKeyPoints(cv::Mat& src,
                     cv::Mat& dst,
                     std::vector<cv::Point>& result,
                     int thres = 32,
                     int bs = 1);
void calcScaledOrientedGradient(cv::Mat& src, cv::Mat& dst, int scale);
void calcSOGKeyPoints(cv::Mat& src, cv::Mat& dst);

}

#endif  // JSK_PERCEPTION_ORIENTEDGRADIENT_HPP_

