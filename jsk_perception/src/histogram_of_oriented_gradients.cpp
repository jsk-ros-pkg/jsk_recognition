
#include <jsk_perception/histogram_of_oriented_gradients.h>

HOGFeatureDescriptor::HOGFeatureDescriptor(
    const int cell_size, const int block_per_cell,
    const int n_bins, const float angle) :
  CELL(cell_size),
  BLOCK(block_per_cell),
  ANGLE(angle),
  N_BINS(n_bins) {
  this->BINS_ANGLE = this->ANGLE / this->N_BINS;
}

void HOGFeatureDescriptor::bilinearBinVoting(
    const float &angle, int &lower_index, int &higher_index) {
    float nearest_lower = FLT_MAX;
    float nearest_higher = FLT_MAX;
    lower_index = 0;
    higher_index = 0;
    for (int i = BINS_ANGLE/2; i < ANGLE; i += BINS_ANGLE) {
       float distance = abs(angle - i);
        if (i < angle) {
            if (distance < nearest_lower) {
                nearest_lower = distance;
                lower_index = i;
            }
        } else {
            if (distance < nearest_higher) {
                nearest_higher = distance;
                higher_index = i;
            }
        }
    }
}

void HOGFeatureDescriptor::imageGradient(
    const cv::Mat &image, cv::Mat &hog_bins) {
    cv::Mat xsobel;
    cv::Mat ysobel;
    cv::Sobel(image, xsobel, CV_32F, 1, 0, 7);
    cv::Sobel(image, ysobel, CV_32F, 0, 1, 7);
    cv::Mat Imag;
    cv::Mat Iang;
    cv::cartToPolar(xsobel, ysobel, Imag, Iang, true);
    cv::add(Iang, cv::Scalar(ANGLE), Iang, Iang < 0);
    cv::add(Iang, cv::Scalar(-ANGLE), Iang, Iang >= ANGLE);
    cv::Mat orientation_histogram;
    for (int j = 0; j < image.rows; j += CELL) {
        for (int i = 0; i < image.cols; i += CELL) {
           cv::Rect rect = cv::Rect(i, j, CELL, CELL);
           if ((rect.x + rect.width <= image.cols) &&
               (rect.y + rect.height <= image.rows)) {
              cv::Mat bin = cv::Mat::zeros(1, N_BINS, CV_32F);
                for (int y = rect.y; y < (rect.y + rect.height); y++) {
                    for (int x = rect.x; x < (rect.x + rect.width); x++) {
                       float angle = static_cast<float>(Iang.at<float>(y, x));
                       int l_bin;
                       int h_bin;
                       this->bilinearBinVoting(angle, l_bin, h_bin);
                       float l_ratio = 1 - (angle - l_bin)/BINS_ANGLE;
                       float h_ratio = 1 + (angle - h_bin)/BINS_ANGLE;
                       int l_index = (l_bin-(BINS_ANGLE/2))/BINS_ANGLE;
                       int h_index = (h_bin-(BINS_ANGLE/2))/BINS_ANGLE;
                        bin.at<float>(0, l_index) +=
                           (Imag.at<float>(y, x) * l_ratio);
                        bin.at<float>(0, h_index) +=
                           (Imag.at<float>(y, x) * h_ratio);
                    }
                }
                orientation_histogram.push_back(bin);
            }
        }
    }
    hog_bins = orientation_histogram.clone();
}

cv::Mat HOGFeatureDescriptor::blockGradient(
    const int col, const int row, cv::Mat &bins) {
    cv::Mat block_hogMD = cv::Mat(cv::Size(N_BINS * BLOCK * BLOCK, 1), CV_32F);
    int icounter = 0;
    for (int j = row; j < (row + BLOCK); j++) {
       for (int i = col; i < (col + BLOCK); i++) {
          int index = (j * CELL) + i;
          for (int k = 0; k < N_BINS; k++) {
             block_hogMD.at<float>(0, icounter++) = bins.at<float>(index);
          }
        }
    }
    return block_hogMD;
}

void HOGFeatureDescriptor::getHOG(
    const cv::Mat &image, cv::Mat &bins, cv::Mat &featureMD) {
    for (int j = 0; j < image.rows; j += CELL) {
       for (int i = 0; i < image.cols; i += CELL) {
          cv::Rect rect = cv::Rect(i, j, CELL*BLOCK, CELL*BLOCK);
            if ((rect.x + rect.width <= image.cols) &&
                (rect.y + rect.height <= image.rows)) {
               cv::Mat hogMD = this->blockGradient(rect.x, rect.y, bins);
               normalize(hogMD, hogMD, 1, 0, CV_L2);
               featureMD.push_back(hogMD);
            }
        }
    }
}

cv::Mat HOGFeatureDescriptor::computeHOG(
    const cv::Mat &img) {
    cv::Mat image = img.clone();
    if (image.type() != CV_8U) {
       cv::cvtColor(image, image, CV_BGR2GRAY);
    }
    cv::Mat bins;
    this->imageGradient(image, bins);
    cv::Mat featureMD;
    getHOG(image, bins, featureMD);
    featureMD = featureMD.reshape(1, 1);
    return featureMD;
}

template<typename T>
T HOGFeatureDescriptor::computeHOGHistogramDistances(
    const cv::Mat &patch, std::vector<cv::Mat> &imageHOG,
    const int distance_type) {
    T sum = 0.0;
    T argMinDistance = FLT_MAX;
    for (int i = 0; i < imageHOG.size(); i++) {
       cv::Mat img_hog = imageHOG[i];
       T d = compareHist(patch, img_hog, distance_type);
        if (d < argMinDistance) {
            argMinDistance = static_cast<double>(d);
        }
    }
    sum = static_cast<T>(argMinDistance);
    return static_cast<T>(sum);
}
