// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 *
 *********************************************************************/


#ifndef JSK_PERCEPTION_DARKNET_OBJECT_DETECTION_H__
#define JSK_PERCEPTION_DARKNET_OBJECT_DETECTION_H__

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <jsk_recognition_msgs/LabeledRectArray.h>

extern "C" {
#ifdef __cplusplus
#define ___cplusplus __cplusplus
#undef __cplusplus
#endif
#include <darknet/box.h>
#include <darknet/network.h>
#include <darknet/cost_layer.h>
#include <darknet/region_layer.h>
#include <darknet/image.h>
#include <darknet/parser.h>
#include <darknet/utils.h>
#ifdef ___cplusplus
#define __cplusplus ___cplusplus
#undef ___cplusplus
#endif
}

extern "C" image ipl_to_image(IplImage* src);


namespace jsk_perception
{

class AverageArray {
  float ** arr;
  bool valid;
  int n, e, c;
public:
  AverageArray(int n, int e) : n(n), e(e), c(0), valid(false) {
    arr = (float**)calloc(n, sizeof(float*));
    for (int i = 0; i < n; ++i) arr[i] = (float*)calloc(e, sizeof(float));
  }
  ~AverageArray() {
    for (int i = 0; i < n; ++i) free(arr[i]);
    free(arr);
  }
  void add(float *in) {
    std::memcpy(arr[c], in, e * sizeof(float));
    ++c;
    if (c >= n) c = 0;
    if (c == n) valid = true;
  }
  bool canAverage() { return valid; }
  void averageArray(float* ret) {
    for (int j = 0; j < e; ++j) {
      float v = 0.0f;
      for (int i = 0; i < n; ++i) v += arr[i][j];
      ret[j] = v / n;
    }
  }
};

class DarknetObjectDetection : public jsk_topic_tools::ConnectionBasedNodelet
{
public:
  std::vector<std::string> labels_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<ros::NodeHandle> nh_, pnh_;
  image_transport::Subscriber sub_image_;
  ros::Publisher pub_rects_;

  // darknet
  network network_;
  int max_box_num_;
  int mean_;
  boost::shared_ptr<AverageArray> average_pred_;
  float detection_threshold_;
  float nms_;
  box* boxes_;
  float** probs_;

  virtual ~DarknetObjectDetection();

  virtual void onInit();
  virtual void subscribe();
  virtual void unsubscribe();
  virtual void imageCb(const sensor_msgs::Image::ConstPtr &msg);

  virtual void processResult(image* dn_img, box* boxes, float** probs, jsk_recognition_msgs::LabeledRectArray &msg);
}; // class DarknetObjectDetection
} // namespace jsk_perception


#endif // JSK_PERCEPTION_DARKNET_OBJECT_DETECTION_H__
