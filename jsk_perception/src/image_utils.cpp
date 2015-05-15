// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
 *********************************************************************/

#include "jsk_perception/image_utils.h"
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception
{
  cv::Rect boundingRectOfMaskImage(const cv::Mat& image)
  {
    int min_x = image.cols;
    int min_y = image.rows;
    int max_x = 0;
    int max_y = 0;
    for (int j = 0; j < image.rows; j++) {
      for (int i = 0; i < image.cols; i++) {
        if (image.at<uchar>(j, i) != 0) {
          min_x = std::min(min_x, i);
          min_y = std::min(min_y, j);
          max_x = std::max(max_x, i);
          max_y = std::max(max_y, j);
        }
      }
    }
    
    return cv::Rect(min_x, min_y, std::max(max_x - min_x, 0), std::max(max_y - min_y, 0));
  }

  // Utility functions for inspecting an encoding string
  bool isBGR(const std::string& encoding)
  {
    return encoding == enc::BGR8 || encoding == enc::BGR16;
  }

  bool isRGB(const std::string& encoding)
  {
    return encoding == enc::RGB8 || encoding == enc::RGB16;
  }

  bool isBGRA(const std::string& encoding)
  {
    return encoding == enc::BGRA8 || encoding == enc::BGRA16;
  }

  bool isRGBA(const std::string& encoding)
  {
    return encoding == enc::RGBA8 || encoding == enc::RGBA16;
  }

}
