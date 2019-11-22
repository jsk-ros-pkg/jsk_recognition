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


#ifndef JSK_PCL_ROS_HEIGHTMAP_UTILS_H_
#define JSK_PCL_ROS_HEIGHTMAP_UTILS_H_

#include <jsk_recognition_msgs/HeightmapConfig.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  typedef pcl::PointXYZI HeightMapPointType;

  inline std::string getHeightmapConfigTopic(const std::string& base_topic)
  {
    return base_topic + "/config";
  }

  inline void convertHeightMapToPCL(const cv::Mat &float_image,
                                   pcl::PointCloud<HeightMapPointType > &cloud,
                                   float max_x_, float min_x_, float max_y_, float min_y_)
  {
    int height = float_image.rows;
    int width  = float_image.cols;
    cloud.points.reserve(float_image.rows * float_image.cols); // normal
    double dx = (max_x_ - min_x_) / width;
    double dy = (max_y_ - min_y_) / height;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        float v = float_image.at<cv::Vec2f>(j, i)[0];
        float fint = float_image.at<cv::Vec2f>(j, i)[1];
        if (v != -FLT_MAX) {
          HeightMapPointType p;
          p.y = j * dy + min_y_ + dy / 2.0;
          p.x = i * dx + min_x_ + dx / 2.0;
          p.z = v;
          p.intensity = fint;
          cloud.points.push_back(p); // normal
        }
      }
    }
  }

  inline void convertHeightMapToPCLOrganize(const cv::Mat &float_image,
                                           pcl::PointCloud<HeightMapPointType > &cloud,
                                           float max_x_, float min_x_, float max_y_, float min_y_)
  {
    int height = float_image.rows;
    int width  = float_image.cols;
    cloud.points.resize(height * width); // organized
    cloud.width  = width;
    cloud.height = height;
    double dx = (max_x_ - min_x_) / width;
    double dy = (max_y_ - min_y_) / height;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        float v = float_image.at<cv::Vec2f>(j, i)[0];
        float fint = float_image.at<cv::Vec2f>(j, i)[1];
        HeightMapPointType p;
        p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN(); // organized
        if (v != -FLT_MAX) {
          p.y = j * dy + min_y_ + dy / 2.0;
          p.x = i * dx + min_x_ + dx / 2.0;
          p.z = v;
          p.intensity = fint;
        }
        cloud.points[(j * width) + i] = p; // organized
      }
    }
  }
}

#endif
