// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#ifndef JSK_PCL_ROS_PCL_CONVERSION_UTIL_H_
#define JSK_PCL_ROS_PCL_CONVERSION_UTIL_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point32.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/range_image/range_image_planar.h>
#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
typedef pcl_msgs::ModelCoefficients PCLModelCoefficientMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
typedef pcl::ModelCoefficients PCLModelCoefficientMsg;
#endif

#include <opencv2/opencv.hpp>

namespace jsk_pcl_ros
{

  /** @brief
   * Convert pcl::RangeImage to cv::Mat. Distance is normalized
   * to 0-1 and colorized.
   *
   * @param range_image instance of pcl::RangeImage
   * @param mat instance of cv::Mat, converted cv::Mat is set into
   *        this argument.
   */
  void rangeImageToCvMat(const pcl::RangeImage& range_image,
                         cv::Mat& mat);
  
  template<class FromT, class ToT>
  void pointFromXYZToVector(const FromT& msg,
                            ToT& p)
  {
    p[0] = msg.x; p[1] = msg.y; p[2] = msg.z;
  }

  template<class FromT, class ToT>
  void pointFromVectorToXYZ(const FromT& p,
                            ToT& msg)
  {
    msg.x = p[0]; msg.y = p[1]; msg.z = p[2];
  }

  template<class FromT, class ToT>
  void pointFromXYZToXYZ(const FromT& from,
                         ToT& to)
  {
    to.x = from.x; to.y = from.y; to.z = from.z;
  }

  template<class FromT, class ToT>
  void pointFromVectorToVector(const FromT& from,
                               ToT& to)
  {
    to[0] = from[0]; to[1] = from[1]; to[2] = from[2];
  }

  template<class FromT, class ToT>
  void convertMatrix4(const FromT& from,
                      ToT& to)
  {
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j++) {
        to(i, j) = from(i, j);
      }
    }
  }

  void convertEigenAffine3(const Eigen::Affine3d& from,
                           Eigen::Affine3f& to);
  void convertEigenAffine3(const Eigen::Affine3f& from,
                           Eigen::Affine3d& to);

}
// extend pcl_conversions package's toPCL and fromPCL functions
namespace pcl_conversions
{  
  std::vector<pcl::PointIndices::Ptr>
  convertToPCLPointIndices(const std::vector<PCLIndicesMsg>& cluster_indices);

  std::vector<pcl::ModelCoefficients::Ptr>
  convertToPCLModelCoefficients(
    const std::vector<PCLModelCoefficientMsg>& coefficients);
  
  std::vector<PCLIndicesMsg>
  convertToROSPointIndices(
    const std::vector<pcl::PointIndices::Ptr> cluster_indices,
    const std_msgs::Header& header);

  std::vector<PCLIndicesMsg>
  convertToROSPointIndices(
    const std::vector<pcl::PointIndices> cluster_indices,
    const std_msgs::Header& header);

  std::vector<PCLModelCoefficientMsg>
  convertToROSModelCoefficients(
    const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
    const std_msgs::Header& header);

}

namespace tf
{
  // for eigen float
  void poseMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Affine3f& eigen);
  void poseEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Pose& msg);
  void transformMsgToEigen(const geometry_msgs::Transform& msg, Eigen::Affine3f& eigen);
  void transformEigenToMsg(Eigen::Affine3f& eigen, geometry_msgs::Transform& msg);
  void transformTFToEigen(const tf::Transform& t, Eigen::Affine3f& eigen);
  void transformEigenToTF(Eigen::Affine3f& eigen , tf::Transform& t);
}

#endif
