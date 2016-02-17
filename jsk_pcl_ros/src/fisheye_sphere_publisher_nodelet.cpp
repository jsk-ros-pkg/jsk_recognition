/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "jsk_pcl_ros/fisheye_sphere_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl/common/centroid.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  void FisheyeSpherePublisher::extract(const sensor_msgs::ImageConstPtr& input)
  {
    float max_degree = 90;
    float max_radian = max_degree * 3.14159265 /180.0;
    float tan_max_radian = tan(max_radian);
    const float K = 341.656050955 * downsample_rate_;
    const float absolute_max_degree = 85;
    const float absolute_max_radian = absolute_max_degree * 3.14159265 /180.0;
    float max_radius = max_radian * K;
    cv::Mat distorted = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8)->image, shrink_distorted;
    cv::resize(distorted, shrink_distorted, cv::Size(), downsample_rate_, downsample_rate_);
    int center_x = shrink_distorted.rows/2, center_y = shrink_distorted.cols/2;

    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;

    for(int i = 0; i < shrink_distorted.rows; ++i){
      for(int j = 0; j < shrink_distorted.cols; ++j){
        float radius = sqrt((i-center_x)*(i-center_x) + (j-center_y)*(j-center_y));
        float radian = radius/K;
        if( radian < max_radian ){
          float l = sphere_radius_ * sin(radian);
          float rate = l/radius;

          pcl::PointXYZRGB p;
          p.x = -(i - center_x) * rate;
          p.y = (j - center_y) * rate;
          p.z = sphere_radius_ * cos(radian);
          p.r = shrink_distorted.data[ int(i) * shrink_distorted.step + int(j) * shrink_distorted.elemSize() + 2];
          p.g = shrink_distorted.data[ int(i) * shrink_distorted.step + int(j) * shrink_distorted.elemSize() + 1];
          p.b = shrink_distorted.data[ int(i) * shrink_distorted.step + int(j) * shrink_distorted.elemSize() + 0];
          pointcloud.push_back(p);
        }
      }
    }

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(pointcloud, pc2);
    pc2.header.frame_id = "fisheye";
    pc2.header.stamp = ros::Time::now();
    pub_sphere_.publish(pc2);
  }

  void FisheyeSpherePublisher::subscribe()
  {
    sub_image_ = pnh_->subscribe("input", 1, &FisheyeSpherePublisher::extract, this);

  }

  void FisheyeSpherePublisher::unsubscribe()
  {
    sub_image_.shutdown();
  }

  void FisheyeSpherePublisher::configCallback(Config &config, uint32_t level)
  {
    downsample_rate_ = config.downsample_rate;
    sphere_radius_ = config.sphere_radius;
  }

  void FisheyeSpherePublisher::onInit(void)
  {
    DiagnosticNodelet::onInit();
    downsample_rate_ = 0.5;
    sphere_radius_ = 1.0;
    pub_sphere_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    sub_image_ = pnh_->subscribe("input", 1, &FisheyeSpherePublisher::extract, this);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FisheyeSpherePublisher::configCallback, this, _1, _2);
    srv_->setCallback (f);

    onInitPostProcess();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FisheyeSpherePublisher, nodelet::Nodelet);
