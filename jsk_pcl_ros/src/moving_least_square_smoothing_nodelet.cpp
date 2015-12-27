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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/moving_least_square_smoothing.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void MovingLeastSquareSmoothing::smooth(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    std::vector<int> indices;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> smoother;
    smoother.setSearchRadius (search_radius_);
    if (gauss_param_set_) smoother.setSqrGaussParam (gauss_param_set_);
    smoother.setPolynomialFit (use_polynomial_fit_);
    smoother.setPolynomialOrder (polynomial_order_);
    smoother.setComputeNormals (calc_normal_);

    typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new typename pcl::search::KdTree<pcl::PointXYZRGB> ());
    smoother.setSearchMethod (tree);
    smoother.setInputCloud (cloud);
    smoother.process (*result_cloud);

    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(*result_cloud, pointcloud2);
    pointcloud2.header.frame_id = input->header.frame_id;
    pointcloud2.header.stamp = input->header.stamp;
    pub_.publish(pointcloud2);
  }

  void MovingLeastSquareSmoothing::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &MovingLeastSquareSmoothing::smooth, this);
  }

  void MovingLeastSquareSmoothing::unsubscribe()
  {
    sub_input_.shutdown();
  }

  void MovingLeastSquareSmoothing::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    search_radius_ = config.search_radius;
    gauss_param_set_ = config.gauss_param_set;
    use_polynomial_fit_ = config.use_polynomial_fit;
    polynomial_order_ = config.polynomial_order;
    calc_normal_ = config.calc_normal;
  }

  void MovingLeastSquareSmoothing::onInit(void)
  {
    DiagnosticNodelet::onInit();
    pnh_->param("gauss_param_set", gauss_param_set_, false);
    pnh_->param("search_radius", search_radius_, 0.03);
    pnh_->param("use_polynomial_fit", use_polynomial_fit_, false);
    pnh_->param("polynomial_order", polynomial_order_, 2);
    pnh_->param("calc_normal", calc_normal_, true);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&MovingLeastSquareSmoothing::configCallback, this, _1, _2);
    srv_->setCallback(f);
    pub_ =advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::MovingLeastSquareSmoothing, nodelet::Nodelet);
