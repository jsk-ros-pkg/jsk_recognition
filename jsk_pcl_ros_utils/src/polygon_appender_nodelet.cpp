// -*- mode: C++ -*-
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
 *     disclaimer in the documentation and/or other materials provided
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

#include "jsk_pcl_ros_utils/polygon_appender.h"
#include <pluginlib/class_list_macros.hpp>

namespace jsk_pcl_ros_utils
{
  void PolygonAppender::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pub_polygon_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output", 1);
    pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_,
      "output_coefficients", 1);
    
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy2> >(100);
    sync_->connectInput(sub_polygon0_, sub_coefficients0_,
                        sub_polygon1_, sub_coefficients1_);
    sync_->registerCallback(boost::bind(&PolygonAppender::callback2, this, _1, _2, _3, _4));
    onInitPostProcess();
  }

  PolygonAppender::~PolygonAppender() {
    // message_filters::Synchronizer needs to be called reset
    // before message_filters::Subscriber is freed.
    // Calling reset fixes the following error on shutdown of the nodelet:
    // terminate called after throwing an instance of
    // 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    //     what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    // Also see https://github.com/ros/ros_comm/issues/720 .
    sync_.reset();
  }

  void PolygonAppender::subscribe()
  {
    sub_polygon0_.subscribe(*pnh_, "input0", 1);
    sub_polygon1_.subscribe(*pnh_, "input1", 1);
    sub_coefficients0_.subscribe(*pnh_, "input_coefficients0", 1);
    sub_coefficients1_.subscribe(*pnh_, "input_coefficients1", 1);
  }

  void PolygonAppender::unsubscribe()
  {
    sub_polygon0_.unsubscribe();
    sub_polygon1_.unsubscribe();
    sub_coefficients0_.unsubscribe();
    sub_coefficients1_.unsubscribe();
  }
  
  void PolygonAppender::callback2(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg0,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients0,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg1,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients1)
  {
    std::vector<jsk_recognition_msgs::PolygonArray::ConstPtr> arrays;
    arrays.push_back(msg0);
    arrays.push_back(msg1);
    std::vector<jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr> coefficients_array;
    coefficients_array.push_back(coefficients0);
    coefficients_array.push_back(coefficients1);
    appendAndPublish(arrays, coefficients_array);
  }

  void PolygonAppender::appendAndPublish(
    const std::vector<jsk_recognition_msgs::PolygonArray::ConstPtr>& arrays,
    const std::vector<jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr>& coefficients_array)
  {
    if (arrays.size() == 0) {
      NODELET_ERROR("there is not enough polygons");
      return;
    }
    if (coefficients_array.size() == 0) {
      NODELET_ERROR("there is not enough coefficients");
      return;
    }
    if (arrays.size() != coefficients_array.size()) {
      NODELET_ERROR("polygons and coefficients are not the same length");
      return;
    }
    jsk_recognition_msgs::PolygonArray new_array;
    new_array.header = arrays[0]->header;
    for (size_t i = 0; i < arrays.size(); i++) {
      jsk_recognition_msgs::PolygonArray::ConstPtr array = arrays[i];
      for (size_t j = 0; j < array->polygons.size(); j++) {
        geometry_msgs::PolygonStamped polygon = array->polygons[j];
        new_array.polygons.push_back(polygon);
      }
    }
    pub_polygon_.publish(new_array);

    jsk_recognition_msgs::ModelCoefficientsArray coefficients_new_array;
    coefficients_new_array.header = coefficients_array[0]->header;
    for (size_t i = 0; i < coefficients_array.size(); i++) {
      jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr array = coefficients_array[i];
      for (size_t j = 0; j < array->coefficients.size(); j++) {
        coefficients_new_array.coefficients.push_back(array->coefficients[j]);
      }
    }
    pub_coefficients_.publish(coefficients_new_array);
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonAppender, nodelet::Nodelet);

