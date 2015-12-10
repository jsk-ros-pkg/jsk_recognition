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

#include "jsk_pcl_ros/polygon_array_unwrapper.h"

namespace jsk_pcl_ros
{
  void PolygonArrayUnwrapper::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pub_polygon_ = advertise<geometry_msgs::PolygonStamped>(
      *pnh_, "output_polygon", 1);
    pub_coefficients_
      = advertise<pcl_msgs::ModelCoefficients>(
        *pnh_,
        "output_coefficients", 1);
  }

  void PolygonArrayUnwrapper::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygon_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_polygon_, sub_coefficients_);
    sync_->registerCallback(boost::bind(
                              &PolygonArrayUnwrapper::unwrap,
                              this, _1, _2));
  }

  void PolygonArrayUnwrapper::unsubscribe()
  {
    sub_polygon_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }
  
  void PolygonArrayUnwrapper::unwrap(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients)
  {
    if (polygons->polygons.size() > 0) {
      geometry_msgs::PolygonStamped polygon_msg = polygons->polygons[0];
      pcl_msgs::ModelCoefficients coefficients_msg = coefficients->coefficients[0];
      pub_polygon_.publish(polygon_msg);
      pub_coefficients_.publish(coefficients_msg);
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PolygonArrayUnwrapper, nodelet::Nodelet);
