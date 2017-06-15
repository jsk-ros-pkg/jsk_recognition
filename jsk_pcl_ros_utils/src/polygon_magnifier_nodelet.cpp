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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros_utils/polygon_magnifier.h"
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_pcl_ros_utils
{
  void PolygonMagnifier::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PolygonMagnifier::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PolygonMagnifier::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &PolygonMagnifier::magnify, this);
  }

  void PolygonMagnifier::unsubscribe()
  {
    sub_.shutdown();
  }

  void PolygonMagnifier::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    use_scale_factor_ = config.use_scale_factor;
    magnify_distance_ = config.magnify_distance;
    magnify_scale_factor_ = config.magnify_scale_factor;
  }

  void PolygonMagnifier::magnify(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();

    jsk_recognition_msgs::PolygonArray ret_polygon_array = *msg;

    for (size_t i = 0; i < msg->polygons.size(); i++) {
      jsk_recognition_utils::ConvexPolygon poly =
        jsk_recognition_utils::ConvexPolygon::fromROSMsg(msg->polygons[i].polygon);

      jsk_recognition_utils::ConvexPolygon::Ptr magnified_poly;
      if (use_scale_factor_) magnified_poly = poly.magnify(magnify_scale_factor_);
      else                   magnified_poly = poly.magnifyByDistance(magnify_distance_);

      if (!magnified_poly->isConvex()) {
        ROS_WARN("Magnified polygon %ld is not convex.", i);
      }

      ret_polygon_array.polygons[i].polygon = magnified_poly->toROSMsg();
    }
    pub_.publish(ret_polygon_array);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonMagnifier, nodelet::Nodelet);
