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

#include "jsk_pcl_ros_utils/polygon_array_area_likelihood.h"
#include "jsk_recognition_utils/tf_listener_singleton.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros_utils
{
  void PolygonArrayAreaLikelihood::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PolygonArrayAreaLikelihood::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PolygonArrayAreaLikelihood::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", 1, &PolygonArrayAreaLikelihood::likelihood, this);
  }

  void PolygonArrayAreaLikelihood::unsubscribe()
  {
    sub_.shutdown();
  }

  void PolygonArrayAreaLikelihood::likelihood(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    jsk_recognition_msgs::PolygonArray new_msg(*msg);

    double min_area = DBL_MAX;
    double max_area = - DBL_MAX;
    std::vector<double> areas; 
    for (size_t i = 0; i < msg->polygons.size(); i++) {
      jsk_recognition_utils::Polygon::Ptr polygon
        = jsk_recognition_utils::Polygon::fromROSMsgPtr(msg->polygons[i].polygon);
      double area = polygon->area();
      min_area = std::min(area, min_area);
      max_area = std::max(area, max_area);
      areas.push_back(area);
    }

    // Normalization
    for (size_t i = 0; i < areas.size(); i++) {
      double diff = areas[i] - area_;
      double likelihood = 1 / (1 + (diff * diff));
      if (msg->likelihood.size() == 0) {
        new_msg.likelihood.push_back(likelihood);
      }
      else {
        new_msg.likelihood[i] = new_msg.likelihood[i] * likelihood;
      }
    }
    pub_.publish(new_msg);
  }

  void PolygonArrayAreaLikelihood::configCallback(
    Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    area_ = config.area;
  }
  
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonArrayAreaLikelihood,
                        nodelet::Nodelet);
