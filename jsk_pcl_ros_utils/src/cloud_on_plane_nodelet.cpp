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

#include "jsk_pcl_ros_utils/cloud_on_plane.h"
#include <jsk_recognition_utils/pcl_ros_util.h>

namespace jsk_pcl_ros_utils
{
  void CloudOnPlane::onInit()
  {
    DiagnosticNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    pnh_->param("approximate_sync", approximate_sync_, false);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &CloudOnPlane::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<jsk_recognition_msgs::BoolStamped>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void CloudOnPlane::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_polygon_.subscribe(*pnh_, "input/polygon", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> > (100);
      async_->connectInput(sub_cloud_, sub_polygon_);
      async_->registerCallback(boost::bind(&CloudOnPlane::predicate, this, _1, _2));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> > (100);
      sync_->connectInput(sub_cloud_, sub_polygon_);
      sync_->registerCallback(boost::bind(&CloudOnPlane::predicate, this, _1, _2));
    }
  }

  void CloudOnPlane::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_polygon_.unsubscribe();
  }

  void CloudOnPlane::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    distance_thr_ = config.distance_thr;
    buf_size_ = config.buf_size;
    buffer_.reset(new jsk_recognition_utils::SeriesedBoolean(buf_size_));
  }

  void CloudOnPlane::predicate(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                               const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    // check header
    if (!jsk_recognition_utils::isSameFrameId(*cloud_msg, *polygon_msg)) {
      NODELET_ERROR("frame_id does not match: cloud: %s, polygon: %s",
                        cloud_msg->header.frame_id.c_str(), polygon_msg->header.frame_id.c_str());
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // convert jsk_recognition_msgs::PolygonArray to jsk_recognition_utils::Polygon
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> polygons;
    for (size_t i = 0; i < polygon_msg->polygons.size(); i++) {
      polygons.push_back(jsk_recognition_utils::ConvexPolygon::fromROSMsgPtr(polygon_msg->polygons[i].polygon));
    }
    
    for (size_t i = 0; i < polygon_msg->polygons.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr poly = polygons[i];
      for (size_t j = 0; j < cloud->points.size(); j++) {
        Eigen::Vector3f p = cloud->points[j].getVector3fMap();
        if (poly->distanceSmallerThan(p, distance_thr_)) {
          buffer_->addValue(false);
          publishPredicate(cloud_msg->header, !buffer_->isAllTrueFilled());
          return;
        }
      }
    }
    buffer_->addValue(true);
    publishPredicate(cloud_msg->header, !buffer_->isAllTrueFilled());
  }

  void CloudOnPlane::publishPredicate(const std_msgs::Header& header,
                                      const bool v)
  {
    jsk_recognition_msgs::BoolStamped bool_stamped;
    bool_stamped.header = header;
    bool_stamped.data = v;
    pub_.publish(bool_stamped);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::CloudOnPlane, nodelet::Nodelet);

