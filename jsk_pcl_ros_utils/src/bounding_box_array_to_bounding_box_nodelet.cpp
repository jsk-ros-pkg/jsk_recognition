/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include "jsk_pcl_ros_utils/bounding_box_array_to_bounding_box.h"
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_recognition_msgs/BoundingBox.h>

namespace jsk_pcl_ros_utils
{

  void BoundingBoxArrayToBoundingBox::onInit()
  {
    DiagnosticNodelet::onInit();

    // dynamic_reconfigure
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&BoundingBoxArrayToBoundingBox::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<jsk_recognition_msgs::BoundingBox>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void BoundingBoxArrayToBoundingBox::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    index_ = config.index;
  }

  void BoundingBoxArrayToBoundingBox::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &BoundingBoxArrayToBoundingBox::convert, this);
  }

  void BoundingBoxArrayToBoundingBox::unsubscribe()
  {
    sub_.shutdown();
  }

  void BoundingBoxArrayToBoundingBox::convert(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& bbox_array_msg)
  {
    vital_checker_->poke();

    jsk_recognition_msgs::BoundingBox bbox_msg;
    bbox_msg.header = bbox_array_msg->header;

    int array_size = bbox_array_msg->boxes.size();
    if (index_ < 0) {
      return;
    } else if (index_ < array_size) {
      bbox_msg = bbox_array_msg->boxes[index_];
    } else {
      NODELET_ERROR_THROTTLE(10, "Invalid ~index %d is specified for array size %d.", index_, array_size);
    }

    pub_.publish(bbox_msg);
  }

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::BoundingBoxArrayToBoundingBox, nodelet::Nodelet);
