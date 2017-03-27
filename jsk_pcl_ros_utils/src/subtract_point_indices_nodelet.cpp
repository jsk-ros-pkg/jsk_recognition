// -*- mode: c++ -*-
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

#define BOOST_PARAMETER_MAX_ARITY 7
#include <jsk_pcl_ros_utils/subtract_point_indices.h>


namespace jsk_pcl_ros_utils
{
  void SubtractPointIndices::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_ = advertise<PCLIndicesMsg>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void SubtractPointIndices::subscribe()
  {
    sub_src1_.subscribe(*pnh_, "input/src1", 1);
    sub_src2_.subscribe(*pnh_, "input/src2", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ASyncPolicy> >(100);
      async_->connectInput(sub_src1_, sub_src2_);
      async_->registerCallback(
        boost::bind(&SubtractPointIndices::subtract,
                    this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_src1_, sub_src2_);
      sync_->registerCallback(
        boost::bind(&SubtractPointIndices::subtract,
                    this, _1, _2));
    }
  }

  void SubtractPointIndices::unsubscribe()
  {
    sub_src1_.unsubscribe();
    sub_src2_.unsubscribe();
  }

  void SubtractPointIndices::subtract(
    const PCLIndicesMsg::ConstPtr& src1,
    const PCLIndicesMsg::ConstPtr& src2)
  {
    vital_checker_->poke();
    pcl::PointIndices a, b;
    pcl_conversions::toPCL(*src1, a);
    pcl_conversions::toPCL(*src2, b);
    pcl::PointIndices::Ptr c = jsk_recognition_utils::subIndices(a, b);
    c->header = a.header;
    PCLIndicesMsg ros_indices;
    pcl_conversions::fromPCL(*c, ros_indices);
    ros_indices.header = src1->header;
    pub_.publish(ros_indices);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::SubtractPointIndices, nodelet::Nodelet);
