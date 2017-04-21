/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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


#ifndef JSK_PERCEPTION_FILTER_MASK_IMAGE_WITH_SIZE_H_
#define JSK_PERCEPTION_FILTER_MASK_IMAGE_WITH_SIZE_H_

#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include "jsk_perception/FilterMaskImageWithSizeConfig.h"

namespace jsk_perception
{
  class FilterMaskImageWithSize: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    FilterMaskImageWithSize(): DiagnosticNodelet("FilterMaskImageWithSize") {}
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::Image > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image > ApproxSyncPolicy;
    typedef jsk_perception::FilterMaskImageWithSizeConfig Config;
  protected:
    virtual void onInit();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void filter(const sensor_msgs::Image::ConstPtr& input_msg);
    virtual void filter(const sensor_msgs::Image::ConstPtr& input_msg,
                        const sensor_msgs::Image::ConstPtr& reference_msg);

    boost::mutex mutex_;

    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    double min_size_;
    double max_size_;
    double min_relative_size_;
    double max_relative_size_;

    bool approximate_sync_;
    int queue_size_;
    bool use_reference_;
    message_filters::Subscriber<sensor_msgs::Image> sub_input_;
    message_filters::Subscriber<sensor_msgs::Image> sub_reference_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy> > async_;

    ros::Publisher pub_;
  private:
  };
}  // namespace jsk_perception

#endif  // JSK_PERCEPTION_FILTER_MASK_IMAGE_WITH_RELATIVE_SIZE_H_
