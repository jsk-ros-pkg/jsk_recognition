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


#ifndef JSK_PERCEPTION_POLYGON_ARRAY_COLOR_LIKELIHOOD_H_
#define JSK_PERCEPTION_POLYGON_ARRAY_COLOR_LIKELIHOOD_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/HistogramWithRangeArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_perception/PolygonArrayColorLikelihoodConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>

namespace jsk_perception
{
  class PolygonArrayColorLikelihood: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<PolygonArrayColorLikelihood> Ptr;
    typedef PolygonArrayColorLikelihoodConfig Config;
    PolygonArrayColorLikelihood(): DiagnosticNodelet("PolygonArrayColorLikelihood") {}
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::HistogramWithRangeArray > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::HistogramWithRangeArray > ApproximateSyncPolicy;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void referenceCallback(
      const jsk_recognition_msgs::HistogramWithRange::ConstPtr& ref_msg);
    virtual void likelihood(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::HistogramWithRangeArray::ConstPtr& histogram_msg);
    virtual void configCallback(Config &config, uint32_t level);
    virtual double compareHist(
      const cv::MatND& ref_hist, const cv::MatND& target_hist);
    virtual void readReference(const std::string& file);
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_reference_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon_;
    message_filters::Subscriber<jsk_recognition_msgs::HistogramWithRangeArray> sub_histogram_;
    jsk_recognition_msgs::HistogramWithRange::ConstPtr reference_;
    bool approximate_sync_;
    int max_queue_size_;
    int sync_queue_size_;
    int coefficient_method_;
    bool reference_from_file_;
  private:
    
  };
}

#endif
