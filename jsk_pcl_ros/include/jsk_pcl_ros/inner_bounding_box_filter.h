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
/*
 * inner_bounding_box_filter.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef INNER_BOUNDING_BOX_FILTER_H__
#define INNER_BOUNDING_BOX_FILTER_H__

#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Dense>
#include <jsk_recognition_utils/pcl_util.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/pcl_nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/InnerBoundingBoxFilterConfig.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>

namespace jsk_pcl_ros
{
  class InnerBoundingBoxFilter: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef jsk_pcl_ros::InnerBoundingBoxFilterConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::BoundingBoxArray> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::BoundingBoxArray> ApproximateSyncPolicy;
  protected:
    virtual void onInit();
    virtual void filter(
      const sensor_msgs::PointCloud2::ConstPtr & pc_msg,
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &bbox_msg);
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);

    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_bbox_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approx_sync_;
    ros::Publisher pub_indices_;
    boost::mutex mutex_;

    jsk_recognition_utils::TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    jsk_topic_tools::VitalChecker::Ptr vital_checker_;
    bool approximate_sync_;
    int processed_msg_num_;
    double padding_rate_;

  private:


    struct BoundingBox {
      BoundingBox(const jsk_recognition_msgs::BoundingBox &bbox, double padding) {
          Eigen::Translation3d ctrans
            = Eigen::Translation3d(bbox.pose.position.x,
                                   bbox.pose.position.y,
                                   bbox.pose.position.z);
          Eigen::Quaterniond cquat
            = Eigen::Quaterniond(bbox.pose.orientation.x,
                                 bbox.pose.orientation.y,
                                 bbox.pose.orientation.z,
                                 bbox.pose.orientation.w);
          double x = bbox.dimensions.x / 2.0 * padding;
          double y = bbox.dimensions.y / 2.0 * padding;
          double z = bbox.dimensions.z / 2.0 * padding;
          Eigen::Affine3d transform = ctrans * cquat;
          Eigen::Vector3d vddd(-x,-y,-z);
          vddd = transform * vddd;
          Eigen::Vector3d vddu(-x,-y,+z);
          vddu = transform * vddu;
          Eigen::Vector3d vdud(-x,+y,-z);
          vdud = transform * vdud;
          Eigen::Vector3d vudd(+x,-y,-z);
          vudd = transform * vudd;
          Eigen::Vector3d vuud(+x,+y,-z);
          vuud = transform * vuud;
          Eigen::Vector3d vudu(+x,-y,+z);
          vudu = transform * vudu;
          Eigen::Vector3d vduu(-x,+y,+z);
          vduu = transform * vduu;
          Eigen::Vector3d vuuu(+x,+y,+z);
          vuuu = transform * vuuu;

          Eigen::Matrix3d mzd;
          mzd.col(0) = vddd;
          mzd.col(1) = vudd;
          mzd.col(2) = vdud;
          matrices.push_back(mzd);
          Eigen::Matrix3d mxd;
          mxd.col(0) = vddd;
          mxd.col(1) = vdud;
          mxd.col(2) = vddu;
          matrices.push_back(mxd);
          Eigen::Matrix3d myd;
          myd.col(0) = vddd;
          myd.col(1) = vddu;
          myd.col(2) = vudd;
          matrices.push_back(myd);
          Eigen::Matrix3d mzu;
          mzu.col(0) = vuuu;
          mzu.col(1) = vudu;
          mzu.col(2) = vduu;
          matrices.push_back(mzu);
          Eigen::Matrix3d mxu;
          mxu.col(0) = vuuu;
          mxu.col(1) = vuud;
          mxu.col(2) = vudu;
          matrices.push_back(mxu);
          Eigen::Matrix3d myu;
          myu.col(0) = vuuu;
          myu.col(1) = vduu;
          myu.col(2) = vuud;
          matrices.push_back(myu);
      }
      bool innerPoint(const pcl::PointXYZ &p) {
          Eigen::Vector3d v(p.x, p.y, p.z);
          bool inner = true;
          for (int i = 0; i < matrices.size(); ++i)
          {
            Eigen::Matrix3d m = matrices[i];
            m.colwise() -= v;
            if (m.determinant() > 0.0) return false;
          }
          return true;
      }

      std::vector<Eigen::Matrix3d> matrices;
    };
  };
}

#endif // INNER_BOUNDING_BOX_FILTER_H__
