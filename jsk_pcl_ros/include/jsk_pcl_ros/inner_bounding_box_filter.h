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
#include <Eigen/Dense>
#include <jsk_recognition_utils/pcl_util.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/pcl_nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>

namespace jsk_pcl_ros
{
  class InnerBoundingBoxFilter: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
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

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_bbox_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approx_sync_;
    ros::Publisher pub_indices_;
    boost::mutex mutex_;

    jsk_recognition_utils::TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    jsk_topic_tools::VitalChecker::Ptr vital_checker_;
    bool approximate_sync_;
    double padding_rate_;

  private:


    struct BoundingBox {
      BoundingBox(const jsk_recognition_msgs::BoundingBox &bbox) {
          Eigen::Translation3d ctrans
            = Eigen::Translation3d(bbox.pose.position.x,
                                   bbox.pose.position.y,
                                   bbox.pose.position.z);
          Eigen::Quaterniond cquat
            = Eigen::Quaterniond(bbox.pose.orientation.x,
                                 bbox.pose.orientation.y,
                                 bbox.pose.orientation.z,
                                 bbox.pose.orientation.w);
          Eigen::Affine3d transform = ctrans * cquat;
          Eigen::Vector3d vddd;
          vddd << -bbox.dimensions.x / 2.0,
            -bbox.dimensions.y / 2.0,
            -bbox.dimensions.z / 2.0;
          vddd = transform * vddd;
          Eigen::Vector3d vddu;
          vddu << -bbox.dimensions.x / 2.0,
            -bbox.dimensions.y / 2.0,
            +bbox.dimensions.z / 2.0;
          vddu = transform * vddu;
          Eigen::Vector3d vdud;
          vdud << -bbox.dimensions.x / 2.0,
            +bbox.dimensions.y / 2.0,
            -bbox.dimensions.z / 2.0;
          vdud = transform * vdud;
          Eigen::Vector3d vudd;
          vudd << +bbox.dimensions.x / 2.0,
            -bbox.dimensions.y / 2.0,
            -bbox.dimensions.z / 2.0;
          vudd = transform * vudd;
          Eigen::Vector3d vuud;
          vuud << +bbox.dimensions.x / 2.0,
            +bbox.dimensions.y / 2.0,
            -bbox.dimensions.z / 2.0;
          vuud = transform * vuud;
          Eigen::Vector3d vudu;
          vudu << +bbox.dimensions.x / 2.0,
            -bbox.dimensions.y / 2.0,
            +bbox.dimensions.z / 2.0;
          vudu = transform * vudu;
          Eigen::Vector3d vduu;
          vduu << -bbox.dimensions.x / 2.0,
            +bbox.dimensions.y / 2.0,
            +bbox.dimensions.z / 2.0;
          vduu = transform * vduu;
          Eigen::Vector3d vuuu;
          vuuu << +bbox.dimensions.x / 2.0,
            +bbox.dimensions.y / 2.0,
            +bbox.dimensions.z / 2.0;
          vuuu = transform * vuuu;

          Eigen::Matrix3d mzd = Eigen::Matrix3d::Zero();
          mzd.col(0) = vddd;
          mzd.col(1) = vudd;
          mzd.col(2) = vdud;
          matrices.push_back(mzd);
          Eigen::Matrix3d mxd = Eigen::Matrix3d::Zero();
          mxd.col(0) = vddd;
          mxd.col(1) = vdud;
          mxd.col(2) = vddu;
          matrices.push_back(mxd);
          Eigen::Matrix3d myd = Eigen::Matrix3d::Zero();
          myd.col(0) = vddd;
          myd.col(1) = vddu;
          myd.col(2) = vudd;
          matrices.push_back(myd);
          Eigen::Matrix3d mzu = Eigen::Matrix3d::Zero();
          mzu.col(0) = vuuu;
          mzu.col(1) = vudu;
          mzu.col(2) = vduu;
          matrices.push_back(mzu);
          Eigen::Matrix3d mxu = Eigen::Matrix3d::Zero();
          mxu.col(0) = vuuu;
          mxu.col(1) = vuud;
          mxu.col(2) = vudu;
          matrices.push_back(mxu);
          Eigen::Matrix3d myu = Eigen::Matrix3d::Zero();
          myu.col(0) = vuuu;
          myu.col(1) = vduu;
          myu.col(2) = vuud;
          matrices.push_back(myu);
      }
      bool innerPoint(const pcl::PointXYZ &p) {
//      bool innerPoint(const Eigen::Vector4d &p) {
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
