// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "jsk_pcl_ros_utils/plane_rejector.h"
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <eigen_conversions/eigen_msg.h>

namespace jsk_pcl_ros_utils
{
  void PlaneRejector::onInit()
  {
    ConnectionBasedNodelet::onInit();
    tf_success_.reset(new jsk_recognition_utils::SeriesedBoolean(30));
    listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    double vital_rate;
    pnh_->param("vital_rate", vital_rate, 1.0);
    vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    
    diagnostic_updater_.reset(new diagnostic_updater::Updater);
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(
      getName() + "::PlaneRejector",
      boost::bind(
        &PlaneRejector::updateDiagnosticsPlaneRejector,
        this, _1));
    if (!pnh_->getParam("processing_frame_id", processing_frame_id_)) {
      NODELET_FATAL("You need to specify ~processing_frame_id");
      return;
    }
    pnh_->param("use_inliers", use_inliers_, false);
    pnh_->param("allow_flip", allow_flip_, false);

    std::vector<double> reference_axis;
    if (!jsk_topic_tools::readVectorParameter(
          *pnh_, "reference_axis", reference_axis)) {
      NODELET_FATAL("you need to specify ~reference_axis");
      return;
    }
    else if (reference_axis.size() != 3){
      NODELET_FATAL("~reference_axis is not 3 length vector");
      return;
    }
    else {
      jsk_recognition_utils::pointFromVectorToVector(reference_axis, reference_axis_);
      reference_axis_.normalize();
    }
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PlaneRejector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    polygons_pub_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output_polygons", 1);
    coefficients_pub_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output_coefficients", 1);
    if (use_inliers_) {
      inliers_pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output_inliers", 1);
    }
    else {
    }
    diagnostics_timer_ = pnh_->createTimer(
      ros::Duration(1.0),
      boost::bind(&PlaneRejector::updateDiagnostics,
                  this,
                  _1));
    onInitPostProcess();
  }

  void PlaneRejector::subscribe()
  {
    if (use_inliers_) {
      sync_inlier_ = boost::make_shared<message_filters::Synchronizer<SyncInlierPolicy> >(100);
      sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
      sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
      sub_inliers_.subscribe(*pnh_, "input_inliers", 1);
      sync_inlier_->connectInput(sub_polygons_, sub_coefficients_, sub_inliers_);
      sync_inlier_->registerCallback(boost::bind(&PlaneRejector::reject, this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
      sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
      sync_->connectInput(sub_polygons_, sub_coefficients_);
      sync_->registerCallback(boost::bind(&PlaneRejector::reject, this, _1, _2));
    }
  }

  void PlaneRejector::unsubscribe()
  {
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }
  
  void PlaneRejector::updateDiagnosticsPlaneRejector(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    bool alivep = vital_checker_->isAlive();
    // check tf successeed or not
    if (alivep) {
      if (tf_success_->getValue()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     "PlaneRejector running");
      }
      else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "failed to tf transform");
      }
      stat.add("Input planes (Avg.)", input_plane_counter_.mean());
      stat.add("Rejected Planes (Avg.)", rejected_plane_counter_.mean());
      stat.add("Passed Planes (Avg.)", passed_plane_counter_.mean());
      stat.add("Angular Threahold", angle_thr_);
      stat.add("Reference Axis",
               (boost::format("[%f, %f, %f]")
                % (reference_axis_[0])
                %  (reference_axis_[1])
                % (reference_axis_[2])).str());
      stat.add("Processing Frame ID", processing_frame_id_);
    }
    else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "PlaneRejector not running");
    }
    
  }
  
  void PlaneRejector::updateDiagnostics(const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(mutex_);
    diagnostic_updater_->update();
  }
  
  void PlaneRejector::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    angle_thr_ = config.angle_thr;
    angle_ = config.angle;
  }
  
  void PlaneRejector::reject(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients)
  {
    reject(polygons, coefficients, jsk_recognition_msgs::ClusterPointIndices::ConstPtr());
  }

  void PlaneRejector::reject(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& inliers)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    jsk_recognition_msgs::PolygonArray result_polygons;
    jsk_recognition_msgs::ModelCoefficientsArray result_coefficients;
    jsk_recognition_msgs::ClusterPointIndices result_inliers;
    result_polygons.header = polygons->header;
    result_coefficients.header = coefficients->header;
    result_inliers.header = polygons->header;
    input_plane_counter_.add(polygons->polygons.size());
    int rejected_plane_counter = 0;
    int passed_plane_counter = 0;
    for (size_t i = 0; i < polygons->polygons.size(); i++) {
      geometry_msgs::PolygonStamped polygon = polygons->polygons[i];
      PCLModelCoefficientMsg coefficient = coefficients->coefficients[i];
      // transform the coefficients to processing_frame_id_
      if (listener_->canTransform(coefficient.header.frame_id,
                                  processing_frame_id_,
                                  coefficient.header.stamp)) {
        tf_success_->addValue(true);
        geometry_msgs::Vector3Stamped plane_axis;
        plane_axis.header = coefficient.header;
        plane_axis.vector.x = coefficient.values[0];
        plane_axis.vector.y = coefficient.values[1];
        plane_axis.vector.z = coefficient.values[2];
        geometry_msgs::Vector3Stamped transformed_plane_axis;
        listener_->transformVector(processing_frame_id_,
                                   plane_axis,
                                   transformed_plane_axis);
        Eigen::Vector3d eigen_transformed_plane_axis;
        tf::vectorMsgToEigen(transformed_plane_axis.vector,
                             eigen_transformed_plane_axis);
        if (allow_flip_ && eigen_transformed_plane_axis.normalized().dot(reference_axis_) < 0) {
          // flip normal vector to make its direction same as reference axis
          eigen_transformed_plane_axis = -eigen_transformed_plane_axis;
        }
        double ang = std::abs(acos(eigen_transformed_plane_axis.normalized().dot(reference_axis_)) - angle_);
        if (ang < angle_thr_) {
          ++passed_plane_counter;
          result_polygons.polygons.push_back(polygons->polygons[i]);
          result_coefficients.coefficients.push_back(coefficient);
          if (use_inliers_) {
            result_inliers.cluster_indices.push_back(inliers->cluster_indices[i]);
          }
        }
        else {
          ++rejected_plane_counter;
        }
      }
      else {
        tf_success_->addValue(false);
     }
    }
    rejected_plane_counter_.add(rejected_plane_counter);
    passed_plane_counter_.add(passed_plane_counter);
    polygons_pub_.publish(result_polygons);
    coefficients_pub_.publish(result_coefficients);
    if (use_inliers_) {
      inliers_pub_.publish(result_inliers);
    }
    diagnostic_updater_->update();
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PlaneRejector, nodelet::Nodelet);
