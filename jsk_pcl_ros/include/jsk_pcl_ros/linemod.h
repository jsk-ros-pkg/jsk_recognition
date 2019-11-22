// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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


#ifndef JSK_PCL_ROS_LINEMOD_H_
#define JSK_PCL_ROS_LINEMOD_H_

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <image_geometry/pinhole_camera_model.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

#include <jsk_pcl_ros/LINEMODDetectorConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include <std_srvs/Empty.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl_ros/pcl_nodelet.h>
#include <yaml-cpp/yaml.h>

namespace jsk_pcl_ros
{
  class LINEMODDetector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef LINEMODDetectorConfig Config;
    LINEMODDetector(): DiagnosticNodelet("LINEMODDetector") {}
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void setTemplate(YAML::Node doc);
    virtual void detect(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void configCallback(
      Config& config, uint32_t level);
    virtual void computeCenterOfTemplate(
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
      const pcl::SparseQuantizedMultiModTemplate& linemod_template,
      const pcl::LINEMODDetection& linemod_detection,
      Eigen::Vector3f& center);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_detect_mask_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_original_template_cloud_;
    boost::mutex mutex_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    std::string template_file_;
    double gradient_magnitude_threshold_;
    double detection_threshold_;
    //pcl::LineRGBD<pcl::PointXYZRGBA> line_rgbd_;
    pcl::LINEMOD linemod_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr template_cloud_;
    std::vector<Eigen::Affine3f> template_poses_;
    std::vector<jsk_recognition_msgs::BoundingBox> template_bboxes_;
    pcl::ColorGradientModality<pcl::PointXYZRGBA> color_gradient_mod_;
    pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_normal_mod_;
  private:
    
  };
  
  class LINEMODTrainer: public pcl_ros::PCLNodelet
  {
  public:
    typedef boost::shared_ptr<LINEMODTrainer> Ptr;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      PCLIndicesMsg> SyncPolicy;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void store(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const PCLIndicesMsg::ConstPtr& indices_msg);
    virtual void subscribeCloud(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void subscribeCameraInfo(
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual bool startTraining(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res);
    virtual std::vector<std::string> trainOneData(
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
      pcl::PointIndices::Ptr mask,
      std::string& tempstr,
      int i);
    virtual void tar(const std::string& directory, const std::string& output);
    virtual bool clearData(std_srvs::Empty::Request& req,
                           std_srvs::Empty::Response& res);
    virtual void trainWithoutViewpointSampling();
    virtual void trainWithViewpointSampling();
    virtual void organizedPointCloudWithViewPoint(
      const Eigen::Affine3f& transform,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr raw_cloud,
      const image_geometry::PinholeCameraModel& model,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output,
      pcl::PointIndices& mask);
    virtual void generateLINEMODTrainingData(
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
      pcl::PointIndices::Ptr mask,
      pcl::ColorGradientModality<pcl::PointXYZRGBA>& color_grad_mod,
      pcl::SurfaceNormalModality<pcl::PointXYZRGBA>& surface_norm_mod,
      pcl::MaskMap& mask_map,
      pcl::RegionXY& region);
    ////////////////////////////////////////////////////////
    // variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<PCLIndicesMsg> sub_indices_;
    ros::ServiceServer start_training_srv_;
    ros::ServiceServer clear_data_srv_;
    ros::Publisher pub_range_image_;
    ros::Publisher pub_colored_range_image_;
    ros::Publisher pub_sample_cloud_;
    ros::Subscriber sub_input_nonsync_;
    ros::Subscriber sub_camera_info_nonsync_;
    sensor_msgs::CameraInfo::ConstPtr camera_info_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> samples_before_sampling_;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> samples_;
    std::vector<pcl::PointIndices::Ptr> sample_indices_;
    boost::mutex mutex_;
    std::string output_file_;
    bool sample_viewpoint_;
    double sample_viewpoint_angle_step_;
    double sample_viewpoint_radius_step_;
    double sample_viewpoint_angle_min_;
    double sample_viewpoint_radius_min_;
    double sample_viewpoint_angle_max_;
    double sample_viewpoint_radius_max_;
    int n_points_;
  private:
    
  };
}


#endif
