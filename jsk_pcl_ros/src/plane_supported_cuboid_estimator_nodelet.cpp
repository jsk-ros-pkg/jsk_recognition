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

#include "jsk_pcl_ros/plane_supported_cuboid_estimator.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_util.h"
#include <ctime>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <algorithm>
#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void PlaneSupportedCuboidEstimator::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_ = TfListenerSingleton::getInstance();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PlaneSupportedCuboidEstimator::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("sensor_frame", sensor_frame_, std::string("odom"));
    pub_result_ = pnh_->advertise<jsk_recognition_msgs::BoundingBoxArray>("output/result", 1);
    pub_result_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>("output/result_pose", 1);
    pub_particles_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/particles", 1);
    pub_candidate_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/candidate_cloud", 1);
    pub_histogram_global_x_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/global/x", 1);
    pub_histogram_global_y_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/global/y", 1);
    pub_histogram_global_z_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/global/z", 1);
    pub_histogram_global_roll_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/global/roll", 1);
    pub_histogram_global_pitch_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/global/pitch", 1);
    pub_histogram_global_yaw_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/global/yaw", 1);
    pub_histogram_dx_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/dx", 1);
    pub_histogram_dy_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/dy", 1);
    pub_histogram_dz_ = pnh_->advertise<jsk_recognition_msgs::HistogramWithRange>("output/histogram/dz", 1);
    random_generator_ = boost::mt19937(static_cast<unsigned long>(time(0)));
    // Subscribe
    sub_polygon_.subscribe(*pnh_, "input/polygon", 10);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 10);
    sync_polygon_ = boost::make_shared<message_filters::Synchronizer<PolygonSyncPolicy> >(100);
    sync_polygon_->connectInput(sub_polygon_, sub_coefficients_);
    sync_polygon_->registerCallback(boost::bind(&PlaneSupportedCuboidEstimator::polygonCallback, this, _1, _2));
    sub_cloud_ = pnh_->subscribe("input", 1, &PlaneSupportedCuboidEstimator::cloudCallback, this);
    sub_fast_cloud_ = pnh_->subscribe("fast_input", 1, &PlaneSupportedCuboidEstimator::fastCloudCallback,
                                      this);
    srv_reset_ = pnh_->advertiseService("reset", &PlaneSupportedCuboidEstimator::resetCallback, this);

    onInitPostProcess();
  }

  void PlaneSupportedCuboidEstimator::subscribe()
  {

  }
  
  void PlaneSupportedCuboidEstimator::unsubscribe()
  {

  }

  size_t PlaneSupportedCuboidEstimator::getNearestPolygon(
    const Particle& p,
    const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& polygons)
  {
    size_t min_index = 0;
    double min_distance = DBL_MAX;
    Eigen::Vector3f inp = p.getVector3fMap();
    for (size_t i = 0; i < polygons.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr polygon = polygons[i];
      Eigen::Vector3f p;
      polygon->project(inp, p);
      double d = (p - inp).norm();
      if (d < min_distance) {
        min_distance = d;
        min_index = i;
      }
    }
    return min_index;
  }
  
  void PlaneSupportedCuboidEstimator::updateParticlePolygonRelationship(
    ParticleCloud::Ptr particles)
  {
    if (latest_polygon_msg_->polygons.size() == 0) {
      NODELET_ERROR("no valid polygons, skip update relationship");
      return;
    }

    // The order of convexes and polygons_ should be same
    // because it is inside of critical section.
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes(latest_polygon_msg_->polygons.size());
    for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr polygon = jsk_recognition_utils::ConvexPolygon::fromROSMsgPtr(latest_polygon_msg_->polygons[i].polygon);
      polygon->decomposeToTriangles();
      convexes[i] = polygon;
    }

#ifdef _OPENMP
#pragma omp parallel for
#endif
    // Second, compute distances and assing polygons.
    for (size_t i = 0; i < particles->points.size(); i++) {
      size_t nearest_index = getNearestPolygon(particles->points[i], convexes);
      //particles->points[i].plane = polygons[nearest_index];
      particles->points[i].plane_index = (int)nearest_index;
    }
  }

  void PlaneSupportedCuboidEstimator::fastCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!tracker_) {
      return;
    }
    ParticleCloud::Ptr particles = tracker_->getParticles();
    Eigen::Vector4f center;
    pcl::compute3DCentroid(*particles, center);
    if (center.norm() < fast_cloud_threshold_) {
      estimate(msg);
    }
  }

  void PlaneSupportedCuboidEstimator::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // Update polygons_ vector
    polygons_.clear();
    for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
      Polygon::Ptr polygon = Polygon::fromROSMsgPtr(latest_polygon_msg_->polygons[i].polygon);
      polygon->decomposeToTriangles();
      polygons_.push_back(polygon);
    }
    
    try {
    // viewpoint
    tf::StampedTransform transform
      = lookupTransformWithDuration(tf_, sensor_frame_, msg->header.frame_id,
                                    ros::Time(0.0),
                                    ros::Duration(1.0));
    tf::vectorTFToEigen(transform.getOrigin(), viewpoint_);

    if (!tracker_) {
      NODELET_INFO("initTracker");
      pcl::PointCloud<pcl::tracking::ParticleCuboid>::Ptr particles = initParticles();
      tracker_.reset(new pcl::tracking::ROSCollaborativeParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleCuboid>);
      tracker_->setCustomSampleFunc(boost::bind(&PlaneSupportedCuboidEstimator::sample, this, _1));
      tracker_->setLikelihoodFunc(boost::bind(&PlaneSupportedCuboidEstimator::likelihood, this, _1, _2));
      tracker_->setParticles(particles);
      tracker_->setParticleNum(particle_num_);
    }
    else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      tracker_->setInputCloud(cloud);
      // Before call compute() method, we prepare candidate_cloud_ for
      // weight phase.
      candidate_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      std::set<int> all_indices;
      for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
        Polygon::Ptr polygon = Polygon::fromROSMsgPtr(latest_polygon_msg_->polygons[i].polygon);
        pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism_extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          boundaries (new pcl::PointCloud<pcl::PointXYZ>);
        polygon->boundariesToPointCloud<pcl::PointXYZ>(*boundaries);
        boundaries->points.push_back(boundaries->points[0]);
        prism_extract.setInputCloud(cloud);
        prism_extract.setViewPoint(viewpoint_[0], viewpoint_[1], viewpoint_[2]);
        prism_extract.setHeightLimits(init_local_position_z_min_, init_local_position_z_max_);
        prism_extract.setInputPlanarHull(boundaries);
        pcl::PointIndices output_indices;
        prism_extract.segment(output_indices);
        for (size_t i = 0; i < output_indices.indices.size(); i++) {
          all_indices.insert(output_indices.indices[i]);
        }
      }
      pcl::ExtractIndices<pcl::PointXYZ> ex;
      ex.setInputCloud(cloud);
      pcl::PointIndices::Ptr indices (new pcl::PointIndices);
      indices->indices = std::vector<int>(all_indices.begin(), all_indices.end());
      ex.setIndices(indices);
      ex.filter(*candidate_cloud_);
      sensor_msgs::PointCloud2 ros_candidate_cloud;
      pcl::toROSMsg(*candidate_cloud_, ros_candidate_cloud);
      ros_candidate_cloud.header = msg->header;
      pub_candidate_cloud_.publish(ros_candidate_cloud);
      // check the number of candidate points
      if (candidate_cloud_->points.size() == 0) {
        NODELET_ERROR("No candidate cloud");
        return;
      }
      tree_.setInputCloud(candidate_cloud_);
      if (support_plane_updated_) {
        // Compute assignment between particles and polygons
        NODELET_INFO("polygon updated");
        updateParticlePolygonRelationship(tracker_->getParticles());
      }
      ROS_INFO("start tracker_->compute()");
      tracker_->compute();
      ROS_INFO("done tracker_->compute()");
      Particle result = tracker_->getResult();
      jsk_recognition_msgs::BoundingBoxArray box_array;
      box_array.header = msg->header;
      box_array.boxes.push_back(result.toBoundingBox());
      box_array.boxes[0].header = msg->header;
      pub_result_.publish(box_array);
      geometry_msgs::PoseStamped pose;
      pose.pose = box_array.boxes[0].pose;
      pose.header = box_array.header;
      pub_result_pose_.publish(pose);
    }
    support_plane_updated_ = false;
    ParticleCloud::Ptr particles = tracker_->getParticles();
    // Publish histograms
    publishHistogram(particles, 0, pub_histogram_global_x_, msg->header);
    publishHistogram(particles, 1, pub_histogram_global_y_, msg->header);
    publishHistogram(particles, 2, pub_histogram_global_z_, msg->header);
    publishHistogram(particles, 3, pub_histogram_global_roll_, msg->header);
    publishHistogram(particles, 4, pub_histogram_global_pitch_, msg->header);
    publishHistogram(particles, 5, pub_histogram_global_yaw_, msg->header);
    publishHistogram(particles, 6, pub_histogram_dx_, msg->header);
    publishHistogram(particles, 7, pub_histogram_dy_, msg->header);
    publishHistogram(particles, 8, pub_histogram_dz_, msg->header);
    // Publish particles
    sensor_msgs::PointCloud2 ros_particles;
    pcl::toROSMsg(*particles, ros_particles);
    ros_particles.header = msg->header;
    pub_particles_.publish(ros_particles);
    }
    catch (tf2::TransformException& e) {
      ROS_ERROR("tf exception");
    }
  }
  
  void PlaneSupportedCuboidEstimator::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_INFO("cloudCallback");
    if (!latest_polygon_msg_ || !latest_coefficients_msg_) {
      NODELET_WARN("Not yet polygon is available");
      return;
    }

    if (!tracker_) {
      NODELET_INFO("initTracker");
      // Update polygons_ vector
      polygons_.clear();
      for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
        Polygon::Ptr polygon = Polygon::fromROSMsgPtr(latest_polygon_msg_->polygons[i].polygon);
        polygons_.push_back(polygon);
      }
      try {
        // viewpoint
        tf::StampedTransform transform
          = lookupTransformWithDuration(tf_, sensor_frame_, msg->header.frame_id,
                                        ros::Time(0.0),
                                        ros::Duration(1.0));
        tf::vectorTFToEigen(transform.getOrigin(), viewpoint_);
      }
      catch (tf2::TransformException& e) {
        NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
        return;
      }

      ParticleCloud::Ptr particles = initParticles();
      tracker_.reset(new pcl::tracking::ROSCollaborativeParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleCuboid>);
      tracker_->setCustomSampleFunc(boost::bind(&PlaneSupportedCuboidEstimator::sample, this, _1));
      tracker_->setLikelihoodFunc(boost::bind(&PlaneSupportedCuboidEstimator::likelihood, this, _1, _2));
      tracker_->setParticles(particles);
      tracker_->setParticleNum(particle_num_);
      support_plane_updated_ = false;
      // Publish histograms
      publishHistogram(particles, 0, pub_histogram_global_x_, msg->header);
      publishHistogram(particles, 1, pub_histogram_global_y_, msg->header);
      publishHistogram(particles, 2, pub_histogram_global_z_, msg->header);
      publishHistogram(particles, 3, pub_histogram_global_roll_, msg->header);
      publishHistogram(particles, 4, pub_histogram_global_pitch_, msg->header);
      publishHistogram(particles, 5, pub_histogram_global_yaw_, msg->header);
      publishHistogram(particles, 6, pub_histogram_dx_, msg->header);
      publishHistogram(particles, 7, pub_histogram_dy_, msg->header);
      publishHistogram(particles, 8, pub_histogram_dz_, msg->header);
      // Publish particles
      sensor_msgs::PointCloud2 ros_particles;
      pcl::toROSMsg(*particles, ros_particles);
      ros_particles.header = msg->header;
      pub_particles_.publish(ros_particles);
      
    }
    else {
      ParticleCloud::Ptr particles = tracker_->getParticles();
      Eigen::Vector4f center;
      pcl::compute3DCentroid(*particles, center);
      if (center.norm() > fast_cloud_threshold_) {
        estimate(msg);
      }
    }
  }

  void PlaneSupportedCuboidEstimator::publishHistogram(
    ParticleCloud::Ptr particles, int index, ros::Publisher& pub, const std_msgs::Header& header)
  {
    const double step = 0.001;
    // Lookup min/max
    float max_value = -FLT_MAX;
    float min_value = FLT_MAX;
    for (size_t i = 0; i < particles->points.size(); i++) {
      max_value = std::max(max_value, particles->points[i][index]);
      min_value = std::min(min_value, particles->points[i][index]);
    }
    int num = (max_value - min_value) / step + 1;
    std::vector<unsigned int> bins(num, 0);
    for (size_t i = 0; i < particles->points.size(); i++) {
      float value =  particles->points[i][index];
      const int bin_index = (value - min_value) / step;
      const int min_confirmed_bin_index = std::min(bin_index, num - 1);
      bins[min_confirmed_bin_index] = bins[min_confirmed_bin_index] + 1;
    }
    
    jsk_recognition_msgs::HistogramWithRange histogram;
    histogram.header = header;
    for (size_t i = 0; i < bins.size(); i++) {
      jsk_recognition_msgs::HistogramWithRangeBin bin;
      bin.min_value = i * step + min_value;
      bin.max_value = (i + 1) * step + min_value;
      bin.count = bins[i];
      histogram.bins.push_back(bin);
    }
    pub.publish(histogram);
  }
  
  pcl::tracking::ParticleCuboid PlaneSupportedCuboidEstimator::sample(const pcl::tracking::ParticleCuboid& p)
  {
    pcl::tracking::ParticleCuboid sampled_particle;
    // Motion model is tricky.
    // It's not a tracking problem, so we need to limit noise of motion model
    // The new particle should satisfy:
    //   1. within a polygon
    //   2. within local z range
    //   3. within local pitch/roll range
    //   4. within world yaw range
    //   5. within dx/dy/dz range
    //sampled_particle = p;
    sampled_particle.x = randomGaussian(p.x, step_x_variance_, random_generator_);
    sampled_particle.y = randomGaussian(p.y, step_y_variance_, random_generator_);
    sampled_particle.z = randomGaussian(p.z, step_z_variance_, random_generator_);
    sampled_particle.roll = randomGaussian(p.roll, step_roll_variance_, random_generator_);
    sampled_particle.pitch = randomGaussian(p.pitch, step_pitch_variance_, random_generator_);
    sampled_particle.yaw = randomGaussian(p.yaw, step_yaw_variance_, random_generator_);
    sampled_particle.dx = std::max(randomGaussian(p.dx, step_dx_variance_, random_generator_),
                                   min_dx_);
    sampled_particle.dy = std::max(randomGaussian(p.dy, step_dy_variance_, random_generator_),
                                   min_dy_);
    sampled_particle.dz = std::max(randomGaussian(p.dz, step_dz_variance_, random_generator_),
                                   min_dz_);
    sampled_particle.plane_index = p.plane_index;
    sampled_particle.weight = p.weight;
    return sampled_particle;
  }
  
  void PlaneSupportedCuboidEstimator::likelihood(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
                                                 pcl::tracking::ParticleCuboid& p)
  {
    p.weight = computeLikelihood(p, candidate_cloud_, tree_, viewpoint_,
                                 polygons_, latest_polygon_msg_->likelihood,
                                 config_);
  }
  
  void PlaneSupportedCuboidEstimator::polygonCallback(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coef_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_polygon_msg_ = polygon_msg;
    latest_coefficients_msg_ = coef_msg;
    support_plane_updated_ = true;
  }

  size_t PlaneSupportedCuboidEstimator::chooseUniformRandomPlaneIndex(const std::vector<Polygon::Ptr>& polygons)
  {
    // randomly select plane according to their area
    std::vector<double> area_table(polygons.size());
    double sum = 0;
    for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
      area_table[i] = polygons[i]->area();
      if (use_init_polygon_likelihood_) {
        area_table[i] = area_table[i] * latest_polygon_msg_->likelihood[i];
      }
      sum += area_table[i];
    }
    double val = randomUniform(0, sum, random_generator_);
    double bin_start = 0.0;
    for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
      if (val >= bin_start && val < bin_start + area_table[i]) {
        return i;
      }
      bin_start += area_table[i];
    }
    // Unexpected region here
    NODELET_ERROR("should not reach here, failed to select plane randomly");
    return 0;
  }
  
  pcl::PointCloud<pcl::tracking::ParticleCuboid>::Ptr PlaneSupportedCuboidEstimator::initParticles()
  {
    pcl::PointCloud<pcl::tracking::ParticleCuboid>::Ptr particles (new pcl::PointCloud<pcl::tracking::ParticleCuboid>);
    particles->points.resize(particle_num_);
    std::vector<Polygon::Ptr> polygons(latest_polygon_msg_->polygons.size());
    for (size_t i = 0; i < latest_polygon_msg_->polygons.size(); i++) {
      Polygon::Ptr polygon = Polygon::fromROSMsgPtr(latest_polygon_msg_->polygons[i].polygon);
      polygons[i] = polygon;
    }
    for (size_t i = 0; i < particle_num_; i++) {
      while (true) {
        pcl::tracking::ParticleCuboid p_local;
        size_t plane_i = chooseUniformRandomPlaneIndex(polygons);
        Polygon::Ptr polygon = polygons[plane_i];
        Eigen::Vector3f v = polygon->randomSampleLocalPoint(random_generator_);
        v[2] = randomUniform(init_local_position_z_min_, init_local_position_z_max_,
                             random_generator_);
        p_local.getVector3fMap() = v;
        p_local.roll = randomGaussian(init_local_orientation_roll_mean_, init_local_orientation_roll_variance_, random_generator_);
        p_local.pitch = randomGaussian(init_local_orientation_pitch_mean_, init_local_orientation_pitch_variance_, random_generator_);
        p_local.yaw = randomGaussian(init_local_orientation_yaw_mean_,
                                     init_local_orientation_yaw_variance_,
                                     random_generator_);
        p_local.dx = randomGaussian(init_dx_mean_, init_dx_variance_, random_generator_);
        p_local.dy = randomGaussian(init_dy_mean_, init_dy_variance_, random_generator_);
        p_local.dz = randomGaussian(init_dz_mean_, init_dz_variance_, random_generator_);
        pcl::tracking::ParticleCuboid p_global =  p_local * polygon->coordinates();
        if (use_init_world_position_z_model_) {
          if (p_global.z < init_world_position_z_min_ ||
              p_global.z > init_world_position_z_max_) {
            continue;
          }
        }
        p_global.plane_index = plane_i;
        if (disable_init_pitch_) {
          p_global.pitch = 0;
        }
        if (disable_init_roll_) {
          p_global.roll = 0;
        }
        if (use_global_init_yaw_) {
          p_global.yaw = randomGaussian(init_global_orientation_yaw_mean_,
                                        init_global_orientation_yaw_variance_,
                                        random_generator_);
        }
        
        particles->points[i] = p_global;
        break;
      }
    }
    return particles;
  }

  void PlaneSupportedCuboidEstimator::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = config;
    init_local_position_z_min_ = config.init_local_position_z_min;
    init_local_position_z_max_ = config.init_local_position_z_max;
    use_init_world_position_z_model_ = config.use_init_world_position_z_model;
    init_world_position_z_min_ = config.init_world_position_z_min;
    init_world_position_z_max_ = config.init_world_position_z_max;
    init_local_orientation_roll_mean_ = config.init_local_orientation_roll_mean;
    init_local_orientation_roll_variance_ = config.init_local_orientation_roll_variance;
    init_local_orientation_pitch_mean_ = config.init_local_orientation_pitch_mean;
    init_local_orientation_pitch_variance_ = config.init_local_orientation_pitch_variance;
    init_local_orientation_yaw_mean_ = config.init_local_orientation_yaw_mean;
    init_local_orientation_yaw_variance_ = config.init_local_orientation_yaw_variance;
    use_global_init_yaw_ = config.use_global_init_yaw;
    init_global_orientation_yaw_mean_ = config.init_global_orientation_yaw_mean;
    init_global_orientation_yaw_variance_ = config.init_global_orientation_yaw_variance;
    init_dx_mean_ = config.init_dx_mean;
    init_dx_variance_ = config.init_dx_variance;
    init_dy_mean_ = config.init_dy_mean;
    init_dy_variance_ = config.init_dy_variance;
    init_dz_mean_ = config.init_dz_mean;
    init_dz_variance_ = config.init_dz_variance;
    particle_num_ = config.particle_num;
    step_x_variance_ = config.step_x_variance;
    step_y_variance_ = config.step_y_variance;
    step_z_variance_ = config.step_z_variance;
    step_roll_variance_ = config.step_roll_variance;
    step_pitch_variance_ = config.step_pitch_variance;
    step_yaw_variance_ = config.step_yaw_variance;
    step_dx_variance_ = config.step_dx_variance;
    step_dy_variance_ = config.step_dy_variance;
    step_dz_variance_ = config.step_dz_variance;
    min_dx_ = config.min_dx;
    min_dy_ = config.min_dy;
    min_dz_ = config.min_dz;
    use_init_polygon_likelihood_ = config.use_init_polygon_likelihood;
    fast_cloud_threshold_ = config.fast_cloud_threshold;
    disable_init_roll_ = config.disable_init_roll;
    disable_init_pitch_ = config.disable_init_pitch;
  }

  bool PlaneSupportedCuboidEstimator::resetCallback(std_srvs::EmptyRequest& req,
                                                    std_srvs::EmptyResponse& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_polygon_msg_ = jsk_recognition_msgs::PolygonArray::ConstPtr();
    latest_coefficients_msg_ = jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr();
    tracker_.reset();
    return true;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PlaneSupportedCuboidEstimator, nodelet::Nodelet);
