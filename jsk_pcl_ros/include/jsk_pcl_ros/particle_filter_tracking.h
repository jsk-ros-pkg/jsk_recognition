// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
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

#ifndef JSK_PCL_ROS_PARTICLE_FILTER_TRACKING_H_
#define JSK_PCL_ROS_PARTICLE_FILTER_TRACKING_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>
#include <boost/circular_buffer.hpp>
// pcl
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <jsk_recognition_msgs/SetPointCloud2.h>
#include <jsk_pcl_ros/ParticleFilterTrackingConfig.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/impl/tracking.hpp>
#include <pcl/tracking/impl/particle_filter.hpp>
#include <jsk_recognition_utils/time_util.h>
#include <std_msgs/Float32.h>
#include <jsk_recognition_utils/pcl_util.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/TrackerStatus.h>
// This namespace follows PCL coding style
namespace pcl
{
  namespace tracking
  {
    // hack pcl::tracking
    // original tracker assumes that the number of reference points is smaller
    // than the number of input.
    // ReversedParticleFilterTracker assumues that the number of reference
    // points is greater than the number of input.
    // So we need to change:
    //   1) transform input pointcloud during weight phase with inverse of each particles
    //      particle should keep meaning the pose of reference for simplicity.
    template <typename PointInT, typename StateT>
    class ReversedParticleFilterTracker: public ParticleFilterTracker<PointInT, StateT>
    {
    public:
      using Tracker<PointInT, StateT>::tracker_name_;
      using Tracker<PointInT, StateT>::search_;
      using Tracker<PointInT, StateT>::input_;
      using Tracker<PointInT, StateT>::indices_;
      using Tracker<PointInT, StateT>::getClassName;
      using ParticleFilterTracker<PointInT, StateT>::particles_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_;
      using ParticleFilterTracker<PointInT, StateT>::change_counter_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_resolution_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_interval_;
      using ParticleFilterTracker<PointInT, StateT>::use_change_detector_;
      using ParticleFilterTracker<PointInT, StateT>::alpha_;
      using ParticleFilterTracker<PointInT, StateT>::changed_;
      using ParticleFilterTracker<PointInT, StateT>::ref_;
      using ParticleFilterTracker<PointInT, StateT>::coherence_;
      using ParticleFilterTracker<PointInT, StateT>::use_normal_;
      using ParticleFilterTracker<PointInT, StateT>::particle_num_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_filter_;
      using ParticleFilterTracker<PointInT, StateT>::transed_reference_vector_;
      //using ParticleFilterTracker<PointInT, StateT>::calcLikelihood;
      using ParticleFilterTracker<PointInT, StateT>::normalizeWeight;
      using ParticleFilterTracker<PointInT, StateT>::initParticles;
      using ParticleFilterTracker<PointInT, StateT>::normalizeParticleWeight;
      using ParticleFilterTracker<PointInT, StateT>::calcBoundingBox;

      typedef Tracker<PointInT, StateT> BaseClass;
      
      typedef typename Tracker<PointInT, StateT>::PointCloudIn PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef typename Tracker<PointInT, StateT>::PointCloudState PointCloudState;
      typedef typename PointCloudState::Ptr PointCloudStatePtr;
      typedef typename PointCloudState::ConstPtr PointCloudStateConstPtr;

      typedef PointCoherence<PointInT> Coherence;
      typedef boost::shared_ptr< Coherence > CoherencePtr;
      typedef boost::shared_ptr< const Coherence > CoherenceConstPtr;

      typedef PointCloudCoherence<PointInT> CloudCoherence;
      typedef boost::shared_ptr< CloudCoherence > CloudCoherencePtr;
      typedef boost::shared_ptr< const CloudCoherence > CloudCoherenceConstPtr;

      // call initCompute only if reference pointcloud is updated
      inline void
      setReferenceCloud (const PointCloudInConstPtr &ref)
      {
        ref_ = ref;
        if (coherence_) {
          coherence_->setTargetCloud (ref_);
          coherence_->initCompute ();
        }
        else {
          PCL_ERROR("coherence_ is not yet available!");
        }
      }


    protected:
      std::vector<PointCloudInPtr> transed_input_vector_;
      
      virtual bool initCompute()
      {
        if (!Tracker<PointInT, StateT>::initCompute ())
        {
          PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
          return (false);
        }
        
        // allocate pointclouds in transed_input_vector_ instead of transed_reference_vector_
        if (transed_input_vector_.empty ())
        {
          //std::cout << "initializing " << particle_num_ << " input" << std::endl;
          // only one time allocation
          transed_input_vector_.resize (particle_num_ + 1);
          for (int i = 0; i < particle_num_ + 1; i++)
          {
            transed_input_vector_[i] = PointCloudInPtr (new PointCloudIn ());
          }
        }

        // set reference instead of input
        // if (coherence_) {
        //   coherence_->setTargetCloud (ref_);
        // }
          
        if (!change_detector_)
          change_detector_ = boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointInT> >(new pcl::octree::OctreePointCloudChangeDetector<PointInT> (change_detector_resolution_));
          
        if (!particles_ || particles_->points.empty ())
          initParticles (true);
        return (true);
      }

      // only is computation without normal supported
      void computeTransformedPointCloudWithoutNormal
      (const StateT& hypothesis, PointCloudIn &cloud)
        {
          const Eigen::Affine3f trans = this->toEigenMatrix (hypothesis);
          // destructively assigns to cloud
          pcl::transformPointCloud<PointInT> (*input_, cloud, trans);
        }

      
      virtual void weight()
      {
        changed_ = true;
        if (!use_normal_)
        {
          for (size_t i = 0; i < particles_->points.size (); i++)
          {
            //std::cout << "processing " << i << " particle: " << particles_->points[i].weight << std::endl;
            // compute `inverse` of particle
            StateT inverse_particle;
            Eigen::Affine3f trans = particles_->points[i].toEigenMatrix();
            Eigen::Affine3f inverse_trans = trans.inverse();
            inverse_particle = StateT::toState(inverse_trans);
            computeTransformedPointCloudWithoutNormal (inverse_particle, *transed_input_vector_[i]);
            IndicesPtr indices;
            coherence_->compute (transed_input_vector_[i], indices, particles_->points[i].weight);
          }
        }
        else
        {
          for (size_t i = 0; i < particles_->points.size (); i++)
          {
            StateT inverse_particle;
            Eigen::Affine3f trans = particles_->points[i].toEigenMatrix();
            Eigen::Affine3f inverse_trans = trans.inverse();
            inverse_particle = StateT::toState(inverse_trans);
            IndicesPtr indices (new std::vector<int>);
            this->computeTransformedPointCloudWithNormal (inverse_particle, *indices, *transed_input_vector_[i]);
            coherence_->compute (transed_input_vector_[i], indices, particles_->points[i].weight);
          }
        }
  
        normalizeWeight ();
        
      }
    private:
    };
    
    template <typename PointInT, typename StateT>
    class ReversedParticleFilterOMPTracker: public ReversedParticleFilterTracker<PointInT, StateT>
    {
    public:
      using Tracker<PointInT, StateT>::tracker_name_;
      using Tracker<PointInT, StateT>::search_;
      using Tracker<PointInT, StateT>::input_;
      using Tracker<PointInT, StateT>::indices_;
      using Tracker<PointInT, StateT>::getClassName;
      using ParticleFilterTracker<PointInT, StateT>::particles_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_;
      using ParticleFilterTracker<PointInT, StateT>::change_counter_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_resolution_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_interval_;
      using ParticleFilterTracker<PointInT, StateT>::use_change_detector_;
      using ParticleFilterTracker<PointInT, StateT>::alpha_;
      using ParticleFilterTracker<PointInT, StateT>::changed_;
      using ParticleFilterTracker<PointInT, StateT>::ref_;
      using ParticleFilterTracker<PointInT, StateT>::coherence_;
      using ParticleFilterTracker<PointInT, StateT>::use_normal_;
      using ParticleFilterTracker<PointInT, StateT>::particle_num_;
      using ParticleFilterTracker<PointInT, StateT>::change_detector_filter_;
      using ParticleFilterTracker<PointInT, StateT>::transed_reference_vector_;
      //using ParticleFilterTracker<PointInT, StateT>::calcLikelihood;
      using ParticleFilterTracker<PointInT, StateT>::normalizeWeight;
      using ParticleFilterTracker<PointInT, StateT>::initParticles;
      using ParticleFilterTracker<PointInT, StateT>::normalizeParticleWeight;
      using ParticleFilterTracker<PointInT, StateT>::calcBoundingBox;

      typedef Tracker<PointInT, StateT> BaseClass;
      
      typedef typename Tracker<PointInT, StateT>::PointCloudIn PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef typename Tracker<PointInT, StateT>::PointCloudState PointCloudState;
      typedef typename PointCloudState::Ptr PointCloudStatePtr;
      typedef typename PointCloudState::ConstPtr PointCloudStateConstPtr;

      typedef PointCoherence<PointInT> Coherence;
      typedef boost::shared_ptr< Coherence > CoherencePtr;
      typedef boost::shared_ptr< const Coherence > CoherenceConstPtr;

      typedef PointCloudCoherence<PointInT> CloudCoherence;
      typedef boost::shared_ptr< CloudCoherence > CloudCoherencePtr;
      typedef boost::shared_ptr< const CloudCoherence > CloudCoherenceConstPtr;
      using ReversedParticleFilterTracker<PointInT, StateT>::transed_input_vector_;
    protected:

      unsigned int threads_;
      
    public:
      ReversedParticleFilterOMPTracker (unsigned int nr_threads = 0)
        : ReversedParticleFilterTracker<PointInT, StateT> ()
        , threads_ (nr_threads)
      {
        tracker_name_ = "ReversedParticleFilterOMPTracker";
      }

      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

    protected:
      
      virtual void weight()
      {
        changed_ = true;
        if (!use_normal_)
        {
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
          for (size_t i = 0; i < particles_->points.size (); i++)
          {
            //std::cout << "processing " << i << " particle: " << particles_->points[i].weight << std::endl;
            // compute `inverse` of particle
            StateT inverse_particle;
            Eigen::Affine3f trans = particles_->points[i].toEigenMatrix();
            Eigen::Affine3f inverse_trans = trans.inverse();
            inverse_particle = StateT::toState(inverse_trans);
            this->computeTransformedPointCloudWithoutNormal (inverse_particle, *transed_input_vector_[i]);
            //computeTransformedPointCloudWithoutNormal (particles_->points[i], *transed_input_vector_[i]);
            IndicesPtr indices;
            coherence_->compute (transed_input_vector_[i], indices, particles_->points[i].weight);
            //std::cout << "processing " << i << " particle: " << particles_->points[i].weight << std::endl;
            //std::cout << inverse_particle << std::endl;
          }
        }
        else
        {
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
          for (size_t i = 0; i < particles_->points.size (); i++)
          {
            StateT inverse_particle;
            Eigen::Affine3f trans = particles_->points[i].toEigenMatrix();
            Eigen::Affine3f inverse_trans = trans.inverse();
            inverse_particle = StateT::toState(inverse_trans);
            IndicesPtr indices (new std::vector<int>);
            this->computeTransformedPointCloudWithNormal (inverse_particle, *indices, *transed_input_vector_[i]);
            coherence_->compute (transed_input_vector_[i], indices, particles_->points[i].weight);
          }
        }
  
        normalizeWeight ();
        for (size_t i = 0; i < particles_->points.size (); i++)
        {
          //std::cout << "normalized " << i << " particle: " << particles_->points[i].weight << std::endl;
        }
      }
    private:
    };

    template <typename PointInT>
    class CachedApproxNearestPairPointCloudCoherence: public ApproxNearestPairPointCloudCoherence<PointInT>
    {
    public:
      typedef typename ApproxNearestPairPointCloudCoherence<PointInT>::PointCoherencePtr PointCoherencePtr;
      typedef typename ApproxNearestPairPointCloudCoherence<PointInT>::PointCloudInConstPtr PointCloudInConstPtr;
      //using NearestPairPointCloudCoherence<PointInT>::search_;
      using ApproxNearestPairPointCloudCoherence<PointInT>::maximum_distance_;
      using ApproxNearestPairPointCloudCoherence<PointInT>::target_input_;
      using ApproxNearestPairPointCloudCoherence<PointInT>::point_coherences_;
      using ApproxNearestPairPointCloudCoherence<PointInT>::coherence_name_;
      using ApproxNearestPairPointCloudCoherence<PointInT>::new_target_;
      using ApproxNearestPairPointCloudCoherence<PointInT>::getClassName;
      using ApproxNearestPairPointCloudCoherence<PointInT>::search_;
      
      /** \brief empty constructor */
      CachedApproxNearestPairPointCloudCoherence (const double bin_x,
                                                  const double bin_y,
                                                  const double bin_z) : 
        ApproxNearestPairPointCloudCoherence<PointInT> (),
        bin_x_(bin_x), bin_y_(bin_y), bin_z_(bin_z)
      {
        coherence_name_ = "CachedApproxNearestPairPointCloudCoherence";
      }
      
    protected:
      /** \brief compute the nearest pairs and compute coherence using point_coherences_ */
      virtual void
      computeCoherence (const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices, float &w_j)
      {
        boost::mutex::scoped_lock lock(cache_mutex_);
        double val = 0.0;
        //for (size_t i = 0; i < indices->size (); i++)
        for (size_t i = 0; i < cloud->points.size (); i++)
        {
          PointInT input_point = cloud->points[i];
          int xi, yi, zi;
          computeBin(input_point.getVector3fMap(), xi, yi, zi);
          int k_index;
          if (checkCache(xi, yi, zi)) {
            k_index = getCachedIndex(xi, yi, zi);
          }
          else {
            float k_distance = 0.0; // dummy
            search_->approxNearestSearch(input_point, k_index, k_distance);
            registerCache(k_index, xi, yi, zi);
          }
          PointInT target_point = target_input_->points[k_index];
          float dist = (target_point.getVector3fMap() - input_point.getVector3fMap()).norm();
          if (dist < maximum_distance_)
          {
            double coherence_val = 1.0;
            for (size_t i = 0; i < point_coherences_.size (); i++)
            {
              PointCoherencePtr coherence = point_coherences_[i];  
              double w = coherence->compute (input_point, target_point);
              coherence_val *= w;
            }
            val += coherence_val;
          }
        }
        w_j = - static_cast<float> (val);
        //ROS_INFO("hit: %d", counter);
      }

      virtual void computeBin(
        const Eigen::Vector3f& p, int& xi, int& yi, int& zi)
      {
        xi = (int)(p[0]/bin_x_);
        yi = (int)(p[1]/bin_y_);
        zi = (int)(p[2]/bin_z_);
      }
      
      virtual void registerCache(int k_index, int bin_x, int bin_y, int bin_z)
      {
        //boost::mutex::scoped_lock lock(cache_mutex_);
        if (cache_.find(bin_x) == cache_.end()) {
          cache_[bin_x] = std::map<int, std::map<int, int> >();
        }
        if (cache_[bin_x].find(bin_y) == cache_[bin_x].end()) {
          cache_[bin_x][bin_y] = std::map<int, int>();
        }
        cache_[bin_x][bin_y][bin_z] = k_index;
      }
      
      virtual int getCachedIndex(int bin_x, int bin_y, int bin_z)
      {
        //boost::mutex::scoped_lock lock(cache_mutex_);
        return cache_[bin_x][bin_y][bin_z];
      }
      
      virtual bool checkCache(int bin_x, int bin_y, int bin_z)
      {
        //boost::mutex::scoped_lock lock(cache_mutex_);
        if (cache_.find(bin_x) == cache_.end()) {
          return false;
        }
        else {
          if (cache_[bin_x].find(bin_y) == cache_[bin_x].end()) {
            return false;
          }
          else {
            if (cache_[bin_x][bin_y].find(bin_z) == cache_[bin_x][bin_y].end()) {
              return false;
            }
            else {
              return true;
            }
          }
        }
      }
      
      virtual void clearCache()
      {
        boost::mutex::scoped_lock lock(cache_mutex_);
        cache_ = CacheMap();
      }

      virtual bool initCompute()
      {
        if (!ApproxNearestPairPointCloudCoherence<PointInT>::initCompute ())
        {
          PCL_ERROR ("[pcl::%s::initCompute] PointCloudCoherence::Init failed.\n", getClassName ().c_str ());
          //deinitCompute ();
          return false;
        }
        clearCache();
        return true;
      }
      
      //typename boost::shared_ptr<pcl::search::Octree<PointInT> > search_;
      typedef std::map<int, std::map<int, std::map<int, int> > > CacheMap;
      CacheMap cache_;
      boost::mutex cache_mutex_;
      double bin_x_;
      double bin_y_;
      double bin_z_;
      
    };

    
    template <typename PointInT>
    class OrganizedNearestPairPointCloudCoherence: public NearestPairPointCloudCoherence<PointInT>
    {
    public:
      using NearestPairPointCloudCoherence<PointInT>::target_input_;
      using NearestPairPointCloudCoherence<PointInT>::new_target_;
      using NearestPairPointCloudCoherence<PointInT>::getClassName;
      typename boost::shared_ptr<pcl::search::OrganizedNeighbor<PointInT> > search_;
    protected:
      virtual bool initCompute ()
      {
        if (!PointCloudCoherence<PointInT>::initCompute ())
        {
          PCL_ERROR ("[pcl::%s::initCompute] PointCloudCoherence::Init failed.\n", getClassName ().c_str ());
          //deinitCompute ();
          return (false);
        }
        
        // initialize tree
        if (!search_)
          search_.reset (new pcl::search::OrganizedNeighbor<PointInT> (false));
        
        if (new_target_ && target_input_)
        {
          search_->setInputCloud (target_input_);
          if (!search_->isValid())
            return false;
          new_target_ = false;
        }
      
        return true;
      }
    };
  }  
}

using namespace pcl::tracking;
namespace jsk_pcl_ros
{
  class ParticleFilterTracking: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef ParticleFilterTrackingConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::BoundingBox > SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2 > SyncChangePolicy;
    typedef ParticleFilterTracker<PointT, ParticleXYZRPY>::PointCloudStatePtr
    PointCloudStatePtr;
    ParticleFilterTracking(): timer_(10), distance_error_buffer_(100), angle_error_buffer_(100), no_move_buffer_(10) {}
  protected:
    pcl::PointCloud<PointT>::Ptr cloud_pass_;
    pcl::PointCloud<PointT>::Ptr cloud_pass_downsampled_;
    pcl::PointCloud<PointT>::Ptr target_cloud_;

    //boost::shared_ptr<ParticleFilterTracker<PointT, ParticleXYZRPY> > tracker_;
    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleXYZRPY> > tracker_;
    boost::shared_ptr<ReversedParticleFilterOMPTracker<PointT, ParticleXYZRPY> > reversed_tracker_;
    //boost::shared_ptr<ReversedParticleFilterTracker<PointT, ParticleXYZRPY> > reversed_tracker_;
    boost::mutex mtx_;
    bool new_cloud_;
    bool track_target_set_;
    bool align_box_;
    bool change_frame_;
    std::string frame_id_;
    std::string base_frame_id_;
    std::string track_target_name_;
    ros::Time stamp_;
    ros::Time prev_stamp_;
    tf::Transform reference_transform_;

    ros::Subscriber sub_;
    ros::Subscriber sub_update_model_;
    ros::Subscriber sub_update_with_marker_model_;
    ros::Publisher pub_latest_time_;
    ros::Publisher pub_average_time_;
    ros::Publisher pub_rms_distance_;
    ros::Publisher pub_rms_angle_;
    ros::Publisher pub_velocity_;
    ros::Publisher pub_velocity_norm_;
    ros::Publisher pub_no_move_;
    ros::Publisher pub_no_move_raw_;
    ros::Publisher pub_skipped_;
    ros::Publisher pub_change_cloud_marker_;
    ros::Publisher pub_tracker_status_;
    jsk_recognition_utils::WallDurationTimer timer_;
    Eigen::Affine3f initial_pose_;
    boost::circular_buffer<double> distance_error_buffer_;
    boost::circular_buffer<double> angle_error_buffer_;

    jsk_recognition_utils::SeriesedBoolean no_move_buffer_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> sub_box_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_change_cloud_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<SyncChangePolicy> > change_sync_;
    ros::Publisher particle_publisher_;
    ros::Publisher track_result_publisher_;
    ros::Publisher pose_stamped_publisher_;
    ros::ServiceServer renew_model_srv_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    tf::TransformListener* listener_;
    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    bool use_change_detection_;
    int max_particle_num_;
    double delta_;
    double epsilon_;
    int iteration_num_;
    double resample_likelihood_thr_;
    ParticleXYZRPY bin_size_;
    ParticleXYZRPY prev_result_;
    int counter_;
    std::vector<double> default_step_covariance_;
    bool reversed_;
    bool not_use_reference_centroid_;
    bool not_publish_tf_;
    int marker_to_pointcloud_sampling_nums_;
    double static_velocity_thr_;
    double change_cloud_near_threshold_;
    virtual void config_callback(Config &config, uint32_t level);
    virtual void publish_particles();
    virtual void publish_result();
    virtual std::string reference_frame_id();
    virtual void reset_tracking_target_model(
      const pcl::PointCloud<PointT>::ConstPtr &new_target_cloud);
    virtual tf::Transform change_pointcloud_frame(
      pcl::PointCloud<PointT>::Ptr cloud);
    virtual double rms(boost::circular_buffer<double>& buffer) {
      double res = 0.0;
      for (size_t i = 0; i < buffer.size(); i++) {
        res += buffer[i] * buffer[i];
      }
      return sqrt(res / buffer.size());
    }
    virtual void cloud_cb(const sensor_msgs::PointCloud2 &pc);
    virtual void cloud_change_cb(const sensor_msgs::PointCloud2::ConstPtr &pc, const sensor_msgs::PointCloud2::ConstPtr &chnage_cloud);
    virtual bool renew_model_cb(
      jsk_recognition_msgs::SetPointCloud2::Request &req,
      jsk_recognition_msgs::SetPointCloud2::Response &response);
    virtual void renew_model_with_box_topic_cb(
      const sensor_msgs::PointCloud2::ConstPtr &pc_ptr,
      const jsk_recognition_msgs::BoundingBox::ConstPtr &bb_ptr);
    virtual void renew_model_topic_cb(const sensor_msgs::PointCloud2 &pc);
    virtual void renew_model_with_marker_topic_cb(const visualization_msgs::Marker &marker);

    virtual void publish_tracker_status(const std_msgs::Header& header,
                                        const bool is_tracking);

    
    ////////////////////////////////////////////////////////
    // Wrap particle filter methods
    ////////////////////////////////////////////////////////
    virtual void tracker_set_trans(const Eigen::Affine3f& trans);
    virtual void tracker_set_step_noise_covariance(
      const std::vector<double>& covariance);
    virtual void tracker_set_initial_noise_covariance(
      const std::vector<double>& covariance);
    virtual void tracker_set_initial_noise_mean(
      const std::vector<double>& mean);
    virtual void tracker_set_iteration_num(const int num);
    virtual void tracker_set_particle_num(const int num);
    virtual void tracker_set_resample_likelihood_thr(double thr);
    virtual void tracker_set_use_normal(bool use_normal);
    virtual void tracker_set_cloud_coherence(
      ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence);
    virtual void tracker_set_maximum_particle_num(int num);
    virtual void tracker_set_delta(double delta);
    virtual void tracker_set_epsilon(double epsilon);
    virtual void tracker_set_bin_size(const ParticleXYZRPY bin_size);
    virtual ParticleFilterTracker<PointT, ParticleXYZRPY>::PointCloudStatePtr
    tracker_get_particles();
    virtual ParticleXYZRPY tracker_get_result();
    virtual Eigen::Affine3f tracker_to_eigen_matrix(
      const ParticleXYZRPY& result);
    virtual pcl::PointCloud<PointT>::ConstPtr tracker_get_reference_cloud();
    virtual void tracker_set_reference_cloud(pcl::PointCloud<PointT>::Ptr ref);
    virtual void tracker_reset_tracking();
    virtual void tracker_set_input_cloud(pcl::PointCloud<PointT>::Ptr input);
    virtual void tracker_compute();

    virtual void subscribe() {}
    virtual void unsubscribe() {}    
    
  private:
    virtual void onInit();

  };
}

#endif
