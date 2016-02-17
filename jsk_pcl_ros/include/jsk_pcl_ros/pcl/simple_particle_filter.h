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


#ifndef JSK_PCL_ROS_ROS_COLLABORATIVE_PARTICLE_FILTER_H_
#define JSK_PCL_ROS_ROS_COLLABORATIVE_PARTICLE_FILTER_H_

#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/impl/particle_filter.hpp>

namespace pcl
{
  namespace tracking
  {
        // Particle filter class which is friendly collaborative with ROS programs.
    // Default behavior is same to the ParticleFilterTracker but you can customize
    // the behavior from outer of the class.
    template <typename PointInT, typename StateT>
    class ROSCollaborativeParticleFilterTracker: public ParticleFilterTracker<PointInT, StateT>
    {
    public:
      using Tracker<PointInT, StateT>::input_;
      using ParticleFilterTracker<PointInT, StateT>::particles_;
      using ParticleFilterTracker<PointInT, StateT>::changed_;
      typedef boost::shared_ptr<ROSCollaborativeParticleFilterTracker> Ptr;
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

      typedef boost::function<StateT (const StateT&)> CustomSampleFunc;
      typedef boost::function<void (PointCloudInConstPtr, StateT&)> CustomLikelihoodFunc;
      ROSCollaborativeParticleFilterTracker():
        ParticleFilterTracker<PointInT, StateT>()
      {
        motion_ratio_ = 0.0;
        changed_ = true;
      }

      void setParticles(PointCloudStatePtr particles)
      {
        particles_ = particles;
      }

      void setCustomSampleFunc(CustomSampleFunc f)
      {
        custom_sample_func_ = f;
      }
      
      void setLikelihoodFunc(CustomLikelihoodFunc f)
      {
        custom_likelihood_func_ = f;
      }
      
    protected:
      bool initCompute()
      {
        // Do nothing
        return true;
      }
      
      void weight()
      {
        if (!particles_) {
          std::cerr << "no particles" << std::endl;
        }
        if (!input_) {
          std::cerr << "no input pointcloud" << std::endl;
        }

#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (size_t i = 0; i < particles_->points.size (); i++) {
          custom_likelihood_func_ (input_, particles_->points[i]);
        }
        normalizeWeight();
      }
      
      void 
      resample()
      {
        std::vector<int> a (particles_->points.size ());
        std::vector<double> q (particles_->points.size ());
        this->genAliasTable (a, q, particles_);
  
        // memoize the original list of particles
        
        PointCloudStatePtr origparticles = particles_;
        particles_.reset(new PointCloudState());
        particles_->points.reserve(origparticles->points.size() + 1); // particle_num_??
        // the first particle, it is a just copy of the maximum result
        // StateT p = representative_state_;
        // particles_->points.push_back (p);
  
        // with motion
        int motion_num = static_cast<int> (particles_->points.size ()) * static_cast<int> (motion_ratio_);
        for ( int i = 1; i < motion_num; i++ ) {
          int target_particle_index = sampleWithReplacement (a, q);
          StateT p = origparticles->points[target_particle_index];
          p = custom_sample_func_(p);
          p = p + motion_;
          particles_->points.push_back (p);
        }
  
        // no motion
        for ( int i = motion_num; i < particle_num_; i++ ) {
          int target_particle_index = sampleWithReplacement (a, q);
          StateT p = origparticles->points[target_particle_index];
          // add noise using gaussian
          p = custom_sample_func_(p);
          particles_->points.push_back (p);
        }
      }

      bool initParticles(bool)
      {
        // Do nothing
        return true;
      }

      void computeTracking()
      {
        // r->w->u ?
        // w->u->r ?
        for (int i = 0; i < iteration_num_; i++) {
          resample();
          weight();
          update();
        }
      }

      void normalizeWeight()
      {
        double n = 0.0;
        for (size_t i = 0; i < particles_->points.size(); i++) {
          n = particles_->points[i].weight + n;
        }
        if (n != 0.0) {
          for (size_t i = 0; i < particles_->points.size(); i++) {
            particles_->points[i].weight = particles_->points[i].weight / n;
          }
        }
        else {
          for (size_t i = 0; i < particles_->points.size(); i++) {
            particles_->points[i].weight = 1.0 / particles_->points.size();
          }
        }
      }
      
      using ParticleFilterTracker<PointInT, StateT>::iteration_num_;
      using ParticleFilterTracker<PointInT, StateT>::update;
      using ParticleFilterTracker<PointInT, StateT>::representative_state_;
      using ParticleFilterTracker<PointInT, StateT>::motion_ratio_;
      using ParticleFilterTracker<PointInT, StateT>::particle_num_;
      using ParticleFilterTracker<PointInT, StateT>::motion_;
      using ParticleFilterTracker<PointInT, StateT>::sampleWithReplacement;
      CustomSampleFunc custom_sample_func_;
      CustomLikelihoodFunc custom_likelihood_func_;
    private:
    };
  }
}

#endif
