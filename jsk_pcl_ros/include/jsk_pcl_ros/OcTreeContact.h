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

#ifndef OCTOMAP_OCTREECONTACT_H_
#define OCTOMAP_OCTREECONTACT_H_


#include <octomap/OcTree.h>

namespace octomap {
  /**
   * @brief
   * This is a inherited class of OcTree.
   * The probability of contact sensor model is defined.
   **/
  class OcTreeContact : public OcTree
  {
   public:
     inline bool isNodeFree(const OcTreeNode* occupancy_node) const {
       return (occupancy_node->getLogOdds() <= free_prob_thres_log_);
     }
     inline bool isNodeFree(const OcTreeNode& occupancy_node) const {
       return (occupancy_node.getLogOdds() <= free_prob_thres_log_);
     }
     inline bool isNodeUnknown(const OcTreeNode* occupancy_node) const {
       return (!isNodeOccupied(occupancy_node) and !isNodeFree(occupancy_node));
     }
     inline bool isNodeUnknown(const OcTreeNode& occupancy_node) const {
       return (!isNodeOccupied(occupancy_node) and !isNodeFree(occupancy_node));
     }

     void setFreeThres(double prob) { free_prob_thres_log_ = logodds(prob); }
     void setProbHitContactSensor(double prob) { prob_hit_contact_sensor_log_ = logodds(prob); }
     void setProbMissContactSensor(double prob) { prob_miss_contact_sensor_log_ = logodds(prob); }

     double getFreeThres() const { return probability(free_prob_thres_log_); }
     float getFreeThresLog() const { return free_prob_thres_log_; }

     double getProbHitContactSensor() const { return probability(prob_hit_contact_sensor_log_); }
     float getProbHitContactSensorLog() const { return prob_hit_contact_sensor_log_; }
     double getProbMissContactSensor() const { return probability(prob_miss_contact_sensor_log_); }
     float getProbMissContactSensorLog() const { return prob_miss_contact_sensor_log_; }

     float prob_hit_contact_sensor_log_;
     float prob_miss_contact_sensor_log_;
     float free_prob_thres_log_;

     OcTreeContact(double resolution) : OcTree(resolution)
     {
       setOccupancyThres(0.7);   // = 0.0 in logodds
       setFreeThres(0.3);   // = 0.0 in logodds
       setProbMiss(0.2);         // = -0.4 in logodds
       setProbHitContactSensor(0.7);
       setProbMissContactSensor(0.1);
     }
   };
} // end namespace

#endif
