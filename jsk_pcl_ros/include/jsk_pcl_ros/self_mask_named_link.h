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

#ifndef JSK_PCL_ROS_SELF_MASK_NAMED_LINK_H_
#define JSK_PCL_ROS_SELF_MASK_NAMED_LINK_H_

#include <robot_self_filter/self_mask.h>

namespace robot_self_filter
{
  class SelfMaskNamedLink : public SelfMask<pcl::PointXYZ>
  {
   public:
     /**
     * @brief
     * Construct the filter
     **/
     SelfMaskNamedLink(tf::TransformListener& tf, const std::vector<LinkInfo>& links, const std::string& tf_prefix="")
       : SelfMask<pcl::PointXYZ>(tf, links),
         tf_prefix_(tf_prefix)
       {
       }

     /**
     * @brief
     * Assume subsequent calls to getMaskX() will be in the frame passed to this function.
     * This is override function to use tf_prefix.
     * The frame in which the sensor is located is optional
     **/
     bool assumeFrame(const std_msgs::Header& header) {
       const unsigned int bs = bodies_.size();

       // place the links in the assumed frame
       for (unsigned int i = 0 ; i < bs ; ++i) {
         std::string err;
         if(!tf_.waitForTransform(header.frame_id, tf_prefix_+bodies_[i].name, header.stamp, ros::Duration(.1), ros::Duration(.01), &err)) {
           ROS_ERROR("WaitForTransform timed out from %s to %s after 100ms.  Error string: %s", (tf_prefix_+bodies_[i].name).c_str(), header.frame_id.c_str(), err.c_str());
         }
         // find the transform between the link's frame and the pointcloud frame
         tf::StampedTransform transf;
         try {
           tf_.lookupTransform(header.frame_id, tf_prefix_+bodies_[i].name, header.stamp, transf);
           // tf_.lookupTransform(header.frame_id, tf_prefix_+bodies_[i].name, header.stamp, transf);
         }
         catch (tf::TransformException& ex) {
           transf.setIdentity();
           ROS_ERROR("Unable to lookup transform from %s to %s. Exception: %s", (tf_prefix_+bodies_[i].name).c_str(), header.frame_id.c_str(), ex.what());
           return false;
         }

         // set it for each body; we also include the offset specified in URDF
         bodies_[i].body->setPose(transf * bodies_[i].constTransf);
         bodies_[i].unscaledBody->setPose(transf * bodies_[i].constTransf);
       }
       computeBoundingSpheres();
       return true;
     }

     int getMaskContainmentforNamedLink(const tf::Vector3& pt, const std::string name) const {
       const unsigned int bs = bodies_.size();
       int out = OUTSIDE;
       for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j) {
         if (bodies_[j].name == name) {
           if (bodies_[j].body->containsPoint(pt))
             out = INSIDE;
           break;
         }
       }
       return out;
     }

     /**
     * @brief
     * Get the containment mask (INSIDE or OUTSIDE) value for an individual point.
     * No setup is performed, assumeFrame() should be called before use
     **/
     int getMaskContainmentforNamedLink(double x, double y, double z, const std::string name) const {
       return getMaskContainmentforNamedLink(tf::Vector3(x, y, z), name);
     }

   protected:
     std::string tf_prefix_;
   };
}

#endif
