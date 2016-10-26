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
 *     disclaimer in the documentation and/or other materials provided
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

#ifndef JSK_PCL_ROS_REARRANGE_BOUNDING_BOX_H
#define JSK_PCL_ROS_REARRANGE_BOUNDING_BOX_H

#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <dynamic_reconfigure/server.h>
#include <tf2/LinearMath/Quaternion.h>

#include "jsk_pcl_ros/RearrangeBoundingBoxConfig.h"

namespace jsk_pcl_ros
{
  class RearrangeBoundingBox : public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    RearrangeBoundingBox() { }
    typedef jsk_pcl_ros::RearrangeBoundingBoxConfig Config;

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void rearrangeBoundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array);
    virtual void subscribe();
    virtual void unsubscribe();

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    void configCallback (Config &config, uint32_t level);

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_bounding_box_array_;
    ros::Publisher pub_bouding_box_array_;
    boost::mutex mutex_;

    double scale_x_;
    double scale_y_;
    double scale_z_;

    double offset_x_;
    double offset_y_;
    double offset_z_;

    double rotate_x_;
    double rotate_y_;
    double rotate_z_;

    tf2::Quaternion q_;
  private:
  };
}

#endif
