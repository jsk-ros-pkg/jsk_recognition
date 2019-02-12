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

#ifndef OCTOMAP_SERVER_OCTOMAPSERVER_CONTACT_H_
#define OCTOMAP_SERVER_OCTOMAPSERVER_CONTACT_H_

#include <ros/ros.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>

#define NDEBUG
#include <octomap_server/OctomapServer.h>
#include <jsk_pcl_ros/OcTreeContact.h>
#include <jsk_recognition_msgs/ContactSensorArray.h>
#include <jsk_pcl_ros/self_mask_named_link.h>

using octomap_server::OctomapServer;

namespace jsk_pcl_ros
{
  class OctomapServerContact : public OctomapServer,
                               public jsk_topic_tools::DiagnosticNodelet
  {
   public:
     OctomapServerContact(const ros::NodeHandle& privateNh = ros::NodeHandle("~"));
     virtual ~OctomapServerContact();

     virtual void insertProximityCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
     virtual void insertScanProximity(const tf::Point& sensorOriginTf, const PCLPointCloud& pc);
     virtual void initContactSensor(const ros::NodeHandle& privateNh);
     virtual void insertContactSensor(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg);
     virtual void insertContactSensorCallback(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg);

     virtual void publishAll(const ros::Time& rostime);
     virtual void subscribe() {};
     virtual void unsubscribe() {};

   protected:
     virtual void onInit();
     ros::Publisher m_unknownPointCloudPub, m_umarkerPub;
     message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointProximitySub;
     tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointProximitySub;
     ros::Publisher m_frontierPointCloudPub, m_fromarkerPub;
     message_filters::Subscriber<jsk_recognition_msgs::ContactSensorArray> m_contactSensorSub;
     boost::shared_ptr<tf::MessageFilter<jsk_recognition_msgs::ContactSensorArray> > m_tfContactSensorSub;
     ros::ServiceServer m_octomapBinaryService, m_octomapFullService, m_clearBBXService, m_resetService;

     std_msgs::ColorRGBA m_colorUnknown;
     std_msgs::ColorRGBA m_colorFrontier;

     bool m_publishUnknownSpace;
     double m_offsetVisualizeUnknown;

     bool m_publishFrontierSpace;

     double m_maxRangeProximity;

     double m_occupancyMinX;
     double m_occupancyMaxX;
     double m_occupancyMinY;
     double m_occupancyMaxY;

     bool m_useContactSurface;

     boost::shared_ptr<robot_self_filter::SelfMaskNamedLink> m_selfMask;

     octomap::OcTreeContact* m_octreeContact;
  };
}

#endif
