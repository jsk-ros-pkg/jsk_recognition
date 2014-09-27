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
 *   * Neither the name of the Willow Garage nor the names of its
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


#ifndef CONNECTION_BASED_NODELET_H_
#define CONNECTION_BASED_NODELET_H_

#include <pcl_ros/pcl_nodelet.h>

namespace jsk_pcl_ros
{
  class ConnectionBasedNodelet: public pcl_ros::PCLNodelet
  {
  public:
    ConnectionBasedNodelet(): subscribed_(false) { }
  protected:
    virtual void connectionCallback(const ros::SingleSubscriberPublisher& pub);
    virtual void subscribe() = 0;
    virtual void unsubscribe() = 0;
    
    template<class T> ros::Publisher
    advertise(ros::NodeHandle& nh,
              std::string topic, int queue_size)
    {
      ros::SubscriberStatusCallback connect_cb
        = boost::bind( &ConnectionBasedNodelet::connectionCallback, this, _1);
      ros::SubscriberStatusCallback disconnect_cb
        = boost::bind( &ConnectionBasedNodelet::connectionCallback, this, _1);
      ros::Publisher ret = nh.advertise<T>(topic, queue_size,
                                           connect_cb,
                                           disconnect_cb);
      publishers_.push_back(ret);
      return ret;
    }
    
    boost::mutex connection_mutex_;
    std::vector<ros::Publisher> publishers_;
    bool subscribed_;
  private:
    
  };
}

#endif
