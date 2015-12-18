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


#ifndef JSK_PCL_ROS_UTILS_SPHERICAL_POINTCLOUD_SIMULATOR_H_
#define JSK_PCL_ROS_UTILS_SPHERICAL_POINTCLOUD_SIMULATOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_pcl_ros_utils/SphericalPointCloudSimulatorConfig.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

namespace jsk_pcl_ros_utils
{
  /** @brief
   * This is a class for jsk_pcl/SphericalPointCloudSimulator nodelet.
   *
   */
  class SphericalPointCloudSimulator: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef SphericalPointCloudSimulatorConfig Config;
    SphericalPointCloudSimulator():
      DiagnosticNodelet("SphericalPointCloudSimulator") {}
  protected:
    /** @brief
     * Initialization function.
     */
    virtual void onInit();

    /** @brief
     * Subscribe another pointcloud to synchronize timestamp.
     */
    virtual void subscribe();

    /** @brief
     * Shutdown subscribers.
     */
    virtual void unsubscribe();

    /** @brief
     * dynamic_reconfigure callback
     */
    virtual void configCallback(Config &config, uint32_t level);
    
    /** @brief
     * Callback function for ~input topic.
     *
     * Pointcloud message is used only for timestamp and frame_id.
     */
    virtual void generate(
      const sensor_msgs::PointCloud2::ConstPtr& msg);

    /** @brief
     * Timer callback for fixed rate publishing. 
     */
    virtual void timerCallback(
      const ros::TimerEvent& event);
    
    /** @brief
     * compute a point according to a parametric model of spherical
     * tilting laser.
     *
     * The normal of scan plane is defined as [0, sin(phi), cos(phi)].
     * Each point of scan is represented as follows respected to the plane:
     *  p = [r*cos(theta), r*sin(theta), 0]
     */
    virtual pcl::PointXYZ getPoint(
      double r, double theta, const Eigen::Affine3f& trans);

    /** @brief
     * Transformation for scan plane specified by phi angle.
     */
    virtual Eigen::Affine3f getPlane(double phi);
    
    boost::mutex mutex_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    /** @brief
     * frame_id of output pointcloud.
     * If it is empty string, frame_id of ~input is copied.
     */
    std::string frame_id_;
    double rotate_velocity_;

    ros::Timer timer_;
    // model parameters
    double r_;
    double min_phi_;
    double max_phi_;
    double scan_range_;
    double fps_;
    int scan_num_;
    
  private:
    
  };
}

#endif
