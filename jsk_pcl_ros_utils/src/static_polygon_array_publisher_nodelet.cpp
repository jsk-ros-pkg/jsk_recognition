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

#include <jsk_pcl_ros_utils/static_polygon_array_publisher.h>
#include <pluginlib/class_list_macros.h>
#include <jsk_topic_tools/rosparam_utils.h>

namespace jsk_pcl_ros_utils
{

  void StaticPolygonArrayPublisher::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("use_periodic", use_periodic_, false);
    pnh_->param("use_message", use_message_, false);
    pnh_->param("use_trigger", use_trigger_, false);
    pnh_->param("periodic_rate", periodic_rate_, 10.0);
    
    bool frame_id_read_p
      = jsk_topic_tools::readVectorParameter(*pnh_, "frame_ids",
                                             frame_ids_);
    if (!frame_id_read_p) {
      NODELET_FATAL("failed to read frame_ids from ~frame_ids");
      return;
    }
    
    bool polygon_read_p = readPolygonArray("polygon_array");
    if (!polygon_read_p) {
      NODELET_FATAL("failed to read polygons from ~polygon_array");
      return;
    }

    if (frame_ids_.size() != polygons_.polygons.size()) {
      NODELET_FATAL("the size of frame_ids(%lu) does not match the size of polygons(%lu)",
                    frame_ids_.size(), polygons_.polygons.size());
      return;
    }
    else {
      for (size_t i = 0; i < frame_ids_.size(); i++) {
        polygons_.polygons[i].header.frame_id = frame_ids_[i];
        coefficients_.coefficients[i].header.frame_id = frame_ids_[i];
      }
    }
    
    if (!use_periodic_ && !use_message_ && !use_trigger_) {
      NODELET_FATAL("~use_periodic, ~use_trigger nor ~use_message is not true");
      return;
    }
    polygons_.header.frame_id = frame_ids_[0];
    coefficients_.header.frame_id = frame_ids_[0];

    if (!use_periodic_) {
      polygon_pub_ = advertise<jsk_recognition_msgs::PolygonArray>(
        *pnh_, "output_polygons", 1);
      coefficients_pub_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
        *pnh_, "output_coefficients", 1);
    }
    else {
      polygon_pub_ = pnh_->advertise<jsk_recognition_msgs::PolygonArray>(
        "output_polygons", 1);
      coefficients_pub_ = pnh_->advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
        "output_coefficients", 1);
      subscribe();
      timer_ = pnh_->createTimer(ros::Duration(1.0 / periodic_rate_), &StaticPolygonArrayPublisher::timerCallback, this);
    }
    onInitPostProcess();
  }

  void StaticPolygonArrayPublisher::subscribe()
  {
    if (use_message_) {
      sub_ = pnh_->subscribe("input", 1, &StaticPolygonArrayPublisher::inputCallback, this);
    }
    if (use_trigger_) {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_trigger_.subscribe(*pnh_, "trigger", 1);
      sync_
        = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_trigger_);
      sync_->registerCallback(boost::bind(
                                &StaticPolygonArrayPublisher::triggerCallback,
                                this, _1, _2));
    }
  }

  void StaticPolygonArrayPublisher::unsubscribe()
  {
    if (use_message_) {
      sub_.shutdown();
    }
    if (use_trigger_) {
      sub_input_.unsubscribe();
      sub_trigger_.unsubscribe();
    }
  }

  void StaticPolygonArrayPublisher::triggerCallback(
    const sensor_msgs::PointCloud2::ConstPtr& input,
    const jsk_recognition_msgs::Int32Stamped::ConstPtr& trigger)
  {
    publishPolygon(input->header.stamp);
  }
  
  PCLModelCoefficientMsg StaticPolygonArrayPublisher::polygonToModelCoefficients(const geometry_msgs::PolygonStamped& polygon)
  {
    Eigen::Vector3d A, B, C;
    A[0] = polygon.polygon.points[0].x;
    A[1] = polygon.polygon.points[0].y;
    A[2] = polygon.polygon.points[0].z;
    B[0] = polygon.polygon.points[1].x;
    B[1] = polygon.polygon.points[1].y;
    B[2] = polygon.polygon.points[1].z;
    C[0] = polygon.polygon.points[2].x;
    C[1] = polygon.polygon.points[2].y;
    C[2] = polygon.polygon.points[2].z;
    Eigen::Vector3d n = (B - A).cross(C - A).normalized();
    double a = n[0];
    double b = n[1];
    double c = n[2];
    double d = -(a * A[0] + b * A[1] + c * A[2]);
    PCLModelCoefficientMsg coefficient;
    coefficient.header = polygon.header;
    coefficient.values.push_back(a);
    coefficient.values.push_back(b);
    coefficient.values.push_back(c);
    coefficient.values.push_back(d);
    return coefficient;
  }

  /*
    parameter format is:
    polygon_array: [[[0, 0, 0], [0, 0, 1], [1, 0, 0]], ...]
   */
  bool StaticPolygonArrayPublisher::readPolygonArray(const std::string& param_name)
  {
    if (pnh_->hasParam(param_name)) {
      XmlRpc::XmlRpcValue v;
      pnh_->param(param_name, v, v);
      if (v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (size_t toplevel_i = 0; toplevel_i < v.size(); toplevel_i++) { // polygons
          XmlRpc::XmlRpcValue polygon_v = v[toplevel_i];
          geometry_msgs::PolygonStamped polygon;
          if (polygon_v.getType() == XmlRpc::XmlRpcValue::TypeArray &&
              polygon_v.size() >= 3) {
            for (size_t secondlevel_i = 0; secondlevel_i < polygon_v.size(); secondlevel_i++) { // each polygon, vertices
              XmlRpc::XmlRpcValue vertex_v = polygon_v[secondlevel_i];
              if (vertex_v.getType() == XmlRpc::XmlRpcValue::TypeArray &&
                  vertex_v.size() == 3 ) { // vertex_v := [x, y, z]
                double x = getXMLDoubleValue(vertex_v[0]);
                double y = getXMLDoubleValue(vertex_v[1]);
                double z = getXMLDoubleValue(vertex_v[2]);
                geometry_msgs::Point32 point;
                point.x = x;
                point.y = y;
                point.z = z;
                polygon.polygon.points.push_back(point);
              }
              else {
                NODELET_FATAL("%s[%lu][%lu] is not array or the length is not 3",
                              param_name.c_str(), toplevel_i, secondlevel_i);
                return false;
              }
            }
            polygons_.polygons.push_back(polygon);
            // estimate model coefficients
            coefficients_.coefficients.push_back(polygonToModelCoefficients(polygon));
          }
          else {
            NODELET_FATAL("%s[%lu] is not array or not enough points", param_name.c_str(), toplevel_i);
            return false;
          }
        }
        return true;
      }
      else {
        NODELET_FATAL("%s is not array", param_name.c_str());
        return false;
      }
    }
    else {
      NODELET_FATAL("no %s is available on parameter server", param_name.c_str());
      return false;
    }
    return true;
  }
  
  double StaticPolygonArrayPublisher::getXMLDoubleValue(XmlRpc::XmlRpcValue val) {
    switch(val.getType()) {
    case XmlRpc::XmlRpcValue::TypeInt:
      return (double)((int)val);
    case XmlRpc::XmlRpcValue::TypeDouble:
      return (double)val;
    default:
      return 0;
    }
  }

  
  void StaticPolygonArrayPublisher::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    publishPolygon(msg->header.stamp);
  }
  
  void StaticPolygonArrayPublisher::timerCallback(const ros::TimerEvent& event)
  {
    publishPolygon(event.current_expected);
  }
  
  void StaticPolygonArrayPublisher::publishPolygon(const ros::Time& stamp)
  {
    polygons_.header.stamp = stamp;
    for (size_t i = 0; i < polygons_.polygons.size(); i++) {
      polygons_.polygons[i].header.stamp = stamp;
    }
    
    coefficients_.header.stamp = stamp;
    for (size_t i = 0; i < coefficients_.coefficients.size(); i++) {
      coefficients_.coefficients[i].header.stamp = stamp;
    }
    
    polygon_pub_.publish(polygons_);
    coefficients_pub_.publish(coefficients_);
  }

}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::StaticPolygonArrayPublisher,
                        nodelet::Nodelet);

