/*
 * tf_listener.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef TF_LISTENER_H__
#define TF_LISTENER_H__

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace jsk_recognition_utils
{
  class TfListener
  {
  public:
    typedef boost::shared_ptr<TfListener> Ptr;
    TfListener(const bool use_tf2,
               const double check_frequency = 10.0,
               const ros::Duration timeout_padding = ros::Duration(2.0),
               const ros::Duration timeout_waiting = ros::Duration(0.0));

    bool waitForTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const ros::Time& time,
                          const ros::Duration& timeout) const;

    void lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         tf::StampedTransform& transform) const;

    template<class T>
    T& transform(const T& in, T& out,
                 const std::string& target_frame,
                 const ros::Duration& timeout=ros::Duration(0.0)) const
    {
      if (use_tf2_) {
        return tf2_buffer_client_->transform(in, out, target_frame, timeout);
      } else {
        transform_tf1(in, out, target_frame, timeout);
        return out;
      }
    }

  protected:
    bool use_tf2_;
    tf::TransformListener* tf_listener_;
    boost::shared_ptr<tf2_ros::BufferClient> tf2_buffer_client_;
  private:
    void transform_tf1(const geometry_msgs::Vector3Stamped& in,
                       geometry_msgs::Vector3Stamped& out,
                       const std::string& target_frame) const;
    void transform_tf1(const geometry_msgs::PointStamped& in,
                       geometry_msgs::PointStamped& out,
                       const std::string& target_frame) const;
    void transform_tf1(const geometry_msgs::PoseStamped& in,
                       geometry_msgs::PoseStamped& out,
                       const std::string& target_frame) const;
    void transform_tf1(const geometry_msgs::QuaternionStamped& in,
                       geometry_msgs::QuaternionStamped& out,
                       const std::string& target_frame) const;
    void transform_tf1(const geometry_msgs::TransformStamped& in,
                       geometry_msgs::TransformStamped& out,
                       const std::string& target_frame) const;
  };
}

#endif // TF_LISTENER_H__
