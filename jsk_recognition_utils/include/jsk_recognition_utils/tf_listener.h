/*
 * tf_listener.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef TF_LISTENER_H__
#define TF_LISTENER_H__

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace jsk_recognition_utils
{
  class TfListener
  {
  public:
    typedef boost::shared_ptr<TfListener> Ptr;
    TfListener(const ros::NodeHandle& nh);

    // TF2 Style API
    virtual bool
    canTransform(const std::string&   target_frame,
                 const std::string&   source_frame,
                 const ros::Time&     target_time,
                 const ros::Duration& timeout,
                 std::string*         errstr = NULL) const;

    virtual geometry_msgs::TransformStamped
    lookupTransform(const std::string&   target_frame,
                    const std::string&   source_frame,
                    const ros::Time&     time,
                    const ros::Duration& timeout) const;

    template<class T>
    T& transform(const T& in, T& out,
                 const std::string&   target_frame,
                 const ros::Duration& timeout = ros::Duration(0.0),
                 std::string*         errstr  = NULL) const
    {
      if (use_buffer_client_)
        return buffer_client_->transform(in, out, target_frame, timeout);
      else
        return buffer_->transform(in, out, target_frame, timeout);
    }

    // Backport for TF1 style API
    virtual bool
    waitForTransform(const std::string& target_frame,
                     const std::string& source_frame,
                     const ros::Time& time,
                     const ros::Duration& timeout,
                     const ros::Duration& polling_sleep_duration = ros::Duration(0.01),
                     std::string* error_msg = NULL) const;

    virtual bool
    canTransform(const std::string& target_frame,
                 const std::string& source_frame,
                 const ros::Time& time,
                 std::string* error_msg = NULL) const;

    virtual void
    lookupTransform(const std::string& target_frame,
                    const std::string& source_frame,
                    const ros::Time& time,
                    tf::StampedTransform& transform) const;

  protected:
    bool use_buffer_client_;
    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::BufferClient> buffer_client_;
  };
}

#endif // TF_LISTENER_H__
