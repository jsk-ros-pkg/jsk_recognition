/*
 * tf_listener.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_recognition_utils/tf_listener.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>

namespace jsk_recognition_utils
{
  TfListener::TfListener(const ros::NodeHandle& nh)
  {
    nh.param("tf_use_buffer_client", use_buffer_client_, false);

    // for buffer
    int cache_time;
    nh.param("tf_cache_time", cache_time, 10); // = tf2::BufferCore::DEFAULT_CACHE_TIME

    // for buffer client
    std::string buffer_server_ns;
    double check_frequency, timeout_padding, wait_server_timeout;
    nh.param("tf_buffer_server_ns", buffer_server_ns, std::string("/tf2_buffer_server"));
    nh.param("tf_check_frequency", check_frequency, 10.0);
    nh.param("tf_timeout_padding", timeout_padding, 2.0);
    nh.param("tf_wait_server_timeout", wait_server_timeout, 10.0);

    if (use_buffer_client_) {
      buffer_client_.reset(new tf2_ros::BufferClient(buffer_server_ns, check_frequency,
                                                     ros::Duration(timeout_padding)));
      if (!buffer_client_->waitForServer(ros::Duration(wait_server_timeout))) {
        ROS_ERROR_STREAM("Waiting " << buffer_server_ns << " timed out");
        ROS_WARN_STREAM("Falling back to use tf2_ros::TransformListener");
        buffer_client_.reset();
        use_buffer_client_ = false;
      }
    }
    if (!use_buffer_client_) {
      buffer_.reset(new tf2_ros::Buffer(ros::Duration(cache_time)));
      listener_.reset(new tf2_ros::TransformListener(*buffer_));
    }
  }

  bool
  TfListener::canTransform(const std::string&   target_frame,
                           const std::string&   source_frame,
                           const ros::Time&     target_time,
                           const ros::Duration& timeout,
                           std::string*         errstr) const
  {
    if (use_buffer_client_)
      return buffer_client_->canTransform(target_frame, source_frame,
                                          target_time, timeout, errstr);
    else
      return buffer_->canTransform(target_frame, source_frame,
                                   target_time, timeout, errstr);
  }

  geometry_msgs::TransformStamped
  TfListener::lookupTransform(const std::string&   target_frame,
                              const std::string&   source_frame,
                              const ros::Time&     time,
                              const ros::Duration& timeout) const
  {
    if (use_buffer_client_)
      return buffer_client_->lookupTransform(target_frame, source_frame,
                                             time, timeout);
    else
      return buffer_->lookupTransform(target_frame, source_frame,
                                      time, timeout);
  }

  bool
  TfListener::waitForTransform(const std::string& target_frame,
                               const std::string& source_frame,
                               const ros::Time& time,
                               const ros::Duration& timeout,
                               const ros::Duration& polling_sleep_duration,
                               std::string* error_msg) const
  {
    ros::Time start = ros::Time::now();
    bool ok = false;
    while (ros::Time::now() - start < timeout)
    {
      if (use_buffer_client_)
        ok = buffer_client_->canTransform(target_frame, source_frame,
                                          time, polling_sleep_duration,
                                          error_msg);
      else
        ok = buffer_->canTransform(target_frame, source_frame,
                                   time, polling_sleep_duration,
                                   error_msg);
      if (ok) return true;
    }
    return false;
  }

  bool
  TfListener::canTransform(const std::string& target_frame,
                           const std::string& source_frame,
                           const ros::Time&   time,
                           std::string*       error_msg) const
  {
    if (use_buffer_client_)
      return buffer_client_->canTransform(target_frame, source_frame,
                                          time, ros::Duration(0.001),
                                          error_msg);
    else
      return buffer_->canTransform(target_frame, source_frame,
                                   time, error_msg);
  }

  void
  TfListener::lookupTransform(const std::string&    target_frame,
                              const std::string&    source_frame,
                              const ros::Time&      time,
                              tf::StampedTransform& transform) const
  {
    if (use_buffer_client_) {
      geometry_msgs::TransformStamped ts
        = buffer_client_->lookupTransform(target_frame, source_frame, time);
      tf::transformStampedMsgToTF(ts, transform);
    } else {
      geometry_msgs::TransformStamped ts
        = buffer_->lookupTransform(target_frame, source_frame, time);
      tf::transformStampedMsgToTF(ts, transform);
    }
  }
}
