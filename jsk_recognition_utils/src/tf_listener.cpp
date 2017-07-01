/*
 * tf_listener.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_recognition_utils/tf_listener.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>

namespace jsk_recognition_utils
{
  TfListener::TfListener(const bool use_tf2,
                         const double check_frequency,
                         const ros::Duration timeout_padding,
                         const ros::Duration timeout_waiting) :
    use_tf2_(use_tf2)
  {
    if (use_tf2_) {
      tf2_buffer_client_.reset(new tf2_ros::BufferClient("/tf2_buffer_server", check_frequency, timeout_padding));
      if (!tf2_buffer_client_->waitForServer(timeout_waiting)) {
        ROS_ERROR_STREAM("Timed out to wait /tf2_buffer_server. TF1 is used instead of TF2");
        tf2_buffer_client_.reset();
        use_tf2_ = false;
      }
    }
    if (!use_tf2_) {
      tf_listener_ = TfListenerSingleton::getInstance();
    }
  }

  bool TfListener::waitForTransform(const std::string& target_frame,
                                    const std::string& source_frame,
                                    const ros::Time& time,
                                    const ros::Duration& timeout) const
  {
    if (use_tf2_) {
      std::string errstr;
      bool ok = tf2_buffer_client_->canTransform(target_frame, source_frame,
                                                 time, timeout,
                                                 &errstr);
      if (!ok) ROS_ERROR_STREAM(errstr);
      return ok;
    } else {
      return tf_listener_->waitForTransform(target_frame, source_frame,
                                            time, timeout);
    }
  }

  void TfListener::lookupTransform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const ros::Time& time,
                                   tf::StampedTransform& transform) const
  {
    if (use_tf2_) {
      geometry_msgs::TransformStamped ts
        = tf2_buffer_client_->lookupTransform(target_frame, source_frame, time);
      tf::transformStampedMsgToTF(ts, transform);
    } else {
      tf_listener_->lookupTransform(target_frame, source_frame,
                                    time, transform);
    }
  }

  void TfListener::transform_tf1(const geometry_msgs::Vector3Stamped& in,
                                 geometry_msgs::Vector3Stamped& out,
                                 const std::string& target_frame) const
  {
    tf_listener_->transformVector(target_frame, in, out);
  }

  void TfListener::transform_tf1(const geometry_msgs::PointStamped& in,
                                 geometry_msgs::PointStamped& out,
                                 const std::string& target_frame) const
  {
    tf_listener_->transformPoint(target_frame, in, out);
  }

  void TfListener::transform_tf1(const geometry_msgs::PoseStamped& in,
                                 geometry_msgs::PoseStamped& out,
                                 const std::string& target_frame) const
  {
    tf_listener_->transformPose(target_frame, in, out);
  }

  void TfListener::transform_tf1(const geometry_msgs::QuaternionStamped& in,
                                 geometry_msgs::QuaternionStamped& out,
                                 const std::string& target_frame) const
  {
    tf_listener_->transformQuaternion(target_frame, in, out);
  }

  void TfListener::transform_tf1(const geometry_msgs::TransformStamped& in,
                                 geometry_msgs::TransformStamped& out,
                                 const std::string& target_frame) const
  {
    tf::StampedTransform tin, tout;
    tf::transformStampedMsgToTF(in, tin);
    tf_listener_->lookupTransform(target_frame, in.header.frame_id,
                                  in.header.stamp, tout);
    tf::transformStampedTFToMsg(tout, out);
  }
}
