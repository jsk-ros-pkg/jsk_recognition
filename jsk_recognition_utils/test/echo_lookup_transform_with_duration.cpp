#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "jsk_recognition_utils/tf_listener_singleton.h"


void printTF(tf::StampedTransform& transform) {
  std::cout << "frame_id: " << transform.frame_id_ << ", "
            << "child_frame_id: " << transform.child_frame_id_ << ", "
            << "pos: " << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << ", "
            << "rot: " << transform.getRotation().getX() << " " << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " " << transform.getRotation().getW();
}


int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Usage: echo_lookup_transform_with_duration from_frame to_frame" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "echo_lookup_transform_with_duration");

  boost::mutex mutex;
  tf::TransformListener* tf_listener;

  mutex.lock();
  tf_listener = new tf::TransformListener(ros::Duration(30.0));
  mutex.unlock();

  std::string from_frame(argv[1]);
  std::string to_frame(argv[2]);

  tf::StampedTransform transform;
  std::cerr << "lookupTransformWithDuration(to_frame=" << to_frame  << ", from_frame=" << from_frame << ")" << std::endl;
  transform = jsk_recognition_utils::lookupTransformWithDuration(
    /*listener=*/tf_listener,
    /*to_frame=*/to_frame,
    /*from_frame=*/from_frame,
    /*time=*/ros::Time(),
    /*duration=*/ros::Duration(1.0));
  printTF(transform);

  ros::spinOnce();
  return 0;
}
