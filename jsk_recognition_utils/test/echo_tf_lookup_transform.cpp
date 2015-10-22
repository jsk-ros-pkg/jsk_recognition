#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>


void printTF(tf::StampedTransform& transform) {
  std::cout << "frame_id: " << transform.frame_id_ << ", "
            << "child_frame_id: " << transform.child_frame_id_ << ", "
            << "pos: " << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << ", "
            << "rot: " << transform.getRotation().getX() << " " << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " " << transform.getRotation().getW();
}


int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Usage: echo_tf_lookup_transform source_frame target_frame" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "echo_tf_lookup_transform");

  boost::mutex mutex;
  tf::TransformListener* tf_listener;

  mutex.lock();
  tf_listener = new tf::TransformListener(ros::Duration(30.0));
  mutex.unlock();

  std::string source_frame(argv[1]);
  std::string target_frame(argv[2]);

  tf_listener->waitForTransform(target_frame, source_frame, ros::Time(), ros::Duration(1.0));

  tf::StampedTransform transform;
  std::cerr << "lookupTransform(target_frame=" << target_frame  << ", source_frame=" << source_frame << ")" << std::endl;
  tf_listener->lookupTransform(
    /*target_frame=*/target_frame,
    /*source_frame=*/source_frame,
    /*time=*/ros::Time(),
    /*transform=*/transform);
  printTF(transform);

  ros::spinOnce();
  return 0;
}
