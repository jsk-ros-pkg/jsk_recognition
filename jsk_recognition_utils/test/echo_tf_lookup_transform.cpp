#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>


void printTF(tf::StampedTransform& transform) {
  std::cout << ">>>>>>>>>>>>>>>>>>>>>> printTF >>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
  std::cout << "frame_id: " << transform.frame_id_ << std::endl;
  std::cout << "child_frame_id: " << transform.child_frame_id_ << std::endl;
  std::cout << "position_x: " << transform.getOrigin().getX() << std::endl;
  std::cout << "position_y: " << transform.getOrigin().getY() << std::endl;
  std::cout << "position_z: " << transform.getOrigin().getZ() << std::endl;
  std::cout << "rotation_x: " << transform.getRotation().getX() << std::endl;
  std::cout << "rotation_y: " << transform.getRotation().getY() << std::endl;
  std::cout << "rotation_z: " << transform.getRotation().getZ() << std::endl;
  std::cout << "rotation_w: " << transform.getRotation().getW() << std::endl;
  std::cout << "<<<<<<<<<<<<<<<<<<<<<< printTF <<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
}


int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Usage: simple_lookup_transform source_frame target_frame" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "simple_lookup_transform");

  boost::mutex mutex;
  tf::TransformListener* tf_listener;

  mutex.lock();
  tf_listener = new tf::TransformListener(ros::Duration(30.0));
  mutex.unlock();

  std::string source_frame(argv[1]);
  std::string target_frame(argv[2]);

  tf_listener->waitForTransform(target_frame, source_frame, ros::Time(), ros::Duration(1.0));

  tf::StampedTransform transform;
  printf("lookupTransform(target_frame=\"%s\", source_frame=\"%s\"): \n",
         target_frame.c_str(), source_frame.c_str());
  tf_listener->lookupTransform(
    /*target_frame=*/target_frame,
    /*source_frame=*/source_frame,
    /*time=*/ros::Time(),
    /*transform=*/transform);
  printTF(transform);

  ros::spinOnce();
  return 0;
}
