#!/usr/bin/env python
import rospy

PKG='jsk_pcl_ros'

from visualization_msgs.msg import Marker
from jsk_pcl_ros.srv import VoxelModelGenerate

def cloudCallback(msg):
    global requested_flag, generate_srv
    rospy.loginfo("Request model generation.")
    if not generate_srv(voxel=msg, name="test_model", filename="test.urdf"):
        rospy.logwarn("Request of model generation failed.")
    requested_flag = True

def main():
    global requested_flag, generate_srv
    requested_flag = False
    rospy.init_node("voxel_urdf_client")
    cloud_sub = rospy.Subscriber("input", Marker, cloudCallback)
    generate_srv = rospy.ServiceProxy("/generate_voxel_urdf", VoxelModelGenerate)
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
        if requested_flag:
            break

if __name__ == "__main__":
  main()
