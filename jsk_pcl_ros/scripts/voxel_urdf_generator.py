#!/usr/bin/env python
import rospy
import os.path

from jsk_pcl_ros.srv import *

def generateURDFModel(req):
    # open file
    filename = req.filename
    if not os.path.isabs(filename):
        filename = "/tmp/" + filename
    try:
        f = open(filename, 'w')
        rospy.loginfo('generated urdf model : %s' % filename)
    except IOError:
        rospy.logwarn('can not open %s' % filename)
        return VoxelModelGenerateResponse(False)

    # write first line
    f.write('''<robot name="%s">
''' % req.name)

    first_link_flag = True
    marker = req.voxel
    for i in range(len(marker.points)):
        point = marker.points[i]
        link_name = "link%d" % i
        # first link is treated as the root link
        if first_link_flag:
            root_link_name = link_name
        # add new link
        f.write('''  <link name="%s">
    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1" />
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="%f %f %f" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="%f %f %f" />
      </geometry>
    </collision>
  </link>
  <gazebo reference="%s">
    <material>Gazebo/Blue</material>
  </gazebo>
  <gazebo reference="%s">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
''' % (link_name,
                  marker.scale.x, marker.scale.y, marker.scale.z,
                  marker.scale.x, marker.scale.y, marker.scale.z,
                  link_name, link_name))
        # add a fixed joint between the new link and the root link
        if not first_link_flag:
            joint_name = "joint%d" % i
            f.write('''  <joint name="%s" type="fixed">
    <origin xyz="%f %f %f"/>
    <parent link="%s"/>
    <child link="%s"/>
  </joint>
''' % (joint_name,
       point.x, point.y, point.z,
       root_link_name, link_name))
        first_link_flag = False

    # write final line
    f.write('''</robot>
''')
    f.close()
    return VoxelModelGenerateResponse(True)

def main():
  rospy.init_node("voxel_urdf_generator")
  voxel_urdf_generator = rospy.Service("/generate_voxel_urdf", VoxelModelGenerate, generateURDFModel)
  rospy.spin()

if __name__ == "__main__":
  main()
