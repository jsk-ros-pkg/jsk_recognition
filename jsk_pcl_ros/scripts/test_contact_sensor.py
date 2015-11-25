#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from jsk_recognition_msgs.msg import ContactSensor, ContactSensorArray
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('test_contact_sensor')
    contact_sensor_array_pub = rospy.Publisher('contact_sensors_in', ContactSensorArray, latch=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = ContactSensorArray()
        msg.header = Header(frame_id="/gazebo_world", stamp=rospy.Time.now())
        nil_link_sensor = ContactSensor(header=Header(frame_id="/gazebo_world", stamp=rospy.Time.now()), contact=False, link_name='nil_link')
        msg.datas = [nil_link_sensor]
        contact_sensor_array_pub.publish(msg)
        r.sleep()

