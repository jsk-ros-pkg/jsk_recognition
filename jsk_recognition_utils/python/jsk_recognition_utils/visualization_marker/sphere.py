import rospy
from std_msgs.msg import Header
import visualization_msgs.msg
from visualization_msgs.msg import Marker


def make_sphere(radius=0.1,
                pos=[0.0, 0.0, 0.0],
                q_xyzw=[0.0, 0.0, 0.0, 1.0],
                color=(0.0, 0.0, 0.0, 1.0),
                lifetime=0.25,
                id=0,
                frame_id='',
                stamp=None):
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    mesh_marker = Marker(type=Marker.SPHERE, header=header, id=id)
    mesh_marker.pose.position.x = pos[0]
    mesh_marker.pose.position.y = pos[1]
    mesh_marker.pose.position.z = pos[2]
    mesh_marker.pose.orientation.x = q_xyzw[0]
    mesh_marker.pose.orientation.y = q_xyzw[1]
    mesh_marker.pose.orientation.z = q_xyzw[2]
    mesh_marker.pose.orientation.w = q_xyzw[3]
    mesh_marker.scale.x = radius
    mesh_marker.scale.y = radius
    mesh_marker.scale.z = radius
    mesh_marker.color.b, mesh_marker.color.g, mesh_marker.color.r, \
        mesh_marker.color.a = color
    mesh_marker.lifetime = lifetime
    mesh_marker.lifetime = rospy.Duration(lifetime)
    return mesh_marker
