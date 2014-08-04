#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("posedetection_msgs")
roslib.load_manifest("dynamic_tf_publisher")
from posedetection_msgs.msg import ObjectDetection
from posedetection_msgs.msg import Object6DPose
from dynamic_tf_publisher.srv import *
import tf
import pgdb
import numpy

class ObjectDetectionDB:
    def __init__(self,connection=None,db_lock=None):
        db_name = rospy.get_param('~db_name','baxterdb')
        username = rospy.get_param('~user_name','baxter')
        self.con = pgdb.connect(database=db_name, user=username)
        self.current_pose =None
        self.latest_pose = None
        self.header_frame=None
        self.no_data_row = True

    def load_latest_pose(self):
        cursor = self.con.cursor()
        cursor.execute("SELECT child_frame_id, transform_translation_x, transform_translation_y, transform_translation_z, transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w FROM tf ORDER BY header_stamp DESC LIMIT 1")
        result = cursor.fetchall()
        if len(result) != 0:
            rospy.logerr("latest position:  pos: %f %f %f, rot: %f %f %f %f" % (result[0][1], result[0][2], result[0][3], result[0][4], result[0][5], result[0][6], result[0][7],))
            self.current_pose = (result[0][1:4],result[0][4:])
            self.latest_pose = (result[0][1:4],result[0][4:])
            self.header_frame = result[0][0]
            self.no_data_row = False
        return (self.latest_pose, self.header_frame)
         
    def update_pose_to_db(self, table, stamp, source, target, pose):
        rospy.loginfo("update_pose")
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot   = [pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]
        cursor = self.con.cursor()
        if self.no_data_row:
            rospy.logerr("INSERT !! pose")
            cursor.execute("INSERT INTO %s (header_stamp,header_frame_id,child_frame_id,transform_translation_x, transform_translation_y, transform_translation_z, transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w) VALUES (%d,'%s','%s',%f,%f,%f,%f,%f,%f,%f);" % (table,stamp.to_nsec(),source,target,trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
            self.no_data_row = False
        else:
            rospy.logerr("UPDATE !! pose")
            cursor.execute("UPDATE %s SET header_stamp = %d ,header_frame_id = '%s', child_frame_id = '%s',transform_translation_x = %f, transform_translation_y = %f, transform_translation_z = %f, transform_rotation_x = %f, transform_rotation_y = %f, transform_rotation_z = %f, transform_rotation_w = %f WHERE id = 1;" % (table,stamp.to_nsec(),source,target,trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        cursor.close()
        self.con.commit()

class ObjectDetectionTfPublisher():
    def __init__(self):
        self.subscriber = rospy.Subscriber("ObjectDetection", ObjectDetection, self.callback);
        self.frame_id = rospy.get_param("~frame_id", "object")
        self.latest_publish = rospy.get_param("~latest_publish", False)
        self.object_detection_db = ObjectDetectionDB()
        self.new_data_flag = False
        if self.latest_publish:
            rospy.logerr("latest publish")
        self.init_object_messages()

    def init_object_messages(self):
        self.object_messages = None
        (latest_pose, header_frame_id) = self.object_detection_db.load_latest_pose()

        if latest_pose:
            loaded_object_messages = ObjectDetection()
            loaded_object = Object6DPose()
            loaded_object.pose.position.x = latest_pose[0][0] 
            loaded_object.pose.position.y = latest_pose[0][1]
            loaded_object.pose.position.z = latest_pose[0][2]
            loaded_object.pose.orientation.x = latest_pose[1][0]
            loaded_object.pose.orientation.y = latest_pose[1][1]
            loaded_object.pose.orientation.z = latest_pose[1][2]
            loaded_object.pose.orientation.w = latest_pose[1][3]
 
            loaded_object_messages.objects = [loaded_object]
            loaded_object_messages.header.frame_id = header_frame_id
            rospy.logerr("object messgage is loded : %f %f %f %f %f %f %f", 
                         latest_pose[0][0], latest_pose[0][1], latest_pose[0][2], latest_pose[1][0], latest_pose[1][1], latest_pose[1][2], latest_pose[1][3])
            self.object_messages = loaded_object_messages
        else:
            rospy.logerr("No database data was found for object detection")

    def callback(self, msg):
        if  msg.objects or ( not self.latest_publish):
            self.object_messages = msg
            self.new_data_flag = True

    def run(self):
        r = rospy.Rate(50)
        counter = 0
        while not rospy.is_shutdown():
            if self.object_messages:
                for detected_object in self.object_messages.objects:
                    br = tf.TransformBroadcaster()
                    br.sendTransform((detected_object.pose.position.x,
                                     detected_object.pose.position.y,
                                     detected_object.pose.position.z),
                                     (detected_object.pose.orientation.x,
                                      detected_object.pose.orientation.y,
                                      detected_object.pose.orientation.z,
                                      detected_object.pose.orientation.w),
                                     rospy.get_rostime(),
                                     self.frame_id,
                                     self.object_messages.header.frame_id,
                                     )
                    if counter%100 == 0 and self.new_data_flag:
                        self.object_detection_db.update_pose_to_db("tf", rospy.get_rostime(), self.frame_id, self.object_messages.header.frame_id, detected_object.pose)
                        self.new_data_flag = False
            r.sleep()
            counter += 1

if __name__== '__main__':
    rospy.init_node('objectdetection_tf_publisher', anonymous=True)
    object_detection_tf_publisher = ObjectDetectionTfPublisher()
    object_detection_tf_publisher.run()
