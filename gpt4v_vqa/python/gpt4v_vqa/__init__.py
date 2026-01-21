import actionlib
import rospy
from jsk_recognition_msgs.msg import VQATaskAction, VQATaskGoal


class VQAClient:
    def __init__(self, action_name="/vqa/inference_server"):
        self.client = actionlib.SimpleActionClient(action_name, VQATaskAction)

    def wait_for_server(self, timeout=10.0):
        self.client.wait_for_server(timeout=rospy.Duration(timeout))

    def vqa(self, question, image=None, timeout=30.0):
        goal = VQATaskGoal()
        goal.questions = [question]
        if image is not None:
            goal.image = image
        self.client.send_goal(goal)
        if self.client.wait_for_result(timeout=rospy.Duration(timeout)):
            result = self.client.get_result()
            return result
        else:
            rospy.logwarn("Timeout")
            return None
