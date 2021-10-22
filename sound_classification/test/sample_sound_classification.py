#!/usr/bin/env python

from __future__ import print_function

import sys
import threading
import time
import unittest

import rospy
import rostest
import roslaunch

CLASSNAME = 'sound_classify_test'


class SoundClassifyTest(unittest.TestCase):
    def __init__(self, *args):
        super(SoundClassifyTest, self).__init__(*args)
        rospy.init_node(CLASSNAME)

    def roslaunch(self, args, timeout=-1):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        roslaunch_args = tuple(args[2:])
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(args)[0],roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        if timeout > 0:
            rospy.sleep(timeout)
        parent.shutdown()
        rospy.set_param('/use_sim_time', False)  # need to disable use_sim_time to run rospy.sleep()
        time.sleep(3)  # wait to shutdown....

    def rosrun(self, package, executable, args=''):
        rospy.logwarn("start {} {} {}".format(package, executable, args))
        node = roslaunch.core.Node(package, executable, args=args, output='screen')

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
        while process.is_alive():
            time.sleep(1)
        process.stop()
        rospy.logwarn("done {} {} {}".format(package, executable, args))

    def test_sound_classify(self):
        try:
            # Getting the attributes of the test.
            # start collect spectrograms from sample rosbags for no_sound class
            self.roslaunch(['sound_classification', 'save_sound.launch', 'use_rosbag:=true', 'filename:=$(find sound_classification)/sample_rosbag/no_sound.bag', 'target_class:=no_sound', 'threshold:=0', 'save_when_sound:=false', 'pause_rosbag:=false', 'gui:=false'], timeout=10)
            # start collect spectrograms from sample rosbags for applause class
            self.roslaunch(['sound_classification', 'save_sound.launch', 'use_rosbag:=true', 'filename:=$(find sound_classification)/sample_rosbag/applause.bag', 'target_class:=applause', 'threshold:=5', 'pause_rosbag:=false', 'gui:=false'], timeout=10)
            # create dataset
            self.rosrun('sound_classification', 'create_dataset.py', '--number 20 --augment 5')
            # train
            self.rosrun('sound_classification', 'train.py', '--epoch 3 --gpu -1')
            # classify sound
            self.roslaunch(['sound_classification', 'classify_sound.launch', 'gui:=false', 'use_microphone:=false', 'use_rosbag:=true', 'filename:=$(find sound_classification)/sample_rosbag/voice.bag', 'pause_rosbag:=false', 'gpu:=-1'], timeout=120)
        except Exception as e:
            rospy.logerr("Failed to test sound_classify ({})".format(e))
            self.fail("test_sound_classify failed ({})".format(e))
        self.assertTrue(True)


if __name__ == '__main__':
    try:
        rostest.run('sound_classify_test', CLASSNAME, SoundClassifyTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("{} exiting".format(CLASSNAME))
