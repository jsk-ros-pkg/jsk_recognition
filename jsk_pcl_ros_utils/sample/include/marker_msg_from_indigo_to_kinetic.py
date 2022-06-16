#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Detail of visualization_msgs/msg/Marker.msg of Indigo and that of Kinetic are
# a little bit different, so checksums of them are different of course.
# That means on Kinetic you cannot subscribe the topic of which type is
# visualization_msgs/Marker published on  Indigo.
# Use this script on Kinetic when you want to subscribe the topic of which type
# is visualization_msgs/MarkerArray published on Indigo.
# By executing this script, you can subscribe converted topics
# whose content is the same with visualization_msgs/MarkerArray
# published on Indigo.

import os
import rospy
import visualization_msgs.msg


class VisualizationMarkerBridgeForKinetic(object):
    def __init__(self):
        if os.getenv('ROS_DISTRO', 'kinetic') != 'kinetic':
            rospy.logwarn('This node only works on kinetic.')

        rate = rospy.get_param('~rate', 1.0)
        self.suffix = rospy.get_param('~suffix', 'kinetic')

        self.publishers = dict()
        self.subscribers = dict()

        self.timer = rospy.Timer(rospy.Duration(rate), self.timer_cb)

    def msg_cb(self, msg, topic_name):
        try:
            self.publishers[topic_name].publish(msg)
            rospy.logdebug('Relayed :{}'.format(topic_name))
        except Exception as e:
            rospy.logerr(e)

    def timer_cb(self, event=None):
        all_topics = rospy.get_published_topics()
        msg_types = [
            'InteractiveMarker',
            'InteractiveMarkerControl',
            'InteractiveMarkerInit',
            'InteractiveMarkerUpdate',
            'Marker',
            'MarkerArray',
        ]
        topic_names = []
        topic_types = []
        for topic in all_topics:
            if topic[0].split('/')[-2] == self.suffix:
                continue
            for msg_type in msg_types:
                if topic[1] == 'visualization_msgs/{}'.format(msg_type):
                    topic_names.append(topic[0])
                    topic_types.append(
                        getattr(visualization_msgs.msg, msg_type))

        for topic_name, topic_type in zip(topic_names, topic_types):
            if topic_name not in self.publishers:
                new_topic_name = '/'.join(
                    topic_name.split('/')[:-1] +
                    [self.suffix, topic_name.split('/')[-1]])
                self.publishers[topic_name] = rospy.Publisher(
                    new_topic_name, topic_type, queue_size=1)
                rospy.logdebug(
                    'Advertised: {0} -> {1}'.format(
                        topic_name, new_topic_name))

        for topic_name, pub in self.publishers.items():
            # clean old topics
            if topic_name not in topic_names:
                try:
                    self.subscribers.pop(topic_name).unregister()
                    rospy.logdebug('Removed subscriber: {}'.format(topic_name))
                except Exception:
                    pass
                try:
                    self.publishers.pop(topic_name).unregister()
                    rospy.logdebug('Removed publisher: {}'.format(topic_name))
                except Exception:
                    pass
            # subscribe topics subscribed
            elif pub.get_num_connections() > 0:
                if topic_name not in self.subscribers:
                    self.subscribers[topic_name] = rospy.Subscriber(
                        topic_name, rospy.AnyMsg, self.msg_cb, topic_name)
                    rospy.logdebug('Subscribed {}'.format(topic_name))
            # unsubscribe topics unsubscribed
            else:
                if topic_name in self.subscribers:
                    self.subscribers.pop(topic_name).unregister()
                    rospy.logdebug('Unsubscribed {}'.format(topic_name))


if __name__ == '__main__':
    rospy.init_node('marker_msg_from_indigo_to_kinetic')
    m = VisualizationMarkerBridgeForKinetic()
    rospy.spin()
