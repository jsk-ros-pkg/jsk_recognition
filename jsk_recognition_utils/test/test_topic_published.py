#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import socket
import sys
import time
import unittest

from nose.tools import assert_false
from nose.tools import assert_true
import rosgraph
import rosnode
import rospy
import rostopic
from rostopic import _master_get_topic_types
from rostopic import get_topic_list
from rostopic import ROSTopicIOException


PKG = 'jsk_recognition_utils'
NAME = 'test_topic_published'


def get_publisher_nodes_from_topic(topic):
    try:
        from cStringIO import StringIO
    except ImportError:
        from io import StringIO
    import itertools
    StringIO()

    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master('/rostopic')
    try:
        pubs, subs = get_topic_list(master=master)
        # filter based on topic
        subs = [x for x in subs if x[0] == topic]
        pubs = [x for x in pubs if x[0] == topic]
        _master_get_topic_types(master)
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    nodes = list(itertools.chain(*[nodes for topic, ttype, nodes in pubs]))
    return nodes


class PublishChecker(object):
    def __init__(self, topic_name, timeout):
        self.topic_name = topic_name
        self.deadline = rospy.Time.now() + rospy.Duration(timeout)
        msg_class, _, _ = rostopic.get_topic_class(
            rospy.resolve_name(topic_name), blocking=True)
        self.msg = None
        self.sub = rospy.Subscriber(topic_name, msg_class, self._callback)

    def _callback(self, msg):
        self.msg = msg

    def assert_published(self):
        if self.msg:
            return True
        if rospy.Time.now() > self.deadline:
            return False
        return None


class TestTopicPublished(unittest.TestCase):
    def __init__(self, *args):
        super(TestTopicPublished, self).__init__(*args)
        rospy.init_node(NAME)
        # topics to check sanity
        self.topics = []
        self.timeouts = []
        self.negatives = []
        params = rospy.get_param(rospy.get_name(), [])
        for name, value in params.items():
            if not re.match(r'^topic_\d$', name):
                continue
            self.topics.append(value)
            id = name.replace('topic_', '')
            self.timeouts.append(rospy.get_param('~timeout_{}'.format(id), 10))
            self.negatives.append(
                rospy.get_param('~negative_{}'.format(id), False))
        if not self.topics:
            rospy.logerr('No topic is specified.')
            sys.exit(1)
        self.check_after_kill_node = rospy.get_param(
            '~check_after_kill_node', False)
        if self.check_after_kill_node:
            self.target_node_names = rospy.get_param('~node_names')
            if not isinstance(self.target_node_names, list):
                self.target_node_names = [self.target_node_names]

    def test_published(self):
        """Test topics are published and messages come"""
        use_sim_time = rospy.get_param('/use_sim_time', False)
        t_start = time.time()
        while not rospy.is_shutdown() and \
                use_sim_time and (rospy.Time.now() == rospy.Time(0)):
            rospy.logwarn('/use_sim_time is specified and rostime is 0, '
                          '/clock is published?')
            if time.time() - t_start > 10:
                self.fail('Timed out (10s) of /clock publication.')
            time.sleep(1)

        self._check_topic_pubilshed()

        if self.check_after_kill_node:
            rospy.logwarn('Check topic published after killing nodes ({})'.
                          format(self.target_node_names))
            rosnode.kill_nodes(self.target_node_names)
            time.sleep(5.0)
            self._check_topic_pubilshed()

    def _check_topic_pubilshed(self):
        checkers = []
        for topic_name, timeout, negative in zip(
                self.topics, self.timeouts, self.negatives):
            checker = PublishChecker(topic_name, timeout)
            checkers.append(checker)

        topics_finished = []
        while not rospy.is_shutdown():
            if len(topics_finished) == len(checkers):
                break
            for i, checker in enumerate(checkers):
                if checker.topic_name in topics_finished:
                    continue
                ret = checker.assert_published()
                if ret is None:
                    continue
                topics_finished.append(checker.topic_name)
                if self.negatives[i]:
                    assert_false(ret, 'Topic [%s] is published' %
                                      checker.topic_name)
                else:
                    assert_true(
                        ret, 'Topic [%s] is not published' %
                             checker.topic_name)
            try:
                rospy.sleep(0.01)
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                continue
        if len(topics_finished) != len(checkers):
            raise ValueError('Not all topics are received')


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestTopicPublished, sys.argv)
