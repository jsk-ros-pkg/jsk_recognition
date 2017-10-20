#!/usr/bin/env python

from and_scale_ros.msg import WeightStamped
from jsk_recognition_msgs.msg import BoolStamped
from jsk_recognition_msgs.msg import Label
from jsk_recognition_msgs.msg import LabelArray
import message_filters
import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class WeightCanditatesRefiner(object):

    def __init__(self):
        # {object_name: object_id}
        self.candidates = {}
        self.input_topics = rospy.get_param('~input_topics')
        # {object_name: weight}
        self.object_weights = rospy.get_param('~object_weights')
        self.error = rospy.get_param('~error', 1.0)

        self.weight_sum_at_reset = 0.0
        self.prev_weight_values = [0] * len(self.input_topics)

        self.weight_sum_pub = rospy.Publisher(
            '~debug/weight_sum', WeightStamped, queue_size=1)
        self.weight_sum_at_reset_pub = rospy.Publisher(
            '~debug/weight_sum_at_reset', WeightStamped, queue_size=1)
        self.changed_pub = rospy.Publisher(
            '~output/changed_from_reset', BoolStamped, queue_size=1)
        self.picked_pub = rospy.Publisher(
            '~output/candidates/picked', LabelArray, queue_size=1)
        self.placed_pub = rospy.Publisher(
            '~output/candidates/placed', LabelArray, queue_size=1)
        self.can_reset = False
        self.subscribe()

    def subscribe(self):
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 10)
        # add candidates subscriber
        self.sub_candidates = rospy.Subscriber(
            '~input/candidates', LabelArray, self._candidates_cb)
        # add scale subscriber
        self.subs = []
        for input_topic in self.input_topics:
            sub = message_filters.Subscriber(
                input_topic, WeightStamped, queue_size=1)
            self.subs.append(sub)
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size=queue_size, slop=slop)
        else:
            sync = message_filters.TimeSynchronizer(
                self.subs, queue_size=queue_size)
        sync.registerCallback(self._scale_cb)

    def _candidates_cb(self, labels_msg):
        self.candidates = {}
        for label_msg in labels_msg.labels:
            self.candidates[label_msg.name] = label_msg.id

    def _scale_cb(self, *weight_msgs):
        assert len(weight_msgs) == len(self.prev_weight_values)

        # Publish debug info
        weight_values = [w.weight.value for w in weight_msgs]
        weight_sum = sum(weight_values)
        sum_msg = WeightStamped()
        sum_msg.header = weight_msgs[0].header
        sum_msg.weight.value = weight_sum
        sum_msg.weight.stable = all(msg.weight.stable for msg in weight_msgs)
        sum_at_reset_msg = WeightStamped()
        sum_at_reset_msg.header = weight_msgs[0].header
        sum_at_reset_msg.weight.value = self.weight_sum_at_reset
        sum_at_reset_msg.weight.stable = True
        self.weight_sum_at_reset_pub.publish(sum_at_reset_msg)
        self.weight_sum_pub.publish(sum_msg)

        if not sum_msg.weight.stable:
            return  # unstable

        # Store stable weight and enable resetting
        self.prev_weight_values = weight_values
        if not self.can_reset:
            self.reset_srv = rospy.Service('~reset', Trigger, self._reset)
            self.can_reset = True

        if not self.candidates:
            rospy.logwarn_throttle(10, 'No candidates, so skip refining')
            return
        candidates = self.candidates

        # Judge if scale value is changed
        weight_diff = weight_sum - self.weight_sum_at_reset
        diff_lower = weight_diff - self.error
        diff_upper = weight_diff + self.error
        weight_min = min(self.object_weights.get(x, float('inf'))
                         for x in candidates.keys())
        changed_msg = BoolStamped()
        changed_msg.header = weight_msgs[0].header
        if -weight_min < diff_lower and diff_upper < weight_min \
                and diff_lower < 0 and 0 < diff_upper:
            changed_msg.data = False
        else:
            changed_msg.data = True
        self.changed_pub.publish(changed_msg)

        # Output candidates
        pick_msg = LabelArray()
        place_msg = LabelArray()
        pick_msg.header = weight_msgs[0].header
        place_msg.header = weight_msgs[0].header
        for obj_name, w in self.object_weights.items():
            if obj_name not in candidates:
                continue
            obj_id = candidates[obj_name]
            if diff_upper > w and w > diff_lower:
                label = Label()
                label.id = obj_id
                label.name = obj_name
                place_msg.labels.append(label)
            elif diff_upper > -w and -w > diff_lower:
                label = Label()
                label.id = obj_id
                label.name = obj_name
                pick_msg.labels.append(label)
        self.picked_pub.publish(pick_msg)
        self.placed_pub.publish(place_msg)

    def _reset(self, req):
        is_success = True
        try:
            self.weight_sum_at_reset = sum(self.prev_weight_values)
        except Exception:
            is_success = False
        return TriggerResponse(success=is_success)


if __name__ == '__main__':
    rospy.init_node('weight_candidates_refiner')
    app = WeightCanditatesRefiner()
    rospy.spin()
