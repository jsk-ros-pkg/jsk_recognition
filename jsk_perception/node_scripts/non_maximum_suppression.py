#!/usr/bin/env python

from jsk_perception.srv import NonMaximumSuppression
from jsk_perception.srv import NonMaximumSuppressionResponse
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
import rospy
from std_msgs.msg import Int64
import numpy as np


class NonMaximumSuppressionServer(ConnectionBasedTransport):

    def __init__(self):
        super(self.__class__, self).__init__()
        self.thre = rospy.get_param('~threshold', 0.0)
        self.pub_rects = self.advertise('~output', RectArray, queue_size=1)
        self.pub_count = self.advertise('~output/count', Int64, queue_size=1)
        self.srv_server = rospy.Service(
                'non_maximum_suppression',
                NonMaximumSuppression, self._req_cb)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', RectArray, self._msg_cb, queue_size=1)

    def unsubscribe(self):
        self.sub.unregister()

    def _msg_cb(self, msg):
        rects, count = self.non_max_suppression_handler(msg.rects, self.thre)

        rects_msg = RectArray()
        rects_msg.header = msg.header
        rects_msg.rects = rects
        self.pub_rects.publish(rects_msg)

        count_msg = Int64()
        count_msg.data = count
        self.pub_count.publish(count_msg)

    def _req_cb(self, req):
        rects, count = self.non_max_suppression_handler(
            req.rect, req.threshold)
        return NonMaximumSuppressionResponse(rects, count)

    # Malisiewicz et al.
    def non_max_suppression_fast(self, boxes, overlapThresh):
        if len(boxes) == 0:
                return []
        if boxes.dtype.kind == "i":
                boxes = boxes.astype("float")
        pick = []
        x1 = boxes[:,0]
        y1 = boxes[:,1]
        x2 = boxes[:,2]
        y2 = boxes[:,3]
        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y2)
        while len(idxs) > 0:
                last = len(idxs) - 1
                i = idxs[last]
                pick.append(i)
                xx1 = np.maximum(x1[i], x1[idxs[:last]])
                yy1 = np.maximum(y1[i], y1[idxs[:last]])
                xx2 = np.minimum(x2[i], x2[idxs[:last]])
                yy2 = np.minimum(y2[i], y2[idxs[:last]])
                w = np.maximum(0, xx2 - xx1 + 1)
                h = np.maximum(0, yy2 - yy1 + 1)
                overlap = (w * h) / area[idxs[:last]]
                idxs = np.delete(idxs, np.concatenate(([last],
                        np.where(overlap > overlapThresh)[0])))
        return boxes[pick].astype("int")

    def non_max_suppression_equator(self, rect, threshold):
        _box = []
        for box in rect:
                x1 = box.x 
                x2 = x1 + box.width
                y1 = box.y
                y2 = y1 + box.height
                r = (x1, y1, x2, y2)
                _box.append(r)
        bbox = np.array(_box)
        return self.non_max_suppression_fast(bbox, threshold)

    def non_max_suppression_handler(self, rects, threshold):
        pick = self.non_max_suppression_equator(rects, threshold)
        bbox = []
        i = 0
        while i < len(pick):
                bx = pick[i][0]
                by = pick[i][1]
                bw = pick[i][2] - bx
                bh = pick[i][3] - by
                bbox.append(Rect(bx, by, bw, bh))
                #print Rect(bx, by, bw, bh)
                i+=1    
        return bbox, len(pick)


if __name__ == "__main__":
    rospy.init_node('non_maximum_suppression_server')
    app = NonMaximumSuppressionServer()
    rospy.spin()
