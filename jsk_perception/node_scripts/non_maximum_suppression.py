#!/usr/bin/env python

from jsk_perception.srv import *
from jsk_recognition_msgs.msg import Rect
import rospy
import numpy as np

# Malisiewicz et al.
def non_max_suppression_fast(boxes, overlapThresh):
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

def non_max_suppression_equator(rect, threshold):
        _box = []
        for box in rect:
                x1 = box.x 
                x2 = x1 + box.width
                y1 = box.y
                y2 = y1 + box.height
                r = (x1, y1, x2, y2)
                _box.append(r)
        bbox = np.array(_box)
        return non_max_suppression_fast(bbox, threshold);

def non_max_suppression_handler(req):
        pick = non_max_suppression_equator(req.rect, req.threshold)
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
        return NonMaximumSuppressionResponse(bbox, len(pick))


def non_max_suppression_server():
        rospy.init_node('non_maximum_suppression_server')
        s = rospy.Service('non_maximum_suppression',
                          NonMaximumSuppression, non_max_suppression_handler)
        print "DEBUG: Bounding Box NMS...."
        rospy.spin()

if __name__ == "__main__":
        non_max_suppression_server()
