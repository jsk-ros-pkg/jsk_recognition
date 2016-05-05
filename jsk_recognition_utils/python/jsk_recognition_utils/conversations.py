import numpy as np


def rects_msg_to_ndarray(rects_msg):
    rects = np.zeros((len(rects_msg.rects), 4), dtype=np.float32)
    for i, r in enumerate(rects_msg.rects):
        xmin = r.x
        ymin = r.y
        xmax = r.x + r.width
        ymax = r.y + r.height
        rects[i] = [xmin, ymin, xmax, ymax]
    return rects
