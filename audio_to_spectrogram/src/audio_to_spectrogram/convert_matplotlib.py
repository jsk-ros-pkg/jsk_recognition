try:
    # for python2.x
    from StringIO import StringIO as BufIO
except ModuleNotFoundError:
    # for python3.x
    from io import BytesIO as BufIO

import numpy as np
import PIL


def convert_matplotlib_to_img(fig):
    buf = BufIO()
    fig.savefig(buf, format="png")
    buf.seek(0)
    img = np.array(
        PIL.Image.open(buf), dtype=np.uint8)
    img = img[..., :3]
    return img
