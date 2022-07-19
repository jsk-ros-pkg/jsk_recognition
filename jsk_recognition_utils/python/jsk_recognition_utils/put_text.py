import sys

import numpy as np
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


def put_text_to_image(
        img, text, pos, font_path, font_size, color, background_color=None,
        offset_x=0, offset_y=0, loc='top'):
    """Put text to image using pillow.

    You can put text to an image including non-ASCII characters.

    Parameters
    ==========
    img : numpy.ndarray
        cv2 image. bgr order.
    text : str
        text information.
    pos : tuple(float)
        xy position of text.
    font_path : str
        path to font.
    font_size : int
        font size
    color : tuple(int)
        text color
    background_color : tuple(int) or None
        background color in text area. If this value is None, do nothing.
    offset_x : float
        x position offset.
    offset_y : float
        y position offset.
    loc : str
        location.
    """
    if sys.version_info < (3, 0):
        text = text.decode('utf-8')
    pil_font = ImageFont.truetype(font=font_path, size=font_size)
    dummy_draw = ImageDraw.Draw(Image.new("RGB", (0, 0)))
    text_w, text_h = dummy_draw.textsize(text, font=pil_font)
    text_bottom_offset = int(0.1 * text_h)
    x, y = pos
    if loc == 'top':
        offset_y = (text_h + text_bottom_offset) + offset_y
    elif loc == 'center':
        offset_y = offset_y
    else:
        raise NotImplementedError('loc {} not implemented.'.format(loc))
    x0 = x - offset_x
    y0 = y - offset_y
    img_h, img_w = img.shape[:2]
    # check outside of image.
    if not ((-text_w < x0 < img_w)
            and (-text_bottom_offset - text_h < y0 < img_h)):
        return img

    x1, y1 = max(x0, 0), max(y0, 0)
    x2 = min(x0+text_w, img_w)
    y2 = min(y0 + text_h + text_bottom_offset, img_h)
    x0 = int(x0)
    y0 = int(y0)
    x1 = int(x1)
    y1 = int(y1)
    x2 = int(x2)
    y2 = int(y2)

    # Create a black image of the same size as the text area.
    text_area = np.full(
        (text_h + text_bottom_offset, text_w, 3),
        (0, 0, 0), dtype=np.uint8)
    if background_color is not None:
        img[y1:y2, x1:x2] = np.array(background_color, dtype=np.uint8)
    # paste the original image on all or part of it.
    text_area[y1-y0:y2-y0, x1-x0:x2-x0] = img[y1:y2, x1:x2]

    # convert pil image to cv2 image.
    if not (text_area.shape[0] == 0 or text_area.shape[0] == 0):
        pil_img = Image.fromarray(text_area)
        draw = ImageDraw.Draw(pil_img)
        draw.text(xy=(0, 0), text=text, fill=color, font=pil_font)

        text_area = np.array(pil_img, dtype=np.uint8)
        img[y1:y2, x1:x2] = text_area[y1-y0:y2-y0, x1-x0:x2-x0]
    return img
