PKG = 'posedetectiondb' # this package name
import roslib; roslib.load_manifest(PKG) 
import os, sys, signal, time, threading, struct, string
import numpy, scipy
import PIL.Image

import rospy
import sensor_msgs.msg

# Number of channels used by a PIL image mode
def imgmsg_to_pil(rosimage, encoding_to_mode = {
        'mono8' :     'L',
        '8UC1' :      'L',
        '8UC3' :      'RGB',
        'rgb8':       'RGB',
        'bgr8':       'RGB',
        'rgba8':      'RGBA',
        'bgra8':      'RGBA',
        'bayer_rggb': 'L',
        'bayer_gbrg': 'L',
        'bayer_grbg': 'L',
        'bayer_bggr': 'L',
        'yuv422':     'YCbCr',
        'yuv411':     'YCbCr'}, PILmode_channels = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }):
    conversion = 'B'
    channels = 1
    if rosimage.encoding.find('32FC') >= 0:
        conversion = 'f'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('64FC') >= 0:
        conversion = 'd'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('8SC') >= 0:
        conversion = 'b'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('8UC') >= 0:
        conversion = 'B'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('16UC') >= 0:
        conversion = 'H'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('16SC') >= 0:
        conversion = 'h'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('32UC') >= 0:
        conversion = 'I'
        channels = int(rosimage.encoding[-1])
    elif rosimage.encoding.find('32SC') >= 0:
        conversion = 'i'
        channels = int(rosimage.encoding[-1])
    else:
        if rosimage.encoding.find('rgb') >= 0 or rosimage.encoding.find('bgr') >= 0:
            channels = 3

    data = struct.unpack( ('>' if rosimage.is_bigendian else '<') + '%d'%(rosimage.width*rosimage.height*channels) + conversion,rosimage.data)
    
    if conversion == 'f' or conversion == 'd':
        dimsizes = [rosimage.height, rosimage.width, channels]
        imagearr = numpy.array(255*I,dtype=numpy.uint8)
        im = PIL.Image.frombuffer('RGB' if channels == 3 else 'L',dimsizes[1::-1],imagearr.tostring(), 'raw','RGB',0,1)
        if channels == 3:
            im = PIL.Image.merge('RGB',im.split()[-1::-1])
        return im,data,dimsizes
    else:
        mode = encoding_to_mode[rosimage.encoding]
        step_size = PILmode_channels[mode]
        dimsizes = [rosimage.height, rosimage.width, step_size]
        im = PIL.Image.frombuffer(mode,dimsizes[1::-1], rosimage.data,'raw',mode,0,1)
        if mode == 'RGB':
            im = PIL.Image.merge('RGB',im.split()[-1::-1])
        return im,data,dimsizes

def pil_to_imgmsg(image,encodingmap={'L':'mono8', 'RGB':'rgb8','RGBA':'rgba8', 'YCbCr':'yuv422'},
                        PILmode_channels = { 'L' : 1, 'RGB' : 3, 'RGBA' : 4, 'YCbCr' : 3 }):
    rosimage = sensor_msgs.msg.Image()
    rosimage.encoding = encodingmap[image.mode]
    (rosimage.width, rosimage.height) = image.size
    rosimage.step = PILmode_channels[image.mode] * rosimage.width
    rosimage.data = image.tostring()
    return rosimage

def numpy_to_imgmsg(image,stamp=None):
    rosimage = sensor_msgs.msg.Image()
    rosimage.height = image.shape[0]
    rosimage.width = image.shape[1]
    if image.dtype == numpy.uint8:
        rosimage.encoding = '8UC%d' % image.shape[2]
        rosimage.step = image.shape[2] * rosimage.width
        rosimage.data = image.ravel().tolist()
    else:
        rosimage.encoding = '32FC%d' % image.shape[2]
        rosimage.step = image.shape[2] * rosimage.width * 4
        rosimage.data = numpy.array(image.flat,dtype=numpy.float32).tostring()
    if stamp is not None:
        rosimage.header.stamp = stamp
    return rosimage
