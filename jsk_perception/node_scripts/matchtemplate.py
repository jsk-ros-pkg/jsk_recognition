#!/usr/bin/env python

import roslib; roslib.load_manifest('jsk_perception')
import rospy
import numpy as np
import thread
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from jsk_recognition_msgs.msg import Rect
import cv2
from cv_bridge import CvBridge, CvBridgeError

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from dynamic_reconfigure.msg import SensorLevels
from jsk_perception.cfg import matchtemplateConfig as ConfigType

# you can set below options for each templates
#   color space: mono8,bgr8,bgra8,hsv,hsva (OpenCV expects bgr)
#   template size: tuple (x,y,width,height)
#   search area:   tuple (x,y,width,height)
#   search topic: (TODO)
#   match method: 6type

# publish topic:
#   current_template    [sensor_msgs/Image]
#   result              [geometry_msgs/TransformStamped]
#   debug_image         [sensor_msgs/Image]

# subscribe topic:
#   reference           [sensor_msgs/Image]
#   search              [sensor_msgs/Image]
#   set_reference_point [geometry_msgs/PointStamped]
#   set_search_rect     [jsk_recognition_msgs/Rect]

# return (1st-val,1st-loc,2nd-val,2nd-loc)
def _MinMaxLock2nd(arr,ex_size,is_min):
    if is_min: idx = 0
    else: idx = 1
    status = cv2.minMaxLoc(arr)
    pt1 = (max(status[2+idx][0]-ex_size[0]/2,0),
           max(status[2+idx][1]-ex_size[1]/2,0))
    pt2 = (min(status[2+idx][0]+ex_size[0]/2,arr.shape[1]),
           min(status[2+idx][1]+ex_size[1]/2,arr.shape[0]))
    mask = np.ones((arr.shape[0], arr.shape[1]), dtype=np.uint8) * 255
    mask[pt1[0]:pt2[0], pt1[1]:pt2[1]] = 0
    status2 = cv2.minMaxLoc(arr, mask)
    return (status[0+idx],status2[0+idx],status[2+idx],status2[2+idx])

def MinLock2nd(arr,ex_size):
    return _MinMaxLock2nd(arr,ex_size,True)
def MaxLock2nd(arr,ex_size):
    return _MinMaxLock2nd(arr,ex_size,False)


class MepConverter:
    bridge = CvBridge()

    def __init__(self):
        self.lockobj = thread.allocate_lock()
        # variables
        self.reference_image_msg = None
        self.templates = {} # frame_id : ref_image,ser_frame,ser_rect
        self.clock = [rospy.Time.now()]

        # publisher
        self.reference_pub = rospy.Publisher(
            "current_template", Image, queue_size=1)
        self.result_pub = rospy.Publisher(
            "result", TransformStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("debug_image", Image, queue_size=1)

        # subscriber
        self.reference_image_sub = rospy.Subscriber(rospy.resolve_name("reference"),
                                                    Image,self.ref_image_callback,queue_size=1)
        self.search_image_sub = rospy.Subscriber(rospy.resolve_name("search"),
                                                  Image,self.ser_image_callback,queue_size=1)
        self.reference_point_sub = rospy.Subscriber(
            rospy.resolve_name("set_reference_point"),
            PointStamped,self.set_reference_point_callback,queue_size=1)
        self.search_sub = rospy.Subscriber(rospy.resolve_name("set_search_rect"),
                                           Rect,self.set_search_callback,queue_size=1)

        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)

    # general template modifier function
    def set_template (self, ref_id='', ref_image=None, ref_rect=None,
                      ser_frame=None, ser_rect=None, color=None, method=None):
        if not self.templates.has_key(ref_id):
            self.templates[ref_id] = {'ref_orig':None,'ref_point':None,'ref_image':None,
                                      'ser_frame':None, 'ser_rect':None,
                                      'color':'mono8', 'method':cv2.TM_SQDIFF_NORMED} # normalized SSD
            color_changed = True
        else:
            color_changed = (self.templates[ref_id]['color'] != color) and (color != None)

        something_changed = (ref_image is not None) or \
                            (ref_rect is not None) or color_changed

        # parameters for template
        if ref_image is not None:
            self.templates[ref_id]['ref_orig'] = ref_image.copy()
        else:
            ref_image = self.templates[ref_id]['ref_orig'] # old image can be set here

        if color != None:
            if ref_image is not None and ref_image.ndim == 2:
                color = 'mono8'
            self.templates[ref_id]['color'] = color
        else:
            color = self.templates[ref_id]['color']

        # copy template from reference original image
        if (ref_rect is not None) and (ref_image is not None) and \
           something_changed:
            ref_rect = (min(max(0,ref_rect[0]),ref_image.shape[1]-ref_rect[2]),
                        min(max(0,ref_rect[1]),ref_image.shape[0]-ref_rect[3]),
                        ref_rect[2],ref_rect[3])
            ref_image_rect = ref_image[
                    ref_rect[1]:ref_rect[1] + ref_rect[3],
                    ref_rect[0]:ref_rect[0] + ref_rect[2]].copy()
            if color == 'bgr8' or (color == 'mono8' and ref_image.ndim == 2):
                template_image = ref_image_rect
            elif color == 'mono8':
                template_image = cv2.cvtColor(
                    ref_image_rect, cv2.COLOR_BGR2GRAY)
            elif color == 'hsv8':
                template_image = cv2.cvtColor(
                    ref_image_rect, cv2.COLOR_BGR2HSV)

            self.templates[ref_id]['ref_point'] = (ref_rect[0]+ref_rect[2]/2,
                                                   ref_rect[1]+ref_rect[3]/2)
            self.templates[ref_id]['ref_image'] = template_image
            rospy.loginfo("set ref_image id=%s, rect=%s", ref_id, ref_rect);

        # matching parameters
        if ser_frame != None:
            self.templates[ref_id]['ser_frame'] = ser_frame
            rospy.loginfo("set ser_frame id=%s frame=%s", ref_id, ser_frame);
        if ser_rect != None:
            self.templates[ref_id]['ser_rect'] = ser_rect
            #rospy.loginfo("set ser_rect id=%s %s", ref_id, ser_rect);
        if method != None:
            self.templates[ref_id]['method'] = method
            rospy.loginfo("set method id=%s method=%s", ref_id, method);

    def set_reference (self, rect):
        if self.reference_image_msg == None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                self.reference_image_msg, "bgr8")
            self.set_template('',ref_image=cv_image, ref_rect=rect)
        except CvBridgeError as e:
            print(e)

    #
    # Callback Functions (lock require)
    #

    # simple point tracking callback
    def set_reference_point_callback (self, msg): # PointStamped
        self.lockobj.acquire()
        pt = (int(msg.point.x),int(msg.point.y))
        rect = (pt[0]-self.default_template_size[0]/2,
                pt[1]-self.default_template_size[1]/2,
                self.default_template_size[0], self.default_template_size[1])
        self.set_reference(rect)
        print(rect)
        search_rect = (pt[0]-self.default_search_size[0]/2,
                       pt[1]-self.default_search_size[1]/2,
                       self.default_search_size[0],self.default_search_size[1])
        self.set_template('',ser_frame=None, ser_rect=search_rect)
        self.lockobj.release()

    # store the latest image
    def set_search_callback (self, msg):
        self.lockobj.acquire()
        self.set_template(ser_rect=(msg.x,msg.y,msg.width,msg.height))
        self.lockobj.release()

    # store the latest image
    def ref_image_callback (self, msg):
        self.lockobj.acquire()
        self.reference_image_msg = msg
        self.lockobj.release()

    # reconfigure
    def reconfigure(self,config,level):
        self.lockobj.acquire()
        # param
        self.default_template_size = (config['default_template_width'],
                                      config['default_template_height'])
        self.default_search_size = (config['default_search_width'],
                                    config['default_search_height'])
        self.show_debug_image = config['show_debug_image']
        self.auto_search_area = config['auto_search_area']

        if config['current_template_id'] in self.templates.keys():
            # set template configuration
            template = self.templates[config['current_template_id']]
            method_enum = [
                cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED, cv2.TM_CCORR,
                cv2.TM_CCORR_NORMED, cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED]
            if (template['ref_point'] is not None) and \
               (template['ref_image'] is not None):
                ref_pt = template['ref_point']
                ref_size = template['ref_image'].shape
                ref_rect = (ref_pt[0]-ref_size[0]/2,ref_pt[1]-ref_size[1]/2,ref_size[0],ref_size[1])
            self.set_template(ref_id=config['current_template_id'],
                              color=config['template_color_space'],
                              ref_rect=ref_rect,
                              method=method_enum[config['match_method']])
            # return current configuration
            config['template_color_method'] = template['color']
        self.lockobj.release()
        return config

    def ser_image_callback (self, msg):
        # initialize debug image
        if self.show_debug_image:
            debug_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.lockobj.acquire()

        # match for each template
        for template_id in self.templates.keys():
            reference_image = self.templates[template_id]['ref_image']
            search_rect = self.templates[template_id]['ser_rect']
            matchmethod = self.templates[template_id]['method']
            color_space = self.templates[template_id]['color']
            if reference_image is None:
                continue
            if search_rect == None: continue
            if (self.templates[template_id]['ser_frame'] != None and
                self.templates[template_id]['ser_frame'] != msg.header.frame_id): continue

            # search rect &= image size
            search_rect = (max(0,search_rect[0]),
                           max(0,search_rect[1]),
                           min(msg.width,search_rect[0]+search_rect[2])- max(0,search_rect[0]),
                           min(msg.height,search_rect[1]+search_rect[3])- max(0,search_rect[1]))

            result = TransformStamped(header=msg.header)

            try:
                if color_space == 'mono8':
                    search_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
                else:
                    search_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                    if color_space != 'bgr8':
                        search_image = cv2.cvtColor(
                            search_image, cv2.COLOR_BGR2HSV)

                image_size = search_image.shape
                reference_size = (
                    reference_image.shape[1], reference_image.shape[0])

                # search size < reference size
                if (search_rect[2] < reference_size[0]) or (search_rect[3] < reference_size[1]):
                    continue

                results = cv2.matchTemplate(
                    search_image[
                        search_rect[1]:search_rect[1] + search_rect[3],
                        search_rect[0]:search_rect[0] + search_rect[2]],
                    reference_image, matchmethod)

                if matchmethod in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                    status = MinLock2nd(results,reference_size) # minimum for SSD
                else:
                    status = MaxLock2nd(results,reference_size) # maximum for others
                    status = (1 - status[0], 1 - status[1], status[2], status[3])

                result_pt = (status[2][0]+search_rect[0]+reference_size[0]/2,
                             status[2][1]+search_rect[1]+reference_size[1]/2)

                # publish center position as result
                result.child_frame_id = template_id
                result.transform.translation.x = result_pt[0]
                result.transform.translation.y = result_pt[1]
                result.transform.rotation.w = 1 # not rotate, temporary
                self.result_pub.publish(result)

                if reference_image.ndim == 2 and \
                   reference_image.dtype == np.uint8:
                    ref_msg = self.bridge.cv2_to_imgmsg(
                        reference_image, "mono8")
                else:
                    ref_msg = self.bridge.cv2_to_imgmsg(
                        reference_image, "bgr8")
                self.reference_pub.publish(ref_msg)

                # self feedback
                if self.auto_search_area:
                    val_x = result_pt[0]-search_rect[2]/2
                    val_y = result_pt[1]-search_rect[3]/2
                    ser_scale = max(2,5+np.log(status[0])) # ???
                    new_ser_rect = (
                        min(max(val_x,0),image_size[0]-search_rect[2]),
                        min(max(val_y,0),image_size[1]-search_rect[3]),
                        reference_size[0]*ser_scale,reference_size[1]*ser_scale)
                    self.set_template(ser_rect=new_ser_rect)

                # draw on debug image
                if self.show_debug_image:
                    cv2.rectangle(debug_image,
                                  (result_pt[0] - reference_size[0]/2,
                                   result_pt[1] - reference_size[1]/2),
                                  (result_pt[0] + reference_size[0]/2,
                                   result_pt[1] + reference_size[1]/2),
                                  color=(0, 0, 255))  # red
                    cv2.rectangle(debug_image,
                                  (search_rect[0], search_rect[1]),
                                  (search_rect[0] + search_rect[2],
                                   search_rect[1] + search_rect[3]),
                                  color=(0, 255, 0))
                    cv2.putText(
                        debug_image, str(status[0]),
                        (search_rect[0], search_rect[1] + search_rect[3]),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1,
                        color=(0, 255, 0), thickness=2,
                        lineType=16)  # 16 means cv2.CV_AA

                    if reference_image.ndim == 2:
                        reference_color_image = cv2.cvtColor(
                            reference_image, cv2.COLOR_GRAY2BGR)
                        debug_image[
                            :reference_size[1], :reference_size[0], :] = \
                            reference_color_image.copy()
                    else:
                        debug_image[
                            :reference_size[1], :reference_size[0], :] = \
                            reference_image.copy()

            except CvBridgeError as e:
                print(e)

        self.lockobj.release()

        if self.show_debug_image:
            # calc and print fps (30frame avg)
            self.clock += [rospy.Time.now()]
            if 30 <= len(self.clock):
                fps = 30.0 / (self.clock[29]-self.clock[0]).to_sec()
                cv2.putText(
                    debug_image, '%.1f fps' % fps, (msg.width - 150, 40),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1,
                    color=(0, 0, 255),  # red
                    thickness=2, lineType=16)  # 16 means cv2.CV_AA
                self.clock = self.clock[-30:]
            # publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)

        return

if __name__=='__main__':
    rospy.init_node('match_template')

    obj = MepConverter()

    ## reconfigure
    rospy.loginfo("reference image %s, size %s", obj.reference_image_sub.resolved_name, obj.default_template_size)
    rospy.loginfo("   search_image %s, size %s", obj.search_image_sub.resolved_name, obj.default_search_size)

    rospy.spin()
