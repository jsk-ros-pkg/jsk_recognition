#!/usr/bin/env python

import roslib; roslib.load_manifest('jsk_perception')
import rospy
import numpy
import thread
from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from jsk_perception.msg import *
import cv
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
#   ~/current_template    [sensor_msgs/Image]
#   ~/result              [geometry_msgs/TransformStamped]
#   ~/debug_image         [sensor_msgs/Image]

# subscribe topic:
#   ~/reference           [sensor_msgs/Image]
#   ~/search              [sensor_msgs/Image]
#   ~/set_reference_point [geometry_msgs/PointStamped]
#   ~/set_search_rect     [jsk_perception/Rect]

# return (1st-val,1st-loc,2nd-val,2nd-loc)
def _MinMaxLock2nd(arr,ex_size,is_min):
    if is_min: idx = 0
    else: idx = 1
    status = cv.MinMaxLoc(arr)
    pt1 = (max(status[2+idx][0]-ex_size[0]/2,0),
           max(status[2+idx][1]-ex_size[1]/2,0))
    pt2 = (min(status[2+idx][0]+ex_size[0]/2,cv.GetSize(arr)[0]),
           min(status[2+idx][1]+ex_size[1]/2,cv.GetSize(arr)[1]))
    mask = cv.CreateImage(cv.GetSize(arr), cv.IPL_DEPTH_8U, 1)
    cv.Set(mask,cv.Scalar(255))
    cv.SetImageROI(mask,(pt1[0],pt1[1],pt2[0]-pt1[0],pt2[1]-pt1[1]))
    cv.Set(mask,cv.Scalar(0))
    cv.ResetImageROI(mask)
    status2 = cv.MinMaxLoc(arr,mask)
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
        self.reference_pub = rospy.Publisher("current_template",Image)
        self.result_pub = rospy.Publisher("result",TransformStamped)
        self.debug_pub = rospy.Publisher("debug_image",Image)

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
                                      'color':'mono8', 'method':cv.CV_TM_SQDIFF_NORMED} # normalized SSD
            color_changed = True
        else:
            color_changed = (self.templates[ref_id]['color'] != color) and (color != None)

        something_changed = (ref_image != None) or (ref_rect != None) or color_changed

        # parameters for template
        if ref_image != None:
            self.templates[ref_id]['ref_orig'] = cv.CloneImage(ref_image)
        else:
            ref_image = self.templates[ref_id]['ref_orig'] # old image can be set here

        if color != None:
            if ref_image != None and ref_image.nChannels == 1:
                color = 'mono8'
            self.templates[ref_id]['color'] = color
        else:
            color = self.templates[ref_id]['color']

        # copy template from reference original image
        if (ref_rect != None) and (ref_image != None) and something_changed:
            ref_rect = (min(max(0,ref_rect[0]),ref_image.width-ref_rect[2]),
                        min(max(0,ref_rect[1]),ref_image.height-ref_rect[3]),
                        ref_rect[2],ref_rect[3])
            template_image = cv.CreateImage((ref_rect[2], ref_rect[3]),
                                         ref_image.depth,
                                         {'mono8':1,'bgr8':3,'hsv8':3}[color])
            cv.SetImageROI(ref_image, ref_rect)
            if color == 'mono8' and ref_image.nChannels == 1:
                cv.Copy(ref_image, template_image)
            elif color == 'mono8':
                cv.CvtColor(ref_image,template_image,cv.CV_BGR2GRAY)
            elif color == 'bgr8':
                cv.Copy(ref_image, template_image)
            elif color == 'hsv8':
                cv.CvtColor(ref_image,template_image,cv.CV_BGR2HSV)

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
            cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(self.reference_image_msg, "bgr8"))
            self.set_template('',ref_image=cv_image, ref_rect=rect)
        except CvBridgeError, e:
            print e

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
        print rect
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
            method_enum = [cv.CV_TM_SQDIFF, cv.CV_TM_SQDIFF_NORMED, cv.CV_TM_CCORR,
                           cv.CV_TM_CCORR_NORMED, cv.CV_TM_CCOEFF, cv.CV_TM_CCOEFF_NORMED]
            if (template['ref_point'] != None) and (template['ref_image'] != None):
                ref_pt = template['ref_point']
                ref_size = cv.GetSize(template['ref_image'])
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
            debug_image = cv.CloneImage(cv.GetImage(self.bridge.imgmsg_to_cv(msg, "bgr8")))

        self.lockobj.acquire()

        # match for each template
        for template_id in self.templates.keys():
            reference_image = self.templates[template_id]['ref_image']
            search_rect = self.templates[template_id]['ser_rect']
            matchmethod = self.templates[template_id]['method']
            color_space = self.templates[template_id]['color']
            if reference_image == None: continue
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
                if color_space in ['mono8','bgr8']:
                    search_image = cv.GetImage(self.bridge.imgmsg_to_cv(msg, color_space))
                else:
                    cv_image = cv.GetImage(self.bridge.imgmsg_to_cv(msg, 'bgr8'))
                    search_image = cv.CreateImage(cv.GetSize(cv_image),cv_image.depth,cv_image.nChannels)
                    cv.CvtColor(cv_image,search_image,cv.CV_BGR2HSV)

                image_size = cv.GetSize(search_image)
                reference_size = cv.GetSize(reference_image)

                # search size < reference size
                if (search_rect[2] < reference_size[0]) or (search_rect[3] < reference_size[1]):
                    continue

                cv.SetImageROI(search_image,search_rect)
                ressize = list(search_rect)[2:]
                ressize[0] -= reference_size[0] - 1
                ressize[1] -= reference_size[1] - 1
                results = cv.CreateImage(ressize, cv.IPL_DEPTH_32F, 1)
                cv.MatchTemplate(search_image, reference_image, results, matchmethod)
                cv.ResetImageROI(search_image)

                if matchmethod in [cv.CV_TM_SQDIFF,cv.CV_TM_SQDIFF_NORMED]:
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

                self.reference_pub.publish(self.bridge.cv_to_imgmsg(reference_image, "passthrough"))

                # self feedback
                if self.auto_search_area:
                    val_x = result_pt[0]-search_rect[2]/2
                    val_y = result_pt[1]-search_rect[3]/2
                    ser_scale = max(2,5+numpy.log(status[0])) # ???
                    new_ser_rect = (
                        min(max(val_x,0),image_size[0]-search_rect[2]),
                        min(max(val_y,0),image_size[1]-search_rect[3]),
                        reference_size[0]*ser_scale,reference_size[1]*ser_scale)
                    self.set_template(ser_rect=new_ser_rect)

                # draw on debug image
                if self.show_debug_image:
                    cv.Rectangle(debug_image,
                              (result_pt[0] - reference_size[0]/2,
                               result_pt[1] - reference_size[1]/2),
                              (result_pt[0] + reference_size[0]/2,
                               result_pt[1] + reference_size[1]/2),
                              cv.CV_RGB(255,0,0))
                    cv.Rectangle(debug_image,
                              (search_rect[0], search_rect[1]),
                              (search_rect[0] + search_rect[2],
                               search_rect[1] + search_rect[3]),
                              cv.CV_RGB(0,255,0))
                    cv.PutText(debug_image, str(status[0]), (search_rect[0], search_rect[1] + search_rect[3]), font, cv.CV_RGB(0,255,0))

                    cv.SetImageROI(debug_image,(0,0,reference_size[0],reference_size[1]))
                    if reference_image.nChannels == 1:
                        reference_color_image = cv.CreateImage(reference_size, debug_image.depth, debug_image.nChannels)
                        cv.CvtColor (reference_image, reference_color_image, cv.CV_GRAY2BGR) # TODO
                        cv.Copy(reference_color_image,debug_image)
                    else:
                        cv.Copy(reference_image,debug_image)
                    cv.ResetImageROI(debug_image)

            except CvBridgeError, e:
                print e

        self.lockobj.release()

        if self.show_debug_image:
            # calc and print fps (30frame avg)
            self.clock += [rospy.Time.now()]
            if 30 <= len(self.clock):
                fps = 30.0 / (self.clock[29]-self.clock[0]).to_sec()
                cv.PutText(debug_image, '%.1f fps'%fps, (msg.width-150,40),font,cv.CV_RGB(255,0,0))
                self.clock = self.clock[-30:]
            # publish debug image
            self.debug_pub.publish(self.bridge.cv_to_imgmsg(debug_image, "bgr8"))

        return

if __name__=='__main__':
    rospy.init_node('match_template')

    obj = MepConverter()

    ## reconfigure
    rospy.loginfo("reference image %s, size %s", obj.reference_image_sub.resolved_name, obj.default_template_size)
    rospy.loginfo("   search_image %s, size %s", obj.search_image_sub.resolved_name, obj.default_search_size)

    # opencv
    font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0.0, 2, cv.CV_AA)

    rospy.spin()
