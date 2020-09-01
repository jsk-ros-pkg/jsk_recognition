#!/usr/bin/env python
#
# Copyright (C) 2009 Rosen Diankov
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 
from __future__ import with_statement # for python 2.5
PKG = 'posedetectiondb' # this package name
NAME = 'PoseFromCorrespondences'

import roslib; roslib.load_manifest(PKG) 
import os, sys, time, threading, string, struct
from optparse import OptionParser
from numpy import *

import cv, gtk
import rospy, tf
from roslib import rostime
import std_msgs.msg
import posedetection_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

from IterativeClosestPoint import *
import ParseMessages

from openravepy import *

from Tkinter import *
import tkFileDialog
import copy

def FindExtrinsicCameraParams(imagepoints, objectpoints, KK):
    """ Use OpenCV to solve for the affine transformation that matches imagepoints to object points
    imagepoints - 2xN array
    objectpoints - 3xN array
    KK - 3x3 array or 4 element array
    """
    imagepoints = array(imagepoints,float)
    objectpoints = array(objectpoints,float)
    if len(KK.shape) == 1:
        cvKK = cv.CreateMat(3,3,cv.CV_32FC1)
        cvKK[0,0] = KK[0]; cvKK[0,1] = 0; cvKK[0,2] = KK[2];
        cvKK[1,0] = 0; cvKK[1,1] = KK[1]; cvKK[1,2] = KK[3];
        cvKK[2,0] = 0; cvKK[2,1] = 0; cvKK[2,2] = 1;
    else:
        cvKK = cv.fromarray(KK)
    cvDist = cv.CreateMat(4,1,cv.CV_32FC1)
    cvDist[0,0] = 0; cvDist[1,0] = 0; cvDist[2,0] = 0; cvDist[3,0] = 0;
    rvec = cv.CreateMat(3,1,cv.CV_32FC1)
    tvec = cv.CreateMat(3,1,cv.CV_32FC1)    
    object_points = cv.CreateMatHeader(3,objectpoints.shape[0],cv.CV_32FC1)
    cv.SetData(object_points,struct.pack('f'*(objectpoints.shape[0]*3),*transpose(objectpoints).flat),4*objectpoints.shape[0])
    image_points = cv.CreateMatHeader(2,imagepoints.shape[0],cv.CV_32FC1)
    cv.SetData(image_points,struct.pack('f'*(imagepoints.shape[0]*2),*transpose(imagepoints).flat),4*imagepoints.shape[0])
    cv.FindExtrinsicCameraParams2(object_points,image_points,cvKK,cvDist,rvec,tvec)
    T = matrixFromAxisAngle((rvec[0,0],rvec[1,0],rvec[2,0]))
    T[0:3,3] = [tvec[0,0],tvec[1,0],tvec[2,0]]
    return T

class PoseFromCorrespondences(metaclass.AutoReloader):
    """
    Extracts poses from a set of point correspondences
    """
    def __init__(self,kinbodyfilename,verboselevel=1,frame_id=None):
        self.orenv = Environment()
        self.orenv.Load(kinbodyfilename)
        self.orbody = self.orenv.GetBodies()[0]
        self.orbody.SetTransform(eye(4))
        self.trimesh = self.orenv.Triangulate(self.orbody)
        self.orenv.SetViewer('qtcoin')
        self.eventhandle = self.orenv.GetViewer().RegisterCallback(Viewer.Events.ItemSelection,self.ormousecb)
        self.orpoint = None
        self.objectpoints = []
        self.verboselevel=verboselevel
        self.extractionlck = threading.Lock()

        self.cvpoint = None
        self.Tobjectrel = None
        self.imagepoints = []

        self.cvwindow = 'ImageDisplay'
        cv.NamedWindow(self.cvwindow, cv.CV_WINDOW_AUTOSIZE)
        cv.SetMouseCallback(self.cvwindow,self.cvmousecb)
         # register keycb with opencv window
        self.bridge = CvBridge()

        self.doquit = False
        self.gui = threading.Thread(target=self.rungui)
        self.gui.start()

        self.pattern = (eye(4),None)
        self.imagemsg = None
        self.KK = None
        self.image_sub = rospy.Subscriber("image",sensor_msgs.msg.Image,self.imagecb)
        self.camerainfo_sub = rospy.Subscriber("camera_info",sensor_msgs.msg.CameraInfo,self.camerainfocb)
        self.frame_id = frame_id
        if self.frame_id is None:
            self.object_sub = rospy.Subscriber("ObjectDetection",posedetection_msgs.msg.ObjectDetection,self.objectcb)
        else:
            self.tflistener=tf.TransformListener()
        self.pub_relpose = rospy.Publisher("RelativePose", geometry_msgs.msg.Pose)
        rospy.init_node(NAME, anonymous=True)#,disable_signals=False)

    def __del__(self):
        try:
            self.image_sub.unregister()
        except:
            pass
        try:
            self.camerainfo_sub.unregister()
        except:
            pass
        try:
            self.object_sub.unregister()
        except:
            pass
        try:
            self.pub_relpose.unregister()
        except:
            pass
        self.orenv.Destroy()

    def ormousecb(self,link,pos,org):
        if link.GetParent().GetNetworkId() == self.orbody.GetNetworkId():
            T = linalg.inv(link.GetParent().GetTransform())
            bodypos = dot(T[0:3,0:3],pos)+T[0:3,3]
            self.orpoint = bodypos
            self.ghandle = self.orenv.plot3(points=reshape(bodypos,(1,3)),pointsize=15.0,colors=array((1,0,0)))
        return False

    def cvmousecb(self,event,x,y,flags,param):
        if event == cv.CV_EVENT_LBUTTONUP:
            self.cvpoint = (x,y)
        if event == cv.CV_EVENT_RBUTTONUP:
            self.AddCorrespondence()
        if event == cv.CV_EVENT_MBUTTONUP:
            self.Reset()

    def rungui(self):
        self.main = Tk()
        self.main.title('Create Object Database - 3 channel image')      # window title
        self.main.resizable(width=False, height=False)
        buttonframe = Frame(self.main)
        buttonframe.pack()
        b1 = Button(buttonframe, text="Add Correspondence (R-Button)", command=self.AddCorrespondence)
        b1.grid(row=1,column=1)
        b2 = Button(buttonframe, text="Reset (M-Button)", command=self.Reset)
        b2.grid(row=1,column=2)
        b2 = Button(buttonframe, text="Quit", command=self.Quit)
        b2.grid(row=1,column=3)
       # b4 = Button(buttonframe, text="Keyboard", command=self.keycb)
       # b4.grid(row=1,column=4)
        entryframe = Frame(self.main)
        entryframe.pack()
        ltrans = Label(entryframe, text='Transform:')
        ltrans.grid(row=1,column=1,sticky=W+E)
        self.T_entry = Entry(entryframe,bg='white',width=100)
        self.T_entry.grid(row=1,column=2)
       # self.T_entry.grid(row=1,column=4)
        self.main.mainloop()

    def camerainfocb(self,infomsg):
        with self.extractionlck:
            self.KK = reshape(infomsg.K,(3,3))
            if any([f!=0 for f in infomsg.D]):
                print('Program does not support distorted images')

    def imagecb(self,imagemsg):
        with self.extractionlck:
            self.imagemsg = imagemsg

    def objectcb(self,objectmsg):
        with self.extractionlck:
            if len(objectmsg.objects) > 0:
                quat = objectmsg.objects[0].pose.orientation
                trans = objectmsg.objects[0].pose.position
                self.pattern = (matrixFromPose([quat.w,quat.x,quat.y,quat.z,trans.x,trans.y,trans.z]),objectmsg.header.frame_id)
            else:
                self.pattern = (None,None)

    def AddCorrespondence(self):
        with self.extractionlck:
            print('add correspondence')
#             if self.frame_id:
#                 base_frame_id = self.orbody.GetLinks()[0].GetName()
#                 (trans,rot) = self.lookupTransform(base_frame_id, self.frame_id, rospy.Time(0))
#                 pose = r_[rot[3],rot[0],rot[1],rot[2],trans]
#                 self.pattern = (matrixFromPose(pose),base_frame_id)
            if self.cvpoint is not None and self.orpoint is not None and self.pattern[0] is not None:
                Tpattern_inv = linalg.inv(self.pattern[0])
                self.imagepoints.append(array(self.cvpoint))
                self.objectpoints.append(array(self.orpoint))
                print('total gathered points: %d'%len(self.imagepoints))
                if len(self.imagepoints) >= 4:
                    print(array(self.imagepoints))
                    print(array(self.objectpoints))
                    print(self.pattern[0])
                    Tcameraobject = FindExtrinsicCameraParams(array(self.imagepoints,float),array(self.objectpoints,float),self.KK)
                    self.Tobjectrel = dot(Tpattern_inv,Tcameraobject)
                    print('camera transform: ', Tcameraobject)
                    values = reshape(self.Tobjectrel[0:3,0:4],(12,))
                    print("relative transform is: ",self.Tobjectrel)
                    self.T_entry.insert(0, ' '.join(str(f) for f in values))
            else:
                print('point data not initialized')

    def Reset(self):
        print('reset')
        self.imagepoints = []
        self.objectpoints = []
        self.Tobjectrel = None
    def Quit(self):
        print('quitting from gui')
        self.doquit = True
        self.main.quit()

    def drawpart(self,cv_image,T,KK):
        N = self.trimesh.vertices.shape[0]
        pts = dot(transformPoints(T,self.trimesh.vertices),transpose(KK))
        imagepts = pts[0:N,0:2]/reshape(repeat(pts[0:N,2],2),[N,2])
        cvimagepts = [tuple(p) for p in array(imagepts,int)]
        for tri in self.trimesh.indices:
            cv.Line(cv_image,cvimagepts[tri[0]],cvimagepts[tri[1]],(255,255,255))
            cv.Line(cv_image,cvimagepts[tri[1]],cvimagepts[tri[2]],(255,255,255))
            cv.Line(cv_image,cvimagepts[tri[2]],cvimagepts[tri[0]],(255,255,255))

    def drawcoordsys(self,cv_image,T,KK):
        points3d = array(((0,0,0),(0.05,0,0),(0,0.05,0),(0,0,0.05)))
        projpts = dot(transformPoints(T,points3d),transpose(KK))
        x = array(projpts[:,0]/projpts[:,2],int)
        y = array(projpts[:,1]/projpts[:,2],int)
        cv.Line(cv_image,(x[0],y[0]),(x[1],y[1]),(0,0,255),1)
        cv.Line(cv_image,(x[0],y[0]),(x[2],y[2]),(0,255,0),1)
        cv.Line(cv_image,(x[0],y[0]),(x[3],y[3]),(255,0,0),1)

    def keycb(self, char):
        a=1
        if (char != -1):
            with self.extractionlck:
                minangle=pi/200
                mintranslation=.001
                if char==1113955: #NumLock Insert
                    R=rotationMatrixFromAxisAngle(array([1,0,0]),minangle)
                    T=array([0,0,0])
                elif char==1114111: #Num Lock Delete
                    R=rotationMatrixFromAxisAngle(array([1,0,0]),-minangle)
                    T=array([0,0,0])
                elif char==1113936: #NumLock Home
                    R=rotationMatrixFromAxisAngle(array([0,1,0]),minangle)
                    T=array([0,0,0])
                elif char==1113943: #Num Lock End
                    R=rotationMatrixFromAxisAngle(array([0,1,0]),-minangle)
                    T=array([0,0,0])
                elif char==1113941: #Num Lock Page Up
                    R=rotationMatrixFromAxisAngle(array([0,0,1]),minangle)
                    T=array([0,0,0])
                elif char==1113942: #Num Lock Page Down
                    R=rotationMatrixFromAxisAngle(array([0,0,1]),-minangle)
                    T=array([0,0,0])
                elif char==65379: #Insert
                    R=eye(3)
                    T=array([1,0,0])
                elif char==65535: #Delete
                    R=eye(3)
                    T=array([-1,0,0])
                elif char==65360: #Home
                    R=eye(3)
                    T=array([0,1,0])
                elif char==65367: #End
                    R=eye(3)
                    T=array([0,-1,0])
                elif char==65365: #Page Up
                    R=eye(3)
                    T=array([0,0,1])
                elif char==65366: #Page Down
                    R=eye(3)
                    T=array([0,0,-1])
                else:
                    a=0
                if a==1:
                    self.Tobjectrel[:3,:3]=dot(R,self.Tobjectrel[:3,:3])
                    self.Tobjectrel[:3,3]=self.Tobjectrel[:3,3]+mintranslation*T
                    print("relative: ",self.Tobjectrel)
                    
    def spin(self):
        while not rospy.is_shutdown() and not self.doquit:
            with self.extractionlck:
                imagemsg = copy.copy(self.imagemsg)
                KK = array(self.KK)
                Tobjectrel = array(self.Tobjectrel) if self.Tobjectrel is not None else None
                Tpattern = array(self.pattern[0]) if self.pattern[0] is not None else None

            if KK is None or imagemsg is None:
                time.sleep(0.1)
                continue

            try:
                cv_image = self.bridge.imgmsg_to_cv(imagemsg, "bgr8")
            except CvBridgeError as e:
                print(e)

            if Tpattern is not None:
                if Tobjectrel is not None:
                    self.drawpart(cv_image,dot(Tpattern,Tobjectrel),KK)
                self.drawcoordsys(cv_image,Tpattern,KK)
            if self.cvpoint is not None:
                cv.Circle(cv_image,self.cvpoint,2,(0,0,255),2)

            if Tobjectrel is not None:
                posemsg = geometry_msgs.msg.Pose()
                posemsg.orientation.w, posemsg.orientation.x, posemsg.orientation.y, posemsg.orientation.z, posemsg.position.x, posemsg.position.y, posemsg.position.z = poseFromMatrix(Tobjectrel)
                self.pub_relpose.publish(posemsg)
            gtk.gdk.threads_enter()
            cv.ShowImage(self.cvwindow, cv_image)
            char=cv.WaitKey(20)
            self.keycb(char)
            gtk.gdk.threads_leave()
            time.sleep(1.0)
        print('quitting spin')
            
if __name__== '__main__':
    parser = OptionParser(description='Estiamtes the pose of an openrave object by specifying manual correspondeces between the image and the openrave environment. If a separate ObjectDetection pattern is added, will publish the pose of the object with respect to the pattern.')
    parser.add_option('--quiet',action='store_true', dest='quiet',default=False,
                      help="If set will not print extraction output and times.")
    parser.add_option('--kinbody', action='store', type='string', dest='kinbody',
                      help='OpenRAVE Kinbody file to load')
#     parser.add_option('--frame_id', action='store', type='string', dest='frame_id',default=None,
#                       help='tf frame id for possible frame that camera is connected to')
    parser.add_option('--Tobjectrel', action='store', type='string', dest='Tobjectrel',default=None,
                      help='Initial estimate of Tobject (3x4 matrix serialized by row-order)')
    (options, args) = parser.parse_args()
    if not options.kinbody:
        print('Error: Need to specify a template')
        sys.exit(1)

    gtk.gdk.threads_init()

    try:
        processor = PoseFromCorrespondences(options.kinbody)
        if options.Tobjectrel is not None:
            processor.Tobjectrel = r_[reshape([float(s) for s in options.Tobjectrel.split()[0:12]],[3,4]),[[0,0,0,1]]]
        processor.spin()
    except KeyboardInterrupt as e:
        pass
