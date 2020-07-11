#!/usr/bin/env python
#
# Copyright (C) 2009-2010 Rosen Diankov
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
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

PKG = 'posedetectiondb' # this package name
import roslib; roslib.load_manifest(PKG) 
import os, sys, time, threading, struct, pickle
from optparse import OptionParser
import numpy, scipy # nice to be able to explicitly call some functions
from numpy import *

from openravepy import *
from openravepy import pyANN
from openravepy.misc import *

import cv
from cv_bridge import CvBridge, CvBridgeError

import rospy
from roslib import rostime
import std_msgs.msg
import posedetection_msgs.msg
import posedetection_msgs.srv
from posedetectiondb.srv import *

class detection_error(Exception):
    def __init__(self,parameter=''):
        self.parameter = parameter
    def __str__(self):
        return 'detection error: '+repr(self.parameter)

class PointPoseExtractor(metaclass.AutoReloader):
    """Core 2D/3D correspondence algorithm"""
    def __init__(self,type,points3d,descriptors):
        if len(descriptors) < 1:
            raise detection_error('no descriptor found')
        self.type = type
        self.points3d = points3d
        self.desckdtree = pyANN.KDTree(descriptors)
        self.besterr_thresh = 0.001
        self.cvKK = cv.fromarray(eye(3))
        self.cvDist = cv.fromarray(zeros((4,1)))
        self.rvec = cv.CreateMat(3,1,cv.CV_32FC1)
        self.tvec = cv.CreateMat(3,1,cv.CV_32FC1)
        self.ninitial = 4
        self.object_points = cv.CreateMatHeader(3,self.ninitial,cv.CV_32FC1)
        self.image_points = cv.CreateMatHeader(2,self.ninitial,cv.CV_32FC1)

    def extractpose(self,KK,positions,descriptors,conf,imagewidth,verboselevel=1,neighthresh=None,thresh=None,dminexpected=None,ransaciters=None):
        starttime = time.time()
        neighs,dists = self.desckdtree.kSearchArray(descriptors,2,0.0001)
        if neighthresh is None:
            neighthresh = 0.8-imagewidth*0.3/1024.0
        goodinds = flatnonzero(dists[:,0] < neighthresh**2 * dists[:,1])
        if len(goodinds) < 20:
            raise detection_error('not enough good indices')
        iKK = linalg.inv(KK)
        points2d = dot(positions[goodinds],transpose(iKK[0:2,0:2])) + tile(iKK[0:2,2],(len(goodinds),1))
        data = c_[points2d,self.points3d[neighs[goodinds,0],:]]#,conf[goodinds])
        if dminexpected is None:
            dminexpected = min(10+int(imagewidth*40.0/1024.0),len(goodinds)/2)
        if thresh is None:
            thresh = ((imagewidth/1024.0)*15.0/KK[0,0])
        if ransaciters is None:
            ransaciters = 1000
        if verboselevel > 0:
            print('thresh=%f,neighthresh=%f,dmin=%d,ransaciters=%d,goodinds=%d'%(thresh,neighthresh,dminexpected,ransaciters,len(goodinds)))
        T,infodict = self.ransac(data=data,model=self,n=self.ninitial,k=ransaciters,t=thresh,d=dminexpected,return_all=True,besterr_thresh=self.besterr_thresh)
        inliers = infodict['inliers']
        pts = transformPoints(T,data[inliers,2:5])
        iz = 1/pts[:,2]
        projerror = mean(sqrt((data[inliers,0]-pts[:,0]*iz)**2 + (data[inliers,1]-pts[:,1]*iz)**2))
        if verboselevel>0:
            print('extract time: %fs, err: %f, inliers: %d'%(time.time()-starttime,projerror,len(inliers)))
        return T, projerror

    def fit(self,data):
        m = mean(data,0)
        diff = data-tile(m,(data.shape[0],1))
        area0 = abs(linalg.det(dot(transpose(diff[:,0:2]),diff[:,0:2])))
        if area0 <0.00001: # check if point area is large enough
            #print 'degenerate 2d data %f'%area0
            return None
        # have to compute if the 3d points are collinear or not
        eigvalues = linalg.eigvalsh(dot(transpose(diff[:,2:5]),diff[:,2:5]))
        if sum(abs(eigvalues)<=1e-9) >= 2: # check if point area is large enough
            #print 'degenerate 3d points',eigvalues
            return None
        if data.shape[0] == self.ninitial:
            object_points = self.object_points
            image_points = self.image_points
        else:
            object_points = cv.CreateMatHeader(3,data.shape[0],cv.CV_32FC1)
            image_points = cv.CreateMatHeader(2,data.shape[0],cv.CV_32FC1)
        cv.SetData(object_points,struct.pack('f'*(data.shape[0]*3),*transpose(data[:,2:5]).flat),4*data.shape[0])
        cv.SetData(image_points,struct.pack('f'*(data.shape[0]*2),*transpose(data[:,0:2]).flat),4*data.shape[0])
        cv.FindExtrinsicCameraParams2(object_points,image_points,self.cvKK,self.cvDist,self.rvec,self.tvec)
        #cv.FindExtrinsicCameraParams2(cv.fromarray(data[:,2:5]),cv.fromarray(data[:,0:2]),self.cvKK,self.cvDist,self.rvec,self.tvec)
        T = matrixFromAxisAngle((self.rvec[0,0],self.rvec[1,0],self.rvec[2,0]))
        T[0:3,3] = [self.tvec[0,0],self.tvec[1,0],self.tvec[2,0]]
        # make sure that texture faces towards the image (ie, z axis has negative z component)
        if T[2,2] < 0:
            return None
        return T
    def get_error(self,data,T):
        if T is None:
            return inf
        pts = transformPoints(T,data[:,2:5])
        iz = 1/pts[:,2]
        return sqrt((data[:,0]-pts[:,0]*iz)**2 + (data[:,1]-pts[:,1]*iz)**2)
    def ransac(self,data,model,n,k,t,d,debug=False,return_all=False,besterr_thresh=None):
        """fit model parameters to data using the RANSAC algorithm

        This implementation written from pseudocode found at
        http://en.wikipedia.org/w/index.php?title=RANSAC&oldid=116358182

        {{{
        Given:
            data - a set of observed data points
            model - a model that can be fitted to data points
            n - the minimum number of data values required to fit the model
            k - the maximum number of iterations allowed in the algorithm
            t - a threshold value for determining when a data point fits a model
            d - the number of close data values required to assert that a model fits well to data
        Return:
            bestfit - model parameters which best fit the data (or nil if no good model is found)
        iterations = 0
        bestfit = nil
        besterr = something really large
        while iterations < k {
            maybeinliers = n randomly selected values from data
            maybemodel = model parameters fitted to maybeinliers
            alsoinliers = empty set
            for every point in data not in maybeinliers {
                if point fits maybemodel with an error smaller than t
                     add point to alsoinliers
            }
            if the number of elements in alsoinliers is > d {
                % this implies that we may have found a good model
                % now test how good it is
                bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
                thiserr = a measure of how well model fits these points
                if thiserr < besterr {
                    bestfit = bettermodel
                    besterr = thiserr
                }
            }
            increment iterations
        }
        return bestfit
        }}}

         Copyright (c) 2004-2007, Andrew D. Straw. All rights reserved.

         Redistribution and use in source and binary forms, with or without
         modification, are permitted provided that the following conditions are
         met:
             * Redistributions of source code must retain the above copyright
               notice, this list of conditions and the following disclaimer.

             * Redistributions in binary form must reproduce the above
               copyright notice, this list of conditions and the following
               disclaimer in the documentation and/or other materials provided
               with the distribution.

             * Neither the name of the Andrew D. Straw nor the names of its
               contributors may be used to endorse or promote products derived
               from this software without specific prior written permission.
        """
        iterations = 0
        bestfit = None
        besterr = numpy.inf
        best_inlier_idxs = None
        while iterations < k:
            maybe_idxs, test_idxs = self.random_partition(n,data.shape[0])
            maybeinliers = data[maybe_idxs,:]
            test_points = data[test_idxs]
            maybemodel = model.fit(maybeinliers)
            if maybemodel is None:
                iterations+=1
                continue
            test_err = model.get_error( test_points, maybemodel)
            also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
            if debug:
                print('test_err.min()',test_err.min())
                print('test_err.max()',test_err.max())
                print('numpy.mean(test_err)',numpy.mean(test_err))
                print('iteration %d:len(alsoinliers) = %d'%(
                    iterations,len(also_idxs)))
            if len(also_idxs) > d:
                alsoinliers = data[also_idxs,:]
                betterdata = numpy.concatenate( (maybeinliers, alsoinliers) )
                bettermodel = model.fit(betterdata)
                better_errs = model.get_error( betterdata, bettermodel)
                thiserr = numpy.mean( better_errs )
                if thiserr < besterr:
                    bestfit = bettermodel
                    besterr = thiserr
                    best_inlier_idxs = numpy.concatenate( (maybe_idxs, also_idxs) )
                    if besterr_thresh is not None and besterr < besterr_thresh:
                        break
            iterations+=1
        if bestfit is None:
            raise detection_error("did not meet fit acceptance criteria")
        if return_all:
            return bestfit, {'inliers':best_inlier_idxs}
        else:
            return bestfit

    def random_partition(self,n,n_data):
        """return n random rows of data (and also the other len(data)-n rows)"""
        all_idxs = numpy.arange( n_data )
        numpy.random.shuffle(all_idxs)
        idxs1 = all_idxs[:n]
        idxs2 = all_idxs[n:]
        return idxs1, idxs2

class ROSPlanarPoseProcessor(metaclass.AutoReloader):
    """Connects to ros feature0d message, extracts the pose, and publishes it"""
    def __init__(self,type=None,templateppfilename=None,errorthresh=None,neighthresh=None,thresh=None,dminexpected=None,Tright=eye(4),showgui=False,verboselevel=1,ransaciters=None):
        self.verboselevel=verboselevel
        self.Tright = Tright
        self.type=type
        self.pe = None
        self.boundingbox=None
        self.cv_image=None
        self.pe = None
        self.templateppfilename = templateppfilename
        self.featuretype = None
        if self.templateppfilename is not None:
            try:
                template = pickle.load(open(templateppfilename,'r'))
                self.type=template['type']
                self.Itemplate = template.get('Itemplate',None)
                self.boundingbox = template.get('boundingbox',None)
                self.Tright = template.get('Tright',eye(4))
                self.featuretype = template.get('featuretype',None)
                self.pe = PointPoseExtractor(type=self.type,points3d=template['points3d'],descriptors=template['descriptors'])
            except IOError as e:
                print('failed to create template: ',e)
        if self.templateppfilename is None:
            self.templateppfilename = 'template.pp'
        self.errorthresh = errorthresh
        self.neighthresh=neighthresh
        self.thresh=thresh
        self.dminexpected=dminexpected
        self.ransaciters=ransaciters
        self.imagepoints = []
        self.extractionlck = threading.Lock()
        self.bridge = CvBridge()
        self.cvwindow = None
        self.cvwindowstarted=False
        self.trimesh = None
        if showgui:
            # get the trimesh from openrave
            env = Environment()
            try:
                if self.type is not None:
                    if self.type.startswith('package://'):
                        packagesplit = self.type[10:].find('/')
                        filename = roslib.packages.get_pkg_dir(self.type[10:(10+packagesplit)])+'/'+self.type[(10+packagesplit+1):]
                    else:
                        filename = self.type
                    env = Environment()
                    body = env.ReadKinBodyXMLFile(filename)
                    env.AddKinBody(body)
                    self.trimesh = env.Triangulate(body)
            except openrave_exception as e:
                print('cannot create trimesh for %s: '%self.type,e)
            finally:
                env.Destroy()
            self.cvwindow = 'ImageDisplay (L-add point,R-reset)'
        self.pub_objdet = rospy.Publisher('ObjectDetection', posedetection_msgs.msg.ObjectDetection)
        self.sub_feature = rospy.Subscriber('ImageFeature0D', posedetection_msgs.msg.ImageFeature0D,self.featureimagecb, queue_size=1)
        #self.srv_detection = rospy.Service('Detect',posedetection_msgs.srv.Detect,self.detect)
        self.srv_settemplate = rospy.Service('SetTemplate',SetTemplate,self.SetTemplateFn)


    def __del__(self):
        print('deleting gui')
        try:
            self.sub_feature.unregister()
        except:
            pass
        try:
            self.pub_pose.unregister()
        except:
            pass
    def cvmousecb(self,event,x,y,flags,param):
        self.pe=None
        if event == cv.CV_EVENT_LBUTTONUP:
            #with self.extractionlck:
            print('add correspondence',x,y)
            self.imagepoints.append(array((x,y),float))
        elif event == cv.CV_EVENT_RBUTTONUP:
            #with self.extractionlck:
            print('reset')
            self.imagepoints = []
            self.pe=None
    def drawpart(self,cv_image,T,KK):
        if self.trimesh is not None:
            vertices=self.trimesh.vertices
            indices=self.trimesh.indices
        elif self.boundingbox is not None:
            vertices,indices=ComputeBoxMesh(0.5*(self.boundingbox[1,:]-self.boundingbox[0,:]))
            vertices += tile(0.5*(self.boundingbox[1,:]+self.boundingbox[0,:]),(len(vertices),1))
        else:
            return
        N = vertices.shape[0]
        pts = dot(transformPoints(T,vertices),transpose(KK))
        imagepts = pts[0:N,0:2]/reshape(repeat(pts[0:N,2],2),[N,2])
        cvimagepts = [tuple(p) for p in array(imagepts,int)]
        for tri in indices:
            cv.Line(cv_image,cvimagepts[tri[0]],cvimagepts[tri[1]],(255,255,255),thickness=1)
            cv.Line(cv_image,cvimagepts[tri[1]],cvimagepts[tri[2]],(255,255,255),thickness=1)
            cv.Line(cv_image,cvimagepts[tri[2]],cvimagepts[tri[0]],(255,255,255),thickness=1)
    def drawcoordsys(self,cv_image,T,KK):
        points3d = array(((0,0,0),(0.05,0,0),(0,0.05,0),(0,0,0.05)))
        projpts = dot(transformPoints(T,points3d),transpose(KK))
        x = array(projpts[:,0]/projpts[:,2],int)
        y = array(projpts[:,1]/projpts[:,2],int)
        cv.Line(cv_image,(x[0],y[0]),(x[1],y[1]),(0,0,255),thickness=3)
        cv.Line(cv_image,(x[0],y[0]),(x[2],y[2]),(0,255,0),thickness=3)
        cv.Line(cv_image,(x[0],y[0]),(x[3],y[3]),(255,0,0),thickness=3)

    def SetTemplateFn(self,req):
        with self.extractionlck:
            try:
                cv_image = self.bridge.imgmsg_to_cv(req.image, "bgr8")
                res=rospy.ServiceProxy('Feature0DDetect',posedetection_msgs.srv.Feature0DDetect)(image=req.image)
                N = len(res.features.positions)/2
                positions = reshape(res.features.positions,(N,2))
                descriptors = reshape(res.features.descriptors,(N,res.features.descriptor_dim))
                points3d=c_[positions[:,0]*req.dimx/req.image.width,positions[:,1]*req.dimy/req.image.height,zeros(len(positions))]
                self.Itemplate = array(cv_image)
                self.Itemplate[:,:,(0,2)] = self.Itemplate[:,:,(2,0)]
                self.Tright = matrixFromPose([req.relativepose.orientation.w,req.relativepose.orientation.x,req.relativepose.orientation.y,req.relativepose.orientation.z,req.relativepose.position.x,req.relativepose.position.y,req.relativepose.position.z])
                self.boundingbox=transformPoints(linalg.inv(self.Tright),array(((0,0,0),(req.dimx,req.dimy,0))))
                self.type=req.type
                template = {'type':self.type,'points3d':points3d,'descriptors':descriptors,'Itemplate':self.Itemplate,'boundingbox':self.boundingbox,'Tright':self.Tright,'featuretype':res.features.type}
                try:
                    self.pe = PointPoseExtractor(type=self.type,points3d=points3d,descriptors=descriptors)
                except detection_error as e:
                    print(e)
                    self.pe=None
                if len(req.savefilename) > 0:
                    pickle.dump(template,open(req.savefilename,'w'))
            except (rospy.service.ServiceException,CvBridgeError) as e:
                print(e)
                return None
        return SetTemplateResponse()

    def featureimagecb(self,featuremsg):
        # if not in GUI mode and no one is subscribing, don't do anything
        if self.cvwindow is None and self.pub_objdet.get_num_connections()==0:
            rospy.logdebug('PointPoseExtraction.py no connections, so ignoring image')
            return
        self.featuremsg=featuremsg
        with self.extractionlck:
            try:
                cv_image = self.bridge.imgmsg_to_cv(featuremsg.image, "bgr8")
            except CvBridgeError as e:
                print(e)
                return
            # decompose P=KK*Tcamera
            P = reshape(array(featuremsg.info.P),(3,4))
            cvKK = cv.fromarray(eye(3))
            cvRcamera = cv.fromarray(eye(3))
            cvTcamera = cv.fromarray(zeros((4,1)))
            cv.DecomposeProjectionMatrix(cv.fromarray(P),cvKK,cvRcamera,cvTcamera)
            KK = array(cvKK)
            Tcamera = eye(4)
            Tcamera[0:3,:] = c_[array(cvRcamera),-dot(array(cvRcamera),array(cvTcamera)[0:3]/cvTcamera[3,0])]
            print("Tcamera: ",Tcamera)
            if self.pe is None:
                if len(self.imagepoints) >= 4:
                    xaxis = self.imagepoints[1]-self.imagepoints[0]
                    yaxis = self.imagepoints[2]-self.imagepoints[1]
                    if xaxis[0]*yaxis[1]-xaxis[1]*yaxis[0] < 0:
                        print('point order is not correct! need to specify points in clockwise order')
                        self.imagepoints=[]
                        return
                    # find the homography, warp the image, and get new features
                    width = int(sqrt(max(sum((self.imagepoints[1]-self.imagepoints[0])**2),sum((self.imagepoints[3]-self.imagepoints[2])**2))))
                    height = int(sqrt(max(sum((self.imagepoints[2]-self.imagepoints[1])**2),sum((self.imagepoints[3]-self.imagepoints[0])**2))))
                    cvimagepoints = cv.fromarray(array(self.imagepoints))
                    cvtexturepoints = cv.fromarray(array(((0,0),(width,0),(width,height),(0,height)),float))
                    cv_texture = cv.CreateMat(height,width,cv_image.type)
                    cv_texture2 = cv.CreateMat(height/2,width/2,cv_image.type)
                    cvH = cv.fromarray(eye(3))
                    cv.FindHomography(cvimagepoints,cvtexturepoints,cvH,0)
                    cv.WarpPerspective(cv_image,cv_texture,cvH)
                    try:
                        res=rospy.ServiceProxy('Feature0DDetect',posedetection_msgs.srv.Feature0DDetect)(image=self.bridge.cv_to_imgmsg(cv_texture))
                        N = len(res.features.positions)/2
                        positions = reshape(res.features.positions,(N,2))
                        descriptors = reshape(res.features.descriptors,(N,res.features.descriptor_dim))
                        texturedims_s = raw_input('creating template '+self.templateppfilename + ' enter the texture dimensions (2 values): ')
                        texturedims=[float(f) for f in texturedims_s.split()]
                        points3d=c_[positions[:,0]*texturedims[0]/width,positions[:,1]*texturedims[1]/height,zeros(len(positions))]
                        self.Itemplate = array(cv_texture)
                        self.Itemplate[:,:,(0,2)] = self.Itemplate[:,:,(2,0)]
                        self.boundingbox=transformPoints(linalg.inv(self.Tright),array(((0,0,0),(texturedims[0],texturedims[1],0))))
                        template = {'type':self.type,'points3d':points3d,'descriptors':descriptors,'Itemplate':self.Itemplate,'boundingbox':self.boundingbox,'Tright':self.Tright,'featuretype':featuremsg.features.type}
                        pickle.dump(template,open(self.templateppfilename,'w'))
                        scipy.misc.pilutil.imshow(self.Itemplate)
                        self.pe = PointPoseExtractor(type=self.type,points3d=points3d,descriptors=descriptors)
                        self.imagepoints = []
                    except rospy.service.ServiceException as e:
                        print(e)
                    return
            else:
                success = False
                projerror=-1.0
                try:
                    if self.featuretype is not None and self.featuretype != featuremsg.features.type:
                        rospy.logwarn('feature types do not match: %s!=%s'%(self.featuretype,featuremsg.features.type))
                        raise detection_error()
                    starttime = time.time()
                    N = len(featuremsg.features.positions)/2
                    positions = reshape(featuremsg.features.positions,(N,2))
                    descriptors = reshape(featuremsg.features.descriptors,(N,featuremsg.features.descriptor_dim))
                    Tlocal,projerror = self.pe.extractpose(KK,positions,descriptors,featuremsg.features.confidences,cv_image.width,verboselevel=self.verboselevel,neighthresh=self.neighthresh,thresh=self.thresh,dminexpected=self.dminexpected,ransaciters=self.ransaciters)
                    success = projerror < self.errorthresh
                    if success: # publish if error is low enough
                        Tlocalobject = dot(Tlocal,self.Tright)
                        Tglobal = dot(linalg.inv(Tcamera),Tlocalobject)
                        poseglobal = poseFromMatrix(Tglobal)
                        pose = posedetection_msgs.msg.Object6DPose()
                        pose.type = self.type
                        pose.pose.orientation.w = poseglobal[0]
                        pose.pose.orientation.x = poseglobal[1]
                        pose.pose.orientation.y = poseglobal[2]
                        pose.pose.orientation.z = poseglobal[3]
                        pose.pose.position.x = poseglobal[4]
                        pose.pose.position.y = poseglobal[5]
                        pose.pose.position.z = poseglobal[6]
                        objdetmsg = posedetection_msgs.msg.ObjectDetection()
                        objdetmsg.objects=[pose]
                        objdetmsg.header = featuremsg.image.header
                        print('local texture: ',repr(Tlocal))
                        self.pub_objdet.publish(objdetmsg)
                        self.drawpart(cv_image,Tlocalobject,KK)
                        self.drawcoordsys(cv_image,Tlocalobject,KK)
                except detection_error as e:
                    pass
                if self.verboselevel > 0:
                    rospy.loginfo('%s: %s, detection time: %fs, projerror %f < %f'%(self.type,'success' if success else 'failure',time.time()-starttime,projerror,self.errorthresh))
            if self.cvwindow is not None:
                if not self.cvwindowstarted:
                    cv.NamedWindow(self.cvwindow, cv.CV_WINDOW_AUTOSIZE)
                    cv.SetMouseCallback(self.cvwindow,self.cvmousecb)
                    self.cvwindowwstarted=True
                for x,y in self.imagepoints:
                    cv.Circle(cv_image,(int(x),int(y)),2,(0,0,255),2)
                cv.ShowImage(self.cvwindow,cv_image)
                char=cv.WaitKey(20)

def CreateTemplateFn(type='',imagefilename='',Tright=eye(4),object_width=100,object_height=100):
    cv_texture = cv.LoadImageM(imagefilename)
    [width,height] = cv.GetSize(cv_texture)

    print('image:name=%s, width=%d hegit=%d, object:width=%f hegith=%f'%(imagefilename,width,height,object_width,object_height))

    res=rospy.ServiceProxy('Feature0DDetect',posedetection_msgs.srv.Feature0DDetect)(image=CvBridge().cv_to_imgmsg(cv_texture))
    N = len(res.features.positions)/2
    positions = reshape(res.features.positions,(N,2))
    descriptors = reshape(res.features.descriptors,(N,res.features.descriptor_dim))

    templateppfilename = os.path.dirname(imagefilename)+'/template_'+os.path.basename(imagefilename)+'.pp'
    texturedims=[object_width,object_height]
    points3d=c_[positions[:,0]*texturedims[0]/width,positions[:,1]*texturedims[1]/height,zeros(len(positions))]
    Itemplate = array(cv_texture)
    Itemplate[:,:,(0,2)] = Itemplate[:,:,(2,0)]
    boundingbox=transformPoints(linalg.inv(Tright),array(((0,0,0),(texturedims[0],texturedims[1],0))))
    template = {'type':type,'points3d':points3d,'descriptors':descriptors,'Itemplate':Itemplate,'boundingbox':boundingbox,'Tright':Tright,'featuretype':res.features.type}
    pickle.dump(template,open(templateppfilename,'w'))
    #scipy.misc.pilutil.imshow(Itemplate)

if __name__=='__main__':
    rospy.init_node('PointPoseExtract')
    parser = OptionParser(description='Connect to ROS and publish 6D poses if the template is discovered in the incoming images. Extracts poses from a set of point features. In order to use with 0D features, first need to create a template model and pickle it.')
    parser.add_option('--hidegui', action="store_false",dest='showgui',default=True,
                      help='Shows the image with the detected object')
    parser.add_option('-q','--quiet',action='store_true', dest='quiet',default=False,
                      help="If set will not print extraction output and times.")
    parser.add_option('--template', action='store', type='string', dest='template',default=None,
                      help='Python pickle file for description of template to detect. Requires a template variable with fieldnames type,points3d,Itemplate,descriptors')
    parser.add_option('--errorthresh',action='store', type='float', dest='errorthresh',default=0.004,
                      help='Threshold on the error of the pose to allow for publishing')
    parser.add_option('--neighthresh',action='store', type='float', dest='neighthresh',default=0.8,
                      help='Threshold on the descriptor vectors')
    parser.add_option('--thresh',action='store', type='float', dest='thresh',default=None,
                      help='The reprojection threshold when choosing inliers in ransac')
    parser.add_option('--dminexpected',action='store', type='int', dest='dminexpected',default=10,
                      help='Minimum number of features to match before outputting a pose')
    parser.add_option('--ransaciters',action='store', type='int', dest='ransaciters',default=200,
                      help='The number of iterations of ransac to perform')
    parser.add_option('--transform',action='store', type='string', dest='transform',default=None,
                      help="3x4 transformation (row-order row-major) of the template with respect to the object, used only for template creation")
    parser.add_option('--type', action='store', type='string', dest='type',default='',
                      help='The object type (filename for geometry), used only for template creation')
    parser.add_option('--imagefilename',
                      action="store",type='string',dest='imagefilename',default='',
                      help='Image file to create description of template')
    parser.add_option('--object-width',
                      action="store",type='float',dest='object_width',default=0.100,
                      help='Width of real object, used for creating template')
    parser.add_option('--object-height',
                      action="store",type='float',dest='object_height',default=0.100,
                      help='Height of real object, used for creating template')
    (options, args) = parser.parse_args()
    T = eye(4)
    if options.transform:
        T[0:3,0:4] = reshape([float(s) for s in options.transform.split()],(3,4))
    if options.imagefilename:
        CreateTemplateFn(type=options.type,imagefilename=options.imagefilename,Tright=T,object_width=options.object_width,object_height=options.object_height)
        sys.exit(0)
    try:
        processor = ROSPlanarPoseProcessor(templateppfilename=options.template,errorthresh=options.errorthresh,neighthresh=options.neighthresh,thresh=options.thresh,Tright=T,dminexpected=options.dminexpected,ransaciters=options.ransaciters,showgui=options.showgui,verboselevel=0 if options.quiet else 1,type=options.type)
        rospy.spin()
    except KeyboardInterrupt as e:
        pass

def test():
    import PointPoseExtraction
    #self = PointPoseExtraction.ROSPlanarPoseProcessor(errorthresh=0.01,showgui=True)
    self = PointPoseExtraction.ROSPlanarPoseProcessor(showgui=True,dminexpected=20,neighthresh=0.85,errorthresh=0.005,templateppfilename='template_peachjuice0.pp',type='test')#scenes/cereal_frootloops.kinbody.xml')

    featuremsg=self.featuremsg
    cv_image = self.bridge.imgmsg_to_cv(featuremsg.image, "bgr8")
    newimage=cv.CloneMat(cv_image)
    ret=cv.PyrMeanShiftFiltering(cv_image,newimage,5,30)
