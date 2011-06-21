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
import roslib; roslib.load_manifest(PKG) 
import os, sys, time, threading, string, struct
from optparse import OptionParser
from numpy import *
import scipy
import scipy.signal
import scipy.ndimage
import numpy

#import cv
#from cv_bridge import CvBridge, CvBridgeError

from openravepy import *
from openravepy import pyANN

class ObjectProjection(metaclass.AutoReloader):
    """Offers several functions for querying projection results of objects whose geometry is loaded into openrave"""
    def __init__(self, kinbodydata=None,kinbodyfilename=None):
        self.orenv = Environment()
        orcol = self.orenv.CreateCollisionChecker('ode')
        if orcol is not None:
            self.orenv.SetCollisionChecker(orcol)
        #self.orenv.SetViewer('qtcoin',False)
        if kinbodydata is not None:
            self.orobj = self.orenv.ReadKinBodyXMLData(kinbodydata)
        elif kinbodyfilename is not None:
            self.orobj = self.orenv.ReadKinBodyXMLFile(kinbodyfilename)
        self.orenv.AddKinBody(self.orobj)
        self.orobj.SetTransform(eye(4))
        self.ab = self.orobj.ComputeAABB()
        self.maxradius = sqrt(sum(self.ab.extents()**2))
        self.vertices = transpose(self.orenv.Triangulate(self.orobj).vertices)

    def __del__(self):
        self.orenv.Destroy()

    def Compute3DPositionImage(self,Tcamera, KK, imagewidth,imageheight,ROI=None,buffer=10,computedepth=True):
        """Given the camera position, its intrisnic matrix, and its region of interest [upperleft_xy lowerright_xy],
        compute the position and depth maps. The position map gives the local 3d surface position of every image point.
        if computedepth is True, second returned parameter will be depthmap, otherwise it is the mask.
        """
        projpts = self.ProjectPoints(self.vertices,linalg.inv(Tcamera),KK)
        if ROI is None:
            bbmin = numpy.min(projpts,1)
            bbmax = numpy.max(projpts,1)
            ROI = [max(0,floor(bbmin[0]-buffer)),max(0,floor(bbmin[1]-buffer)),min(imagewidth,floor(bbmax[0]+2*buffer)),min(imageheight,floor(bbmax[1]+2*buffer))]
        ROI = [max(0,ROI[0]), max(0,ROI[1]), min(imagewidth,ROI[2]), min(imageheight,ROI[3])]
        width = int(ROI[2]-ROI[0])
        height = int(ROI[3]-ROI[1])
        offset = [int(ROI[0]),int(ROI[1])]
        if width <= 0 or height <= 0:
            return array(()),array(()),KK,offset
        newKK = array(KK)
        newKK[0,2] -= offset[0]
        newKK[1,2] -= offset[1]
        inds = array(range(width*height))
        imagepoints = array((mod(inds,width),floor(inds/width)))
        camerapoints = transpose(dot(linalg.inv(newKK), r_[imagepoints,ones((1,imagepoints.shape[1]))]))
        hitindices,hitpositions = self.Get3DPointsFromImageRays(camerapoints, Tcamera)
        infinds = flatnonzero(1-hitindices)
        hitpositions[infinds,:] = inf
        # create an image of positions
        if computedepth:
            Idepth = dot(hitpositions,Tcamera[0:3,2])-dot(Tcamera[0:3,2],Tcamera[0:3,3])
            Idepth = reshape(Idepth,(height,width))
        else:
            Idepth = reshape(isinf(hitpositions[:,0])==False,(height,width))
        Iposition = reshape(hitpositions,(height,width,3))
        return Iposition,Idepth,newKK,offset

    def Get3DPointsFromImageRays(self,*args,**kwargs):
        hitindices,hitpositions = self.GetContactPointsFromImageRays(*args,**kwargs)
        return hitindices,hitpositions[:,0:3]

    def GetContactPointsFromImageRays(self,camerapoints, Tcamera,rayneardist=0):
        """Retrives the positions on the object of points in camera space.
        camerapoints is Nx3 array
        Returns hitindices (N points) ,hitpositions (Nx3 vector)"""
        if camerapoints.size == 0:
            return array(()),array(())
        maxdist = fabs(dot(Tcamera[0:3,2],Tcamera[0:3,3]))+self.maxradius+0.2
        raydirs = dot(camerapoints / tile(vstack(sqrt(sum(camerapoints**2,1))),(1,3)),transpose(Tcamera[0:3,0:3]))
        rays = c_[tile(Tcamera[0:3,3],(raydirs.shape[0],1))+rayneardist*raydirs,raydirs*maxdist]
        with self.orenv:
            indices,info = self.orenv.CheckCollisionRays(rays,self.orobj)
            return indices,info

    @staticmethod
    def ProjectPoints(pts,T,KK):
        pts3d = dot(T[0:3,0:3],pts)+tile(T[0:3,3:4],(1,pts.shape[1]))
        return dot(KK[0:2,0:3],pts3d)/tile(pts3d[2:3,:],(2,1))
    @staticmethod
    def ComputeRigidTransform(points1, points2,doscale=False):
        """Finds an affine k+1xk+1 transformation T such that T*transpose(points1)=transpose(points2). points1, points2 is a Nxk array"""
        assert(points1.shape == points2.shape)
        N = points1.shape[0]
        points1mean = mean(points1,0)
        points2mean = mean(points2,0)
        [U,s,Vh] = svd( dot(transpose(points1-tile(points1mean,(N,1))),points2-tile(points2mean,(N,1))) )
        I = eye(points1.shape[1])
        I[-1,-1] = det(dot(U,Vh))
        R = transpose(dot(dot(U,I),Vh))
        if doscale:
            R = dot(R, diag(s))
        T = eye(points1.shape[1]+1)
        T[0:-1,0:-1] = R
        T[0:-1,-1] = points2mean-dot(R,points1mean)
        return T
    @staticmethod
    def CameraQuaternionDistSqr(q,qarray,angleweight=0.0):
        """ distance between two quaternions ignoring left rotation around z axis of qarray"""
        sinang = -q[0]*qarray[:,3]-q[1]*qarray[:,2]+q[2]*qarray[:,1]+q[3]*qarray[:,0]
        cosang = q[0]*qarray[:,0]+q[1]*qarray[:,1]+q[2]*qarray[:,2]+q[3]*qarray[:,3]
        length = 1.0/sqrt(sinang**2+cosang**2)
        angle = atan2(sinang,cosange)
        sinang *= length
        cosang *= length
        return quatArrayTDist(q,c_[cosang*qarray[:,0]-sinang*qarray[:,3], cosang*qarray[:,1]-sinang*qarray[:,2], cosang*qarray[:,2]-sinang*qarray[:,1], cosang*qarray[:,3]-sinang*qarray[:,0]])**2+angleweight*angle**2
    @staticmethod
    def CameraPoseDistSqr(pose,posearray,rotweightsqr=0.3,angleweight=0.0):
        """distance between two poses ignoring left rotation around zaxis of posearray. Squared quaternion distance is scaled by rotweightsqr"""
        return sum((tile(pose[4:7],(len(posearray),1))-posearray[:,4:7])**2,1)+rotweightsqr*ObjectProjection.CameraQuaternionDistSqr(pose[0:4],posearray[:,0:4],angleweight=angleweight)
    @staticmethod
    def ComputeNormalizationTransformation(points):
        """Computes a k+1xk+1 affine transformation to normalize a Nxk points array"""
        pointsmean = mean(points, 0)
        pointsnorm = points - tile(pointsmean, (points.shape[0],1))
        pointssigma = dot(transpose(pointsnorm), pointsnorm) / points.shape[0]
        [U, s, Vh] = svd(pointssigma)
        T = eye(points.shape[1]+1)
        T[0:-1,0:-1] = transpose(U)
        T[0:-1,-1] = -dot(transpose(U),pointsmean)
        return T
    @staticmethod
    def Compute3DPositionFromDepth(Idepth,Tcamera,KK):
        depthr,depthc = nonzero(isfinite(Idepth))
        depth = Idepth[depthr,depthc]
        camerapos = dot(c_[depthc*depth,depthr*depth,depth],transpose(linalg.inv(KK)))
        Iposition = tile(inf,(Idepth.shape[0],Idepth.shape[1],3))
        # transform by Tcamera
        pos = dot(camerapos,transpose(Tcamera[0:3,0:3]))+tile(Tcamera[0:3,3],(camerapos.shape[0],1))
        Iposition[depthr,depthc,:] = dot(camerapos,transpose(Tcamera[0:3,0:3]))+tile(Tcamera[0:3,3],(camerapos.shape[0],1))
        return Iposition

    @staticmethod
    def FillDepthMap(Idepth,dilateradius=8):
        Imask = isfinite(Idepth)
        Idepthfilled = array(Idepth)
        if dilateradius > 0:
            Ieroded = scipy.signal.convolve2d(Imask,ObjectProjection.streldisk(dilateradius),'same')==0
        else:
            Ieroded = Imask==False;
        
        L,numlabels = scipy.ndimage.label(Ieroded,[[1,1,1], [1,1,1], [1,1,1]])
        props = ObjectProjection.ImageRegionProps(L)
        if len(props) == 0:
            return Idepthfilled

        for prop in props:
            if prop['area'] < 5000:
                Ieroded[L==prop['id']] = False
        Imaskfilled = Ieroded==False
        
        # fill the depth map
        fillr,fillc = nonzero(logical_and(Imask==False,Imaskfilled))
        if len(fillr) > 0:
            # find the 8 nearest neighbors of each point, fit a plane through them,
            # and see where the query point lies on that z of that plane
            maskr,maskc = nonzero(Imask)
            maskpoints = c_[maskr,maskc]
            if len(maskpoints) > 0:
                maskdepth = Idepth[maskr,maskc]
                kdtree = pyANN.KDTree(maskpoints)
                neighs,dists = kdtree.kSearchArray(c_[fillr,fillc],8,0.1)
                for i,n in enumerate(neighs):
                    A = c_[maskpoints[n,:],maskdepth[n]]
                    mA = A-tile(mean(A,0),(A.shape[0],1))
                    u,s,v = linalg.svd(dot(transpose(mA),mA))
                    # compute the plane and evaluate at the middle point
                    planedist = -mean(dot(A,u[:,2:3]))
                    Idepthfilled[fillr[i],fillc[i]] = -(u[0,2]*fillr[i]+u[1,2]*fillc[i]+planedist)/u[2,2]
        return Idepthfilled

    @staticmethod
    def ImageRegionProps(L):
        """calculates region properties of a labeled image similar to MATLAB's regionprops
        Output:
          props - structure where each element contains
          Area - number of pixels in region
          BoundingBox - [startrow startcol numrows numcols]
          """
        ids = unique(L).tolist()
        if 0 in ids:
            ids.remove(0)
        props = []
        for id in ids:
            prop = dict()
            r,c = nonzero(L==id)
            prop['id'] = id
            prop['area'] = len(r)
            ul = [numpy.min(c),numpy.min(r)]
            prop['boundingbox'] = ul+[numpy.max(c)-ul[0],numpy.max(r)-ul[1]]
            props.append(prop)
        return props

    @staticmethod
    def streldisk(r):
        if r == 1:
            return array([[0,1,0],[1,1,1],[0,1,0]])
        elif r == 2:
            return array([[0,0,1,0,0],[0,1,1,1,0],[1,1,1,1,1],[0,1,1,1,0],[0,0,1,0,0]])
        numempty = 2+4*int((r-4)/7)+2*int(mod(r-4,7)/4) #don't ask.. this is the function matlab uses
        A = ones((2*r-1, 2*r-1))
        for i in range(numempty):
            A[i,0:(numempty-i)] = 0
            A[-i-1,0:(numempty-i)] = 0
            A[i,(-numempty+i):] = 0
            A[-i-1,(-numempty+i):] = 0
        return A

    @staticmethod
    def imshow(I):
        If = array(I,'float')
        a = numpy.min([m for m in If.flat if isfinite(m)])
        b = numpy.max([m for m in If.flat if isfinite(m)])
        if fabs(a-b)<1e-5:
            a=0
        scipy.misc.pilutil.imshow(array((If-a)/(b-a)*255,'uint8'))

def test():
    import ObjectProjection
    I = scipy.misc.pilutil.imread('image001.png')
    self = ObjectProjection.ObjectProjection(kinbodyfilename='brkt_hvac.kinbody.xml')
    Tcamera = linalg.inv(array(((-0.2276663,0.0930725,0.9692811,-0.1170402),
                     (-0.4038756,-0.9147867,-0.0070232,-0.0036750),
                     (0.8860320, -0.3930678, 0.2458557,1.8410743),
                     (0,0,0,1))))
    KK = array(((3.4724e+03,0.0000e+00,5.8521e+02),
                (0.0000e+00,3.4861e+03,3.8965e+02),
                (0.0000e+00,0.0000e+00,1.0000e+00)))
    imagewidth = 1024
    imageheight = 768
    Iposition,Idepth,newKK,offset = self.Compute3DPositionImage(Tcamera=Tcamera,KK=KK,imagewidth=imagewidth,imageheight=imageheight)
    ObjectProjection.ObjectProjection.imshow(Iposition)
    ObjectProjection.ObjectProjection.imshow(Idepth)
