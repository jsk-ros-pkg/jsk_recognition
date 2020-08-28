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
NAME = 'GatherDetectionResults'

import roslib; roslib.load_manifest(PKG) 
import os, sys, time, string, threading
from optparse import OptionParser
import numpy # nice to be able to explicitly call some functions
from numpy import *

import rospy
import posedetection_msgs.msg

from openravepy import *

class VisibilityModel(metaclass.AutoReloader):
    def __init__(self,measurements=None,filename=None,symmetricplane=None,kinbodyfile=None):
        if measurements is not None:
            self.rawmeasurements = measurements
            
        if symmetricplane is not None:
            dists = dot(symmetricplane[0:3],transpose(self.rawmeasurements))+symmetricplane[3]
            self.rawmeasurements = r_[self.rawmeasurements,self.rawmeasurements-dot(reshape(2.0*dists,(dists.shape[0],1)),reshape(symmetricplane[0:3],(1,3)))]

        self.trimesh = None
        if kinbodyfile is not None:
            self.env = Environment()
            self.orobj = self.env.ReadKinBodyXMLFile(kinbodyfile)
            if self.orobj is None:
                raise ValueError('failed to open %s openrave file'%kinbodyfile)
            self.env.AddKinBody(self.orobj)
            self.trimesh = self.env.Triangulate(self.orobj)
        self.measurements = self.rawmeasurements
    def CreateReducedModel(self,bandwidth=0.04,bandthresh=0.01,neighthresh=0.01,showdata=False,savefile=None):
        self.measurements,indices = self.Prune(self.rawmeasurements,100,neighthresh**2,1)
        uniformpoints,dists,pointscale = self.UniformlySampleSpace(bandwidth,bandthresh)
        if showdata:
            from enthought.mayavi import mlab
            mlab.figure(1,fgcolor=(0,0,0), bgcolor=(1,1,1))
            src = mlab.pipeline.scalar_field(dists)
            mlab.pipeline.iso_surface(src,contours=[0.01],opacity=0.1)
            mlab.pipeline.volume(mlab.pipeline.scalar_field(dists*500))
            v = pointscale[0]*self.trimesh.vertices+pointscale[1]
            mlab.triangular_mesh(v[:,0],v[:,1],v[:,2],self.trimesh.indices,color=(0,0,0.5))
        if savefile is None:
            savefile = self.getfilename()
        print('saving measurements to %s'%savefile)
        mkdir_recursive(os.path.split(savefile)[0])
        savetxt(savefile,uniformpoints,'%f')
    def getfilename(self):
        return os.path.join(self.env.GetHomeDirectory(),'kinbody.'+self.orobj.GetKinematicsGeometryHash(),'visibility.txt')

    def UniformlySampleSpace(self,bandwidth,bandthresh,delta=0.02):
        maxradius = sqrt(max(sum(self.measurements**2,1)))
        nsteps = floor(maxradius/delta)
        X,Y,Z = mgrid[-nsteps:nsteps,-nsteps:nsteps,-nsteps:nsteps]
        allpoints = c_[X.flat,Y.flat,Z.flat]*delta/bandwidth
        sampleinds = flatnonzero(sum(allpoints**2,1)<(maxradius/bandwidth)**2)
        samplepoints = allpoints[sampleinds,:]
        kdtree = pyANN.KDTree(self.measurements/bandwidth)
        sampledists = zeros(samplepoints.shape[0])
        goodpoints = []
        for i in xrange(samplepoints.shape[0]):
            neighs,dists,kball = kdtree.kFRSearchArray(samplepoints[i:(i+1),:],5.0**2,32,0.0001)
            sampledists[i] = sum(exp(-dists[neighs>=0]))
        uniformpoints = samplepoints[sampledists>bandthresh,:]*bandwidth
        alldists = zeros(prod(X.shape))
        alldists[sampleinds] = sampledists
        return uniformpoints,reshape(alldists,X.shape),array((1.0/delta,nsteps))

    def Prune(self,rawposes, nsize, thresh2, neighsize,giveupiters=100):
        """rawposes is Nx7"""
        iter = 1
        poses = array(rawposes)
        indices = range(poses.shape[0])
        N = poses.shape[0]
        nochange=0
        while N > nsize:
            ind = numpy.random.randint(N)
            g = poses[ind,:]
            # check g's neighbors
            d = sum((poses[0:N,:] - tile(g, (N,1)))**2,1)
            neigh = sum(d < thresh2)
            if neigh > neighsize:
                # move to the last pose and resize
                poses[ind,:] = poses[N-1,:]
                indices[ind] = indices[N-1]
                nochange=0
                N -= 1
            nochange += 1
            iter += 1
            if iter > 5000 or nochange > giveupiters:
                break
        return poses[0:N,:],indices[0:N]

class OpenRAVEVisualizer(metaclass.AutoReloader):
    def __init__(self,kinbodyfile,automaticadd=True,measurementsfilename=None):
        self.automaticadd = automaticadd
        self.orenv = Environment()
        self.orenv.SetViewer('qtcoin')
        self.orobj = self.orenv.ReadKinBodyXMLFile(kinbodyfile)
        if self.orobj is None:
            raise ValueError('failed to open %s openrave file'%kinbodyfile)
        self.orenv.AddKinBody(self.orobj)
        self.objab = self.orobj.ComputeAABB()
        self.Tcamera = None
        self.lck = threading.Lock()
        self.camerahandle = None
        self.measurementhandles = []
        self.measurements = []
        if measurementsfilename is not None:
            f = open(measurementsfilename,'r')
            for l in f.readlines():
                m = array([string.atof(s) for s in string.split(l)])
                self.drawmeasurement(m)
                self.measurements.append(m)
            f.close()

        self.sub_objdet = rospy.Subscriber("ObjectDetection", posedetection_msgs.msg.ObjectDetection,self.objdetcb, queue_size=1)
        rospy.init_node(NAME, anonymous=True)#,disable_signals=False)

    def __del__(self):
        self.sub_objdet.unregister()
        self.orenv.Destroy()

    def objdetcb(self,msg):
        newcamerahandle = None
        if len(msg.objects) > 0:
            q = msg.objects[0].pose.orientation
            t = msg.objects[0].pose.position
            with self.lck:
                self.Tcamera = linalg.inv(matrixFromPose([q.w,q.x,q.y,q.z,t.x,t.y,t.z]))

            # draw in openrave environment
            sx = 0.02
            sy = 0.02
            camlocalpts = array(((sx,sy,0.05),(-sx,sy,0.05),(-sx,-sy,0.05),(sx,-sy,0.05),(sx,sy,0.05),(0,0,0),(-sx,sy,0.05),(0,0,0),(-sx,-sy,0.05),(0,0,0),(sx,-sy,0.05)))
            campts = dot(self.Tcamera,r_[transpose(camlocalpts),ones((1,camlocalpts.shape[0]))])
            self.camerahandle = self.orenv.drawlinestrip(transpose(campts[0:3,:]),2,array([0,0,1]))
        if self.automaticadd:
            self.addmeasurement()

    def addmeasurement(self):
        self.lck.acquire()
        Tcamera = self.Tcamera
        self.lck.release()
        dist = dot(Tcamera[0:3,2],self.objab.pos()-Tcamera[0:3,3:4])
        m = Tcamera[0:3,2]*dist
        self.drawmeasurement(m)
        self.measurements.append(m)
        print('num measurements %d'%len(self.measurements))

    def drawmeasurement(self,m):
        dist = sqrt(sum(m**2))
        dir = m/dist
        p = self.objab.pos()-dist*dir
        self.measurementhandles.append(self.orenv.drawlinestrip(transpose(c_[p-0.02*dir,p+0.02*dir]),4,array((1,min(1,dist/1.0),0))))

    def savemeasurements(self,filename):
        self.lck.acquire()
        print('saving measurements to %s'%filename)
        savetxt(filename,self.measurements,'%f')
        self.lck.release()

if __name__=='__main__':
    parser = OptionParser(description='Gather object detection transformations and filter and display them.')
    parser.add_option('--kinbodyfile',
                      action="store",type='string',dest='kinbodyfile',
                      help='OpenRAVE object file that represents the incoming object pose. Updates are show in the openrave window')
    parser.add_option('-s','--single',
                      action="store_true",dest='single',default=False,
                      help='If set, will wait for user input in order to add a measurement')
    parser.add_option('-f','--savefile',
                      action="store",dest="filename",
                      help='If specified, will save all recorded measurements to this file at exit time')
    parser.add_option('-m','--measurements',
                      action="store",dest="measurements",default=None,
                      help='If specified, will start with the current measurements file')
    (options, args) = parser.parse_args()
    if not options.kinbodyfile:
        print('Error: Need to specify an openrave kinbody file')
        sys.exit(1)

    visualizer = OpenRAVEVisualizer(options.kinbodyfile,measurementsfilename=options.measurements,automaticadd=not options.single)
    while True:
        cmd = raw_input('Enter command (q-quit and save,c-capture): ');
        if cmd == 'q':
            break
        elif cmd == 'c' and options.single:
            print('adding measurement')
            visualizer.addmeasurement()
        else:
            print('bad command',cmd)
    if options.filename:
        visualizer.savemeasurements(options.filename)

def test():
    "rosrun posedetectiondb GatherDetectionResults.py --kinbodyfile=scenes/cereal_frootloops.kinbody.xml -f test.txt ObjectDetection:=/CerealDetection"
    import GatherDetectionResults
    self = GatherDetectionResults.VisibilityModel(measurements=loadtxt('uroncha_raw.txt'),symmetricplane=array([1.0,0,0,0]),kinbodyfile='scenes/uroncha.kinbody.xml')
    self = GatherDetectionResults.VisibilityModel(measurements=loadtxt('peachjuice_raw.txt'),kinbodyfile='scenes/peachjuice.kinbody.xml')
    self.CreateReducedModel(bandwidth=0.03,bandthresh=0.02)
