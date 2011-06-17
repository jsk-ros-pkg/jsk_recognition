% [I,T,KK] = gatherImageAndTransform(objdettopic,imagetopic,caminfotopic)
%

% Copyright (C) 2009 Rosen Diankov
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
function [I,T,KK] = gatherImageAndTransform(objdettopic,imagetopic,caminfotopic)
global g_imagemsg g_objtransform g_KK

if( ~exist('objdettopic','var') || isempty(objdettopic) )
    objdettopic = '/checkerdetector/ObjectDetection';
end
if( ~exist('imagetopic','var') || isempty(imagetopic) )
    imagetopic = '/camera/Image';
end
if( ~exist('caminfotopic','var') || isempty(caminfotopic) )
    caminfotopic = '/camera/CameraInfo';
end

g_imagemsg = [];
g_objtransform = [];
g_KK = [];

queuesize = 1;
rosoct_unsubscribe(objdettopic);
success = rosoct_subscribe(objdettopic, @checkerboard_detector_ObjectDetection, @objdetcb,queuesize);
if( ~success )
    error('subscribe failed');
end

rosoct_unsubscribe(imagetopic);
success = rosoct_subscribe(imagetopic, @sensor_msgs_Image, @imagecb,queuesize);
if( ~success )
    error('subscribe failed');
end

rosoct_unsubscribe(caminfotopic);
success = rosoct_subscribe(caminfotopic, @sensor_msgs_CameraInfo, @caminfocb,queuesize);
if( ~success )
    error('subscribe failed');
end

while(isempty(g_imagemsg) || isempty(g_objtransform) || isempty(g_KK))
    rosoct_worker(1);
end

disp('unsubscribing');
rosoct_unsubscribe(imagetopic);
rosoct_unsubscribe(objdettopic);
rosoct_unsubscribe(caminfotopic);

I = g_imagemsg;
T = g_objtransform;
KK = g_KK;

function imagecb(msg)
global g_imagemsg
g_imagemsg = sensor_msgs_processImage(msg);

function caminfocb(msg)
global g_KK
g_KK = transpose(reshape(msg.K,[3,3]));

function objdetcb(msg)
global g_objtransform

if( length(msg.objects) > 0 )
    pose  = msg.objects{1}.pose;
    R = RotationMatrixFromQuat([pose.orientation.w pose.orientation.x pose.orientation.y pose.orientation.z]);
    g_objtransform = [R [pose.position.x;pose.position.y;pose.position.z]; 0 0 0 1];
end
