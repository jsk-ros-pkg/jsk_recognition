% siftPoseExtraction_ros(texturefilename,Ttexture,objdettopic,imagetopic,caminfotopic,show)
%
%% Detects the poses of the template from the incoming ROS images and publishes this information.
%% Extracts sift features directly in this process

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
function siftPoseExtraction_ros(texturefilename,Ttexture,objdettopic,imagetopic,caminfotopic,show)
global g_KK g_doquit

if( ~exist('show','var') )
    show = 0;
end

if( ~exist('objdettopic','var') || isempty(objdettopic) )
    objdettopic = '/checkerdetector/ObjectDetection';
end
if( ~exist('imagetopic','var') || isempty(imagetopic) )
    imagetopic = '/camera/Image';
end
if( ~exist('caminfotopic','var') || isempty(caminfotopic) )
    caminfotopic = '/camera/CameraInfo';
end

template = siftBuildTemplate(texturefilename,Ttexture);
g_KK = [];
g_doquit = 0;

success = rosoct_advertise(objdettopic,@checkerboard_detector_ObjectDetection,10);
if( ~success )
    error('advertise failed');
end

rosoct_unsubscribe(imagetopic);
success = rosoct_subscribe(imagetopic, @sensor_msgs_Image, @(imagemsg) imagecb(imagemsg,template,objdettopic,show),1);
if( ~success )
    error('subscribe failed');
end

rosoct_unsubscribe(caminfotopic);
success = rosoct_subscribe(caminfotopic, @sensor_msgs_CameraInfo, @caminfocb,1);
if( ~success )
    error('subscribe failed');
end

while(g_doquit == 0)
    rosoct_worker(4);
end

disp('unsubscribing');
rosoct_unsubscribe(imagetopic);
rosoct_unsubscribe(caminfotopic);
rosoct_unadvertise(objdettopic);

function caminfocb(msg)
global g_KK
g_KK = transpose(reshape(msg.K,[3,3]));

function imagecb(imagemsg,template,objdettopic,show)
global g_KK
if( isempty(g_KK) )
    return;
end

I = sensor_msgs_processImage(imagemsg);
[frames,desc,conf] = feature0D_siftfast(I,0);
[T,projerror] = pointPlanarPoseExtraction(template,frames,desc,conf,g_KK,show,I);

if( ~isempty(T) )
    disp(sprintf('found with error %f',projerror));
    msg = checkerboard_detector_ObjectDetection();
    msg.objects = {checkerboard_detector_Object6DPose()};
    msg.objects{1}.type = template.type;
    msg.objects{1}.uid = 1;

    q = QuatFromRotationMatrix(T(1:3,1:3));
    pose = robot_msgs_Pose();
    pose.orientation.w = q(1);
    pose.orientation.x = q(2);
    pose.orientation.y = q(3);
    pose.orientation.z = q(4);
    pose.position.x = T(1,4);
    pose.position.y = T(2,4);
    pose.position.z = T(3,4);
    msg.objects{1}.pose = pose;

    rosoct_publish(objdettopic,msg);
end
