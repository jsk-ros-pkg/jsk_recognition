% siftImagePoseExtraction_ros(texturefilename,Ttexture,objdettopic,sifttopic,show)
%
% Detects the poses of the template from the incoming ROS SIFT data and publishes this information.

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
function siftImagePoseExtraction_ros(texturefilename,Ttexture,objdettopic,sifttopic,show)
global g_doquit

if( ~exist('show','var') )
    show = 0;
end

if( ~exist('objdettopic','var') || isempty(objdettopic) )
    objdettopic = '/ObjectDetection';
end
if( ~exist('sifttopic','var') || isempty(sifttopic) )
    sifttopic = '/ImageSift';
end

template = siftBuildTemplate(texturefilename,Ttexture);
g_doquit = 0;

rosoct_unadvertise(objdettopic);
success = rosoct_advertise(objdettopic,@checkerboard_detector_ObjectDetection,10);
if( ~success )
    error('advertise failed');
end

rosoct_unsubscribe(sifttopic);
success = rosoct_subscribe(sifttopic, @or_msgs_ImageSift, @(imagemsg) imagecb(imagemsg,template,objdettopic,show),1);
if( ~success )
    error('subscribe failed');
end

while(g_doquit == 0)
    rosoct_worker(4);
end

disp('unsubscribing');
rosoct_unsubscribe(sifttopic);
rosoct_unadvertise(objdettopic);

function imagecb(siftmsg,template,objdettopic,show)
global g_siftmsg g_doquit
channels = 1;
if( strcmp(siftmsg.cimage.image.colorspace,'rgb24') )
    channels = 3;
end

if( ~strcmp(siftmsg.cimage.image.compression,'raw') )
    disp('only accept uncompressed images');
    return;
end

if( length(siftmsg.cimage.image.data) ~= channels*siftmsg.cimage.image.width*siftmsg.cimage.image.height )
    disp('image is wrong dimensions');
    return;
end

I = im2double(permute(reshape(siftmsg.cimage.image.data,[channels siftmsg.cimage.image.width siftmsg.cimage.image.height]),[3 2 1]));
I = I(:,:,[3 2 1]); % flip r and b
KK = transpose(reshape(siftmsg.cimage.KK,[3 3]));

N = length(siftmsg.sift.frames)/4;
frames = reshape(siftmsg.sift.frames,[4 N]);
desc = reshape(siftmsg.sift.desc,[siftmsg.sift.descdimension N]);
conf = ones(1,N);

[T,projerror] = pointPlanarPoseExtraction(template,frames,desc,conf,KK,show,I);
T
rosoct('clear')
if( ~isempty(T) )
    disp(sprintf('found with error %f',projerror));
    if( projerror < 0.004 )
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
else
    if( show )
        imshow(I*64);
        drawnow;
    end
end
