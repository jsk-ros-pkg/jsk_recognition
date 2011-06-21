% msg = posedetectiondb_feature0dwrapper(funcname,rosargs)

% Copyright (C) 2009-2010 Rosen Diankov
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
function msg = posedetectiondb_feature0dwrapper(func,rosargs)
rosoct('argv',rosargs,'shutdown');

if( ~rosoct_advertise_service('Feature0DDetector',@posedetectiondb_Feature0DDetector, @(req) featurewrapper(req,func)) )
    error('failed to advertise Feature0DDetector service');
end

while(1)
    rosoct_worker(1);
    pause(0.05);
end

function res = featurewrapper(req,func)
[frames, descriptors, confidences] = func(sensor_msgs_processImage(req.image));
res = req.create_response_();

N = size(frames,2);
res.positions = frames(1:2,:)-1; # octave is 1-indexed
if( size(frames,1) > 2 )
    res.orientations = frames(3,:);
end
if( size(frames,1) > 3 )
    frames(4,:)
    res.scales = frames(4,:);
end

res.confidences = confidences(:);
res.descriptors = descriptors(:);
res.descriptor_dim = size(descriptors,1);
