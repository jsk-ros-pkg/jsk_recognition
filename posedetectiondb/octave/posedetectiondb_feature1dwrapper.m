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
function msg = posedetectiondb_feature1dwrapper(func,rosargs)
rosoct('argv',rosargs,'shutdown');

if( ~rosoct_advertise_service('Feature1DDetector',@posedetectiondb_Feature1DDetector, @(req) featurewrapper(req,func)) )
    error('failed to advertise Feature1DDetector service');
end

while(1)
    rosoct_worker(1);
    pause(0.05);
end

function res = featurewrapper(req,func)
[lines, descriptors, confidences] = func(sensor_msgs_processImage(req.image));
res = req.create_response_();

res.confidences = confidences(:);
res.lines = cell(length(lines),1);
for i = 1:length(lines)
    res.lines{i} = posedetectiondb_Curve1D();
    res.lines{i}.pts = lines{i}-1;  # octave is 1-indexed
end

res.descriptors = descriptors(:);
res.descriptor_dim = size(descriptors,1);
