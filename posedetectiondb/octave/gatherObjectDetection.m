% T = gatherObjectDetection(objdettopic)
%
% returns the a 4x4 transformation of the first detected pose from objdettopic

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
function T = gatherObjectDetection(objdettopic)
global g_T

rosoct_add_msgs('checkerboard_detector');

if( ~exist('objdettopic','var') || isempty(objdettopic) )
    objdettopic = '/ObjectDetection';
end
g_T = [];

rosoct_unsubscribe(objdettopic);
success = rosoct_subscribe(objdettopic, @checkerboard_detector_ObjectDetection, @objdetcb,1);
if( ~success )
    error('subscribe failed');
end

while(isempty(g_T))
    rosoct_worker(1);
end

disp('unsubscribing');
rosoct_unsubscribe(objdettopic);

T = g_T;

function objdetcb(msg)
global g_T

if( length(msg.objects) > 0 )
    pose  = msg.objects{1}.pose;
    R = RotationMatrixFromQuat([pose.orientation.w pose.orientation.x pose.orientation.y pose.orientation.z]);
    g_T = [R [pose.position.x;pose.position.y;pose.position.z]; 0 0 0 1];
end
