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
[status,rosoctpath] = system(['rospack find rosoct']);
rosoctpath = strtrim(rosoctpath);
addpath(fullfile(rosoctpath, 'octave'));
rosoct_add_msgs('sensor_msgs');
addpath(fullfile(rosoct_findpackage('sensor_msgs'),'octave'));
rosoct_add_msgs('std_msgs');
rosoct_add_msgs('or_msgs');
rosoct_add_msgs('openraveros');
rosoct_add_msgs('posedetectiondb');
rosoct_add_srvs('posedetectiondb');
addpath(fullfile(rosoct_findpackage('posedetectiondb'),'octave'));
rosoct_add_msgs('checkerboard_detector');

if( isoctave() )
    addpath(fullfile(rosoct_findpackage('libsiftfast'),'share','siftfast','octave'));
else
    addpath(fullfile(rosoct_findpackage('libsiftfast'),'share','siftfast','matlab'));
end
