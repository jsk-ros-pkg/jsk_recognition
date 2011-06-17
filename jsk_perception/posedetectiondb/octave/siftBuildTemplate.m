% template = siftBuildTemplate(filename)

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
function template = siftBuildTemplate(texturefilename,Ttexture)
global NNFNS

if( ~exist('Ttexture','var') )
    Ttexture = eye(4);
end

load(texturefilename);
[frames,desc,conf] = feature0D_siftfast(Itexture,0);
acceptedpoints = find(frames(1,:)>5&frames(1,:)<(size(Itexture,2)-5)&frames(2,:)>5&frames(2,:)<(size(Itexture,1)-5));
N = length(acceptedpoints);
if(N > 0 )
    points3d = Ttexture(1:3,1:4) * [(frames(1:2,acceptedpoints)-1).*repmat(imagescale,[1 N]);zeros(1,N);ones(1,N)];
    nndesc = NNFNS.Create(desc(:,acceptedpoints));
else
    points3d = [];
    nndesc = [];
end

[tdir, tname, text] = fileparts(texturefilename);

template = struct('I',Itexture,'nndesc',nndesc,'points3d',points3d,'points2d',frames(1:2,acceptedpoints),'extents',imagescale.*[size(Itexture,2);size(Itexture,1)],'type',tname);
