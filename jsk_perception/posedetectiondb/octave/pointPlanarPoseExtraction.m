% T = pointPlanarPoseExtraction(template,frames,desc,conf,KK,show,I)
%
% returns the pose of the template in I
% Input:
%   template - {Itexture,nndesc,points3d}
% Output:
%   T - 4x4 matrix of the pose

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
function [T,projerror] = pointPlanarPoseExtraction(template,frames,desc,conf,KK,show,I)
global NNFNS
if( ~exist('show','var') )
    show = 0;
end

[neighs,dists] = NNFNS.Search(template.nndesc,desc,2);

goodinds = find(dists(:,1) < 0.7*dists(:,2));

if( show == 1 )
    Itotal = zeros(size(I)+[0 size(template.I,2) 0]);
    Itotal(:,1:size(I,2),:) = I;
    Itotal(1:size(template.I,1),(size(I,2)+1):end,:) = template.I;
    clf; imshow(Itotal*64); hold on;
    for ind = transpose(goodinds)
        pt = template.points2d(:,neighs(ind,1))+[size(I,2);0];
        plot([frames(1,ind) pt(1)],[frames(2,ind) pt(2)]);
    end
    drawnow;
end

iKK = inv(KK);
X = [iKK(1:2,1:2)*frames(1:2,goodinds)+repmat(iKK(1:2,3),[1 length(goodinds)]); template.points3d(:,neighs(goodinds,1));conf(goodinds)];
thresh = 10/KK(1,1);

[T,projerror] = computePoseFromCorrespondences(X,thresh);

if( ~isempty(T) && show == 2 )
    points3d = [transpose([0 0; 1 0; 1 1; 0 1; 0 0]) .* repmat(template.extents,[1 5]); zeros(1,5)];
    points3d = KK*(T(1:3,1:3)*points3d + repmat(T(1:3,4),[1 5]));
    proj = points3d(1:2,:) ./ repmat(points3d(3,:),[2 1]);
    clf; imshow(I*64); hold on;
    plot(proj(1,:),proj(2,:),'b');
    drawnow;
end
