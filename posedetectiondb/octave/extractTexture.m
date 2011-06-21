% [Itexture,imagescale]=extractTexture(I,T,KK,xyoffset,extents,savefilename)
%
% Output:
%   imagescale - how many meters a pixel is

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
function [Itexture,imagescale] = extractTexture(I,T,KK,xyoffset,extents,savefilename)

% get the 4 corners and find dimensions of texture
corners = [xyoffset xyoffset+extents.*[1;0] xyoffset+extents.*[0;1] xyoffset+extents.*[1;1]; 0 0 0 0];
corners = T(1:3,1:4) * [corners; ones(1,size(corners,2))];
corners2d = KK * (corners ./ repmat(corners(3,:),[3 1]));

imagescale=zeros(2,1);
width1 = floor(norm(corners2d(:,1)-corners2d(:,2)));
width2 = floor(norm(corners2d(:,3)-corners2d(:,4)));
width = max(width1,width2);
if( width1 > width2 )
    imagescale(1) = norm(corners(:,1)-corners(:,2))/width;
else
    imagescale(1) = norm(corners(:,3)-corners(:,4))/width;
end

height1 = floor(norm(corners2d(:,1)-corners2d(:,3)));
height2 = floor(norm(corners2d(:,2)-corners2d(:,4)));
height = max(height1,height2);
if( height1 > height2 )
    imagescale(2) = norm(corners(:,1)-corners(:,3))/height;
else
    imagescale(2) = norm(corners(:,2)-corners(:,4))/height;
end

% get the texture
xrange = 0:imagescale(1):extents(1);
yrange = 0:imagescale(2):extents(2);
[X,Y] = meshgrid(xrange(1:width),yrange(1:height));
N = size(X,1)*size(X,2);
points = transpose([X(:) Y(:)]) + repmat(xyoffset,[1 N]);
points = T(1:3,1:4) * [points;zeros(1,N);ones(1,N)];
points2d = KK * (points ./ repmat(points(3,:),[3 1]));
Colors = imbilinearsample(I, points2d([2 1],:));
Itexture = permute(reshape(Colors,[3 size(X)]),[2 3 1]);

if( exist('savefilename','var') )
    save('-v7',savefilename,'Itexture','imagescale');
end
