% [T,projerror] = computePoseFromCorrespondences(X,thresh)

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
function [T,projerror] = computePoseFromCorrespondences(X,thresh)
try
    [T,inliers] = ransac(X, @fittingfn, @distfn, @degenfn, 4, thresh,0,100,30);

    pts = T(1:3,1:3)*X(3:5,inliers)+repmat(T(1:3,4),[1 length(inliers)]);
    proj = pts(1:2,:) ./ repmat(pts(3,:),[2 1]);
    projerror = mean(sqrt(sum((X(1:2,inliers) - proj).^2,1)),2);
catch
    T = [];
    projerror = inf;
end

function T = fittingfn(x)
%% find a good initialization, estimate depths of 2D points
m2d = mean(x(1:2,:),2);
radius2d = sum(sqrt(sum( (x(1:2,:)-repmat(m2d,[1 size(x,2)])).^2, 1)));
m3d = mean(x(3:5,:),2);
radius3d = sum(sqrt(sum( (x(3:5,:)-repmat(m3d,[1 size(x,2)])).^2, 1)));
avgdepth = radius3d/radius2d;
Tinit = GetRigidTransformation(x(3:5,:), avgdepth*[x(1:2,:);ones(1,size(x,2))], 0);
Tinit_rotaxis = [AxisAngleFromRotation(Tinit(1:3,1:3));Tinit(1:3,4)];

[F,T_rotaxis] = leasqr(transpose(x(1:5,:)),zeros(size(x,2),1),Tinit_rotaxis,@getgradient,0.0001,20,transpose(x(6,:)));
T = [RotationMatrixFromAxisAngle(T_rotaxis(1:3)) T_rotaxis(4:6)];

function F = getgradient(x_t,T_rotaxis)
x = transpose(x_t);
R = RotationMatrixFromAxisAngle(T_rotaxis(1:3));
pts = R*x(3:5,:) + repmat(T_rotaxis(4:6),[1 size(x,2)]);
proj = pts(1:2,:) ./ repmat(pts(3,:),[2 1]);
F = sqrt(transpose(sum((x(1:2,:) - proj).^2,1)));

function r = degenfn(x)
%% calculate the area covered by the points, if small, reject
m = mean(x(1:2,:),2);
xm = x(1:2,:)-repmat(m,[1 size(x,2)]);
r = abs(det(xm*transpose(xm)))<0.001;

function [bestinliers, bestT] = distfn(T,x,thresh)
if( iscell(T) )
    bestT = [];
    bestinliers = [];
    for i = 1:length(T)
        Tt = T{i};
        pts = Tt(1:3,1:3)*x(3:5,:)+repmat(Tt(1:3,4),[1 size(x,2)]);
        proj = pts(1:2,:) ./ repmat(pts(3,:),[2 1]);
        d = sum((x(1:2,:) - proj).^2);
        inliers = find(d < thresh^2);
    
        if( length(inliers) > length(bestinliers) )
            bestinliers = inliers;
            bestT = T{i};
        end
    end
else
    pts = T(1:3,1:3)*x(3:5,:)+repmat(T(1:3,4),[1 size(x,2)]);
    proj = pts(1:2,:) ./ repmat(pts(3,:),[2 1]);
    d = sum((x(1:2,:) - proj).^2);
    bestinliers = find(d < thresh^2);
    bestT = T;
end
