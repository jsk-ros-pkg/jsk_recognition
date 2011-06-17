% [frames, descriptors, confidence]  = feature0D_siftfast(I,show)
function [frames, descriptors, confidence]  = feature0D_siftfast(I,show)
 
if( ~exist('show','var') )
    show = 0;
end

if( isrgb(I) )
    [frames,descriptors] = siftfast(rgb2gray(I));
else
    [frames,descriptors] = siftfast(I);
end

if( ~isempty(frames) )
    frames = frames([1 2 4 3],:);
end
confidence = ones(1,size(frames,2));
