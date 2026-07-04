function [frameOut,targetSize] = prepareVideoFrame(fig,targetSize)
%PREPAREVIDEOFRAME Lock all H.264 frames to one even pixel dimension.

captured = getframe(fig);
imageData = captured.cdata;
if isempty(targetSize)
    targetHeight = size(imageData,1) + mod(size(imageData,1),2);
    targetWidth = size(imageData,2) + mod(size(imageData,2),2);
    targetSize = [targetHeight targetWidth];
end

canvas = uint8(255*ones(targetSize(1),targetSize(2),3));
copyHeight = min(size(imageData,1),targetSize(1));
copyWidth = min(size(imageData,2),targetSize(2));
canvas(1:copyHeight,1:copyWidth,:) = ...
    imageData(1:copyHeight,1:copyWidth,:);
frameOut.cdata = canvas;
frameOut.colormap = [];
end
