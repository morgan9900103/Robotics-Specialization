close all

imagepath = './train';


I = imread(sprintf('%s/%03d.png',imagepath, 2));
imshow(I)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = [0.163652138436360,0.564142481838881];
sig = [0.00106756979695403,-0.00281883764893072;-0.00281883764893072,0.0284465761665965];
thre = 0.01;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 

HSV = rgb2hsv(I);

[m, n, o] = size(I);
prob = zeros(m, n);

for i = 1:m
    for j = 1:n
        x = [HSV(i,j,1) HSV(i,j,2)];
        if abs(x - mu) < thre
            prob(i,j) = 1;
        end
    end
end

figure, imshow(prob);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

bw_biggest = false(size(prob));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(prob);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
figure,
imshow(bw_biggest); hold on;

% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
A = regionprops(CC,'Centroid');
loc = A(idx).Centroid;
plot(loc(1), loc(2),'r+');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)


