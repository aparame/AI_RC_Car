clc
clear all
close all

pic = imread('63.jpg');
shape = size(pic);

rlim1=60;glim1=120;blim1=160; % Low Limits
rlim2=160;glim2=230;blim2=255; % High Limits
draw_rx = [2*shape(2)/3,2*shape(2)/3];
% Color Selection for lane detection
[x,y] = find(pic(:,:,1)<rlim1|pic(:,:,2)<glim1|pic(:,:,3)<blim1|...
    pic(:,:,1)>rlim2|pic(:,:,2)>glim2|pic(:,:,3)>blim2);
[x2,y2] = find(rlim2>pic(:,:,1)>rlim1 | glim2>pic(:,:,2)>glim1...
    | blim2>pic(:,:,3)>blim1);
nl_pic = pic;
for i =1:length(x)
    nl_pic(x(i), y(i),:)=[0, 0, 0];
    % make all colors below & above lim as 0
end

for i =1:length(x2)
    nl_pic(x2(i), y2(i),:)=[255, 255, 255];
    % make all colors between lim as 255
end
gray_pic=rgb2gray(pic);
edge_pic = edge(gray_pic,'canny',[0.1 0.5]);

% region masking by selecting a polygon
% specify row coordinates of polygon
a = [0, shape(2), shape(2), 0];

% specify column coordinates of polygon
b = [shape(1)*0.55, shape(1)*0.55, shape(1), shape(1)];
bw = roipoly(pic, a, b);
BW=(nl_pic(:,:,1)&bw | nl_pic (:,:,2)&bw...
    | nl_pic (:,:,3)&bw);

% Clean it up.
% Fill holes.
BW = imfill(BW, 'holes');
% Get rid of small blobs.
BW = bwareaopen(BW, 100);
% Smooth border
BW = imclose(BW, true(9));
subplot(2,1,1)

imshow(pic)
subplot(2,1,2)

imshow(BW)

