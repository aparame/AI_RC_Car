clc
clear all
%% Import Trained Model
net = importKerasNetwork("my_model.h5",...
  OutputLayerType="classification");
Classes = net.Layers(end).Classes;
% Classes(14) = 'STOP';

%% Pre Process
cd drive-download-20220420T182727Z-001/
pic = imread('53.jpg');
imshow(pic)
%%
Im = imread("STOP_sign.jpeg");
size(Im)
% Resize the image to the input size of the network.
InputSize = net.Layers(1).InputSize;
Im = imresize(Im,InputSize(1:2));
Im = rgb2gray(Im);
% % The inputs to  EfficientNetV2L require further preprocessing. Rescale the image. Normalize the image by subtracting the training images mean and dividing by the training images standard deviation. To find the values that you should use, see https://github.com/keras-team/keras-applications/blob/master/keras_applications/imagenet_utils.py.
% ImProcessed = rescale(Im,0,1);
% meanIm = [0.485 0.456 0.406];
% stdIm = [0.229 0.224 0.225];
% ImProcessed = (ImProcessed-reshape(meanIm,[1 1 3]))./reshape(stdIm,[1 1 3]);
% Classify Image Using the Imported Network

% Classify the image using the imported network. Show the image with the classification label.
label = classify(net,Im);
imshow(Im)
title(strcat("Predicted label: ",string(label)))