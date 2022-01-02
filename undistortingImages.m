clear all;
close all;

% Set directory
walking_path = '../datasets/walking_horz_video';
addpath(walking_path);
addpath(genpath(walking_path))
addpath(genpath('Exercise Solutions'));

% From Calib_Results.m
%-- Focal length:
fc = [ 3064.716473538157516 ; 3064.871114102417778 ];
%-- Principal point:
cc = [ 2074.906646124388772 ; 1554.518695308139968 ];
%-- Skew coefficient:
alpha_c = 0.000000000000000;
%-- Distortion coefficients:
kc = [ 0.054879289895450 ; -0.106881360666110 ; 0.003970921416368 ; 0.011588508763675 ; 0.000000000000000 ];
tangential = [kc(3) kc(4)];
radial = [kc(1) kc(2) kc(5)];
%-- Image size:
nxny = [4032 3024];

intrinsics = cameraIntrinsics(fc,cc,nxny);
K = intrinsics.IntrinsicMatrix; 

cameraParams = cameraParameters('IntrinsicMatrix',K,'TangentialDistortion',tangential,'RadialDistortion',radial);

last_frame = 2116;

for i = 1:2
    image = imread([walking_path '/images/' sprintf('Image_%d.jpg',i)]);
    image = rgb2gray(image);
    %imshow(image);
    %pause(0.01)

    % Undistort image and safe in dataset
    J = undistortImage(image,cameraParams);
    imwrite(J,[walking_path '/undistorted_images/' sprintf('Image_%d.jpg',i)]);

end


% calibration:
% PROBLEM: Datatype is 3024x4032x4 (last one is 255)
%images = imageDatastore(fullfile([walking_path '/calibration_images/']));
% images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata','calibration','mono'));
% [imagePoints, boardSize] = detectCheckerboardPoints(images.Files);
% squareSize = 29; &ours: 27
% worldPoints = generateCheckerboardPoints(boardSize,squareSize);
% I = readimage(images,1);
% imageSize = [size(I,1),size(I,2)];
% cameraParams = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize)



