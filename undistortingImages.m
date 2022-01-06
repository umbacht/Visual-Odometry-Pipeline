clear all;
close all;

% Set directory
image_path = '../datasets/walking_horz_video';
addpath(image_path);
addpath(genpath(image_path))
addpath(genpath('Exercise Solutions'));

% From Calib_Results.m
% Focal length:
fc = [ 3064.716473538157516 ; 3064.871114102417778 ];
% Principal point:
cc = [ 2074.906646124388772 ; 1554.518695308139968 ]/2; % CHECK RESOLUTION /2 FOR [1920 1080]
% Skew coefficient:
alpha_c = 0.000000000000000;
% Distortion coefficients:
kc = [ 0.054879289895450 ; -0.106881360666110 ; 0.003970921416368 ; 0.011588508763675 ; 0.000000000000000 ];
tangential = [kc(3) kc(4)];
radial = [kc(1) kc(2) kc(5)];


%-- Image size:
nxny = [1920 1080];%[4032 3024];

last_frame = 4432;


intrinsics = cameraIntrinsics(fc,cc,nxny);
K = intrinsics.IntrinsicMatrix; 
cameraParams = cameraParameters('IntrinsicMatrix',K,'TangentialDistortion',tangential,'RadialDistortion',radial);



% Undistort Images: 
% f = waitbar(0, 'Starting');
% for i = 1:last_frame
%     image = imread([image_path '/Images_60Hz_resized/' sprintf('Image_%d.jpg',i)]);
%     image = rgb2gray(image);
% 
%     % Undistort image and safe in dataset
%     J = undistortImage(image,cameraParams);
%     imwrite(J,[image_path '/Imgages_60Hz_undistorted/' sprintf('Image_%d.jpg',i)]);
%     waitbar(i/last_frame, f, sprintf('Progress: %d %%', floor(i/last_frame*100)));
%     pause(0.01);
% end
% close(f)

% Rescale Images:
% f = waitbar(0, 'Starting');
% for i = 1:last_frame
%     image = imread([image_path '/Images_60Hz/' sprintf('Image_%d.jpg',i)]);
%     %imshow(image);
%     %pause(0.01)
% 
%     % Undistort image and safe in dataset
%     J = imresize(image,0.5);
%     imwrite(J,[image_path '/Images_60Hz_resized/' sprintf('Image_%d.jpg',i)]);
%     waitbar(i/last_frame, f, sprintf('Progress: %d %%', floor(i/last_frame*100)));
%     pause(0.01);
% end
% close(f)

% Crop Images:
% f = waitbar(0, 'Starting');
% for i = 1:last_frame
%     image = imread([image_path '/Imgages_60Hz_undistorted/' sprintf('Image_%d.jpg',i)]);
%     %imshow(image);
%     %pause(0.01)
% 
%     % Crop image and safe in dataset
%     J = imcrop(image, [15 10 1889 1059]);
%     imwrite(J,[image_path '/Images_60Hz_cropped/' sprintf('Image_%d.jpg',i)]);
%     waitbar(i/last_frame, f, sprintf('Progress: %d %%', floor(i/last_frame*100)));
%     pause(0.01);
% end
% close(f)



