close all
clear;

% Set directory

image_path = '../datasets/walking_sens';
addpath(image_path);
addpath(genpath(image_path))

load Calib_Results.mat

%-- Distortion coefficients:
% kc = [ 0.054879289895450 ; -0.106881360666110 ; 0.003970921416368 ; 0.011588508763675 ; 0.000000000000000 ];
% 
% tangential = [kc(3) kc(4)];
% radial = [kc(1) kc(2) kc(5)];


last_frame = 4432;

% cameraParams = cameraParameters('IntrinsicMatrix',KK,'TangentialDistortion',tangential,'RadialDistortion',radial);


% All in one Images: (Grayscale, scale 0.5, undistort and crop)
f = waitbar(0, 'Starting');
for i = 1:last_frame
    image = imread([image_path '/images/' sprintf('Image_%d.jpg',i)]);
    J = rgb2gray(image);

    % Scale
%     J = imresize(image,0.5);
    
    % Undistort
%     J = undistortImage(J,cameraParams);
    
    % Crop
%     J = imcrop(J, [15 10 1889 1059]);

    imwrite(J,[image_path '/images_grayscale/' sprintf('Image_%d.jpg',i)]);
    waitbar(i/last_frame, f, sprintf('Progress: %d %%', floor(i/last_frame*100)));
    pause(0.01);
end
close(f)





