clear all;
close all;

%% Adding required paths

kitti_path = '../datasets/kitti';
malaga_path = '../datasets/malaga';
parking_path = '../datasets/parking';

addpath(kitti_path);
addpath(parking_path);
addpath(malaga_path)

addpath(genpath('Exercise Solutions'));

%% Setup
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    parameter.K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    parameter.bearing_angle_threshold = 3/180*pi;

elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    parameter.K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    parameter.K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [0, 2];

if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

[P1,X1, C1, F1, T1] = initialization(img0, img1, parameter.K);



%% Continuous operation
% Creating intial state
S_prv.P = P1;
S_prv.X = X1;
S_prv.C = [];
S_prv.F = [];
S_prv.T = [];

T_WC_prv = [T1; 0 0 0 1];

% This needs to be fixed in initialization part:
S_prv.X = S_prv.X(1:3,:)';
keypoints_prv = flipud(S_prv.P(1:2,:));
S_prv.P = keypoints_prv';

range = (bootstrap_frames(2)+1):last_frame;
image_prv = img1;

for i = range(3:50)
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image_crt = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image_crt = rgb2gray(imread([malagdfsa_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image_crt = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end

    [S_crt, T_WC_crt] = continuous_operation(image_crt, image_prv, S_prv, T_WC_prv,parameter);

    % Makes sure that plots refresh.    
    pause(0.01);
    
    image_prv = image_crt;
    S_prv = S_crt;
    T_WC_prv = T_WC_crt;

end