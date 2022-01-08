clear all;
close all;

%% Adding required paths

kitti_path = '../datasets/kitti';
malaga_path = '../datasets/malaga';
parking_path = '../datasets/parking';
walking_path = '../datasets/walking';

addpath(kitti_path);
addpath(parking_path);
addpath(malaga_path);
addpath(walking_path);
addpath(genpath('Exercise Solutions'));

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking, 3: walking

if ds == 0 % KITTI
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 2760;
    parameter.n_th_element = 1;
    
    % Parameters
    parameter.K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    parameter.bootstrap_frames = [0, 5];
    % Initialization:
    % PointTracker
    parameter.MaxBidirectionalError_init = 0.7;
    parameter.NumPyramidLevels_init = 6;
    parameter.BlockSize_init = [31 31];
    parameter.MaxIterations_init = 40;
    % Continious:
    % PointTracker
    parameter.MaxBidirectionalError_cont = 0.7;
    parameter.NumPyramidLevels_cont = 6;
    parameter.BlockSize_cont = [31 31];
    parameter.MaxIterations_cont = 50;
    % Triangulation of new landmarks
    % PointTracker
    parameter.MaxBidirectionalError_triang = 0.7;
    parameter.NumPyramidLevels_triang = 6;
    parameter.BlockSize_triang = [31 31];
    parameter.MaxIterations_triang = 50;
    % Used in Init and Cont:
    % Harris 
    parameter.corner_patch_size = 9;
    parameter.harris_patch_size = 9;
    parameter.harris_kappa = 0.08;
    parameter.nonmaximum_supression_radius = 16;
    parameter.descriptor_radius = 9;
    parameter.match_lambda = 4;
    % Fundamental Matrix RANSAC
    parameter.method = 'RANSAC';
    parameter.NumTrials = 3000;
    parameter.DistanceThreshold = 0.0001;
    % New keypoints
    parameter.num_keypoints = 500;
    parameter.threshold = 5; %Minimum distance to previous
    parameter.angle_threshold = 3/180*pi; % Bearing angle threshold
    parameter.max_distance = 500;


elseif ds == 1 % MALAGA
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    parameter.n_th_element = 1;
    last_frame = length(left_images);
    % Parameters
    parameter.K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    parameter.bootstrap_frames = [1, 7];
    % Initialization:
    % PointTracker
    parameter.MaxBidirectionalError_init = 0.7;
    parameter.NumPyramidLevels_init = 6;
    parameter.BlockSize_init = [31 31];
    parameter.MaxIterations_init = 40;
    % Continuous:
    % PointTracker
    parameter.MaxBidirectionalError_cont = 0.7;
    parameter.NumPyramidLevels_cont = 6;
    parameter.BlockSize_cont = [31 31];
    parameter.MaxIterations_cont = 40;
    % Triangulation of new landmarks
    % PointTracker
    parameter.MaxBidirectionalError_triang = 0.7;
    parameter.NumPyramidLevels_triang = 6;
    parameter.BlockSize_triang = [31 31];
    parameter.MaxIterations_triang = 40;
    % Used in Init and Cont:
    % Harris 
    parameter.corner_patch_size = 9;
    parameter.harris_patch_size = 9;
    parameter.harris_kappa = 0.08;
    parameter.nonmaximum_supression_radius = 16;
    parameter.descriptor_radius = 9;
    parameter.match_lambda = 4;
    % Fundamental Matrix RANSA
    parameter.method = 'RANSAC';
    parameter.NumTrials = 3000;
    parameter.DistanceThreshold = 0.0001;
    % New keypoints
    parameter.num_keypoints = 500;
    parameter.threshold = 5; %Minimum distance to previous
    parameter.angle_threshold = 3/180*pi; % Bearing angle threshold
    parameter.max_distance = 500;


elseif ds == 2 % PARKING
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    parameter.n_th_element = 1;
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    % Parameters
    parameter.K = load([parking_path '/K.txt']);
    parameter.bootstrap_frames = [0, 5];
    % Initialization:
    % PointTracker
    parameter.MaxBidirectionalError_init = 0.7;
    parameter.NumPyramidLevels_init = 6;
    parameter.BlockSize_init = [31 31];
    parameter.MaxIterations_init = 40;
    % Continuous:
    % PointTracker
    parameter.MaxBidirectionalError_cont = 0.7;
    parameter.NumPyramidLevels_cont = 6;
    parameter.BlockSize_cont = [31 31];
    parameter.MaxIterations_cont = 40;
    % Triangulation of new landmarks
    % PointTracker
    parameter.MaxBidirectionalError_triang = 0.7;
    parameter.NumPyramidLevels_triang = 6;
    parameter.BlockSize_triang = [31 31];
    parameter.MaxIterations_triang = 40;
    % Used in Init and Cont:
    % Harris 
    parameter.corner_patch_size = 9;
    parameter.harris_patch_size = 9;
    parameter.harris_kappa = 0.08;
    parameter.nonmaximum_supression_radius = 16;
    parameter.descriptor_radius = 9;
    parameter.match_lambda = 4;
    % Fundamental Matrix RANSAC
    parameter.method = 'RANSAC';
    parameter.NumTrials = 3000;
    parameter.DistanceThreshold = 0.0001;
    % New keypoints
    parameter.num_keypoints = 500;
    parameter.threshold = 15; %Minimum distance to previous
    parameter.angle_threshold = 10/180*pi; % Bearing angle threshold
    parameter.max_distance = 200;


elseif ds == 3 % WALKING
    % Path containing images, depths and all...
    last_frame = 4432;
    parameter.n_th_element = 3;
    % Parameters
    parameter.K = load([walking_path '/K_walking.txt']);
    parameter.bootstrap_frames = [1 10];%[2955 2965];%
    % Initialization:
    % PointTracker
    parameter.MaxBidirectionalError_init = 0.7;
    parameter.NumPyramidLevels_init = 6;
    parameter.BlockSize_init = [31 31];
    parameter.MaxIterations_init = 40;
    % Continuous:
    % PointTracker
    parameter.MaxBidirectionalError_cont = 0.6;
    parameter.NumPyramidLevels_cont = 6;
    parameter.BlockSize_cont = [31 31];
    parameter.MaxIterations_cont = 50;
    % Triangulation of new landmarks
    % PointTracker
    parameter.MaxBidirectionalError_triang = 0.6;
    parameter.NumPyramidLevels_triang = 6; 
    parameter.BlockSize_triang = [31 31];
    parameter.MaxIterations_triang = 50;
    % Used in Init and Cont:
    % Harris 
    parameter.corner_patch_size = 9;
    parameter.harris_patch_size = 9;
    parameter.harris_kappa = 0.08;
    parameter.nonmaximum_supression_radius = 16;
    parameter.descriptor_radius = 9;
    parameter.match_lambda = 4;
    % Fundamental Matrix RANSAC
    parameter.method = 'RANSAC';
    parameter.NumTrials = 5000;
    parameter.DistanceThreshold = 0.00001;
    % New keypoints
    parameter.num_keypoints = 500;
    parameter.threshold = 5; %Minimum distance to previous
    parameter.angle_threshold = 3/180*pi; % Bearing angle threshold 10
    parameter.max_distance = 300;

    parameter.distortion_frame = [75 50 1769 979];
    
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
init_frames = cell(parameter.bootstrap_frames(2)-parameter.bootstrap_frames(1)+1,1);
init_frame_ids = parameter.bootstrap_frames(1):parameter.bootstrap_frames(2);

if ds == 0
    for i = 1:size(init_frame_ids,2)
        init_frames{i} = imread([kitti_path '/05/image_0/' ...
            sprintf('%06d.png',init_frame_ids(i))]);
    end
elseif ds == 1
    for i = 1:size(init_frame_ids,2)
        init_frames{i} = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(init_frame_ids(i)).name]));
    end
elseif ds == 2
    for i = 1:size(init_frame_ids,2)
        init_frames{i} = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',init_frame_ids(i))]));
    end
elseif ds == 3 
    for i = 1:parameter.n_th_element:size(init_frame_ids,2)
        init_frames{i} = imread([walking_path '/image_distorted/' ...
            sprintf('Image_%d.jpg',init_frame_ids(i))]);
    end    
else
    assert(false);
end

[P1,X1, T1] = initializationKLT(init_frames(1:parameter.n_th_element:end), parameter);

%% Continuous operation
% Creating intial state
S_prv.P = P1(1:2,:)';
S_prv.X = X1(1:3,:)';
S_prv.C = [];
S_prv.F = [];   
S_prv.T = [];

T_WC_prv = [T1; 0 0 0 1];

range = (parameter.bootstrap_frames(2)+1):last_frame;
image_prv = init_frames{1};

% Initialize data for plotting
last20Frameidx = 1:20;
numMatched3dPoints = [zeros(1, 18), size(S_prv.X, 1), ...
                        zeros(1, last_frame - parameter.bootstrap_frames(2))];
xzCoordinates = [zeros(2, 18), T_WC_prv([1, 3], end), ...
                       zeros(2, last_frame - parameter.bootstrap_frames(2))];
                                              
for i = range(1:parameter.n_th_element:end)
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image_crt = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image_crt = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image_crt = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 3
    image_crt = im2uint8(imread([walking_path '/image_distorted/' ...
        sprintf('Image_%d.jpg',i)]));
    else
        assert(false);
    end

    [S_crt, T_WC_crt] = continuous_operation(image_crt, image_prv, S_prv, T_WC_prv,parameter);
    
    % Plotting
    [numMatched3dPoints, xzCoordinates, last20Frameidx] = ...
        plotting(S_crt, T_WC_crt, image_crt, numMatched3dPoints, xzCoordinates, last20Frameidx);

    % Makes sure that plots refresh    
    pause(0.01);
        
    image_prv = image_crt;
    S_prv = S_crt;
    T_WC_prv = T_WC_crt;

end
