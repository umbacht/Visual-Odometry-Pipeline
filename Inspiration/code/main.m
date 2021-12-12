%% Clear environment

clear
close all
clc
%rng(1) % Set seed for reproducable results

%% Setup
% Setup for the respective dataset

ds = 0; % 0: KITTI, 1: Malaga, 2: parking 3: rosie
datasets = {'KITTI','Malaga', 'parking','rosie'};

% load params
params = loadParams(datasets{ds+1});

% KITTI
if ds == 0     
    % need to set kitti_path to folder containing "00" and "poses"
    ground_truth = load([params.path_data '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];

% MALAGA
elseif ds == 1    
    % Path containing the many files of Malaga 7.
    images = dir([params.path_data ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
    
% PARKING   
elseif ds == 2    
    % Path containing images, depths and all...
    last_frame = 598;
    K = load([params.path_data '/K.txt']);
    ground_truth = load([params.path_data '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    
% ROSIE
elseif ds == 3    
   K =  load([params.path_data '/K.txt']);
   last_frame = 921;
   
else
    assert(false);
end

%% Initialize
% Initialization for the respective dataset

% create camera object with transpose K
params.camera = cameraParameters('IntrinsicMatrix',K'); % Matlab uses transposed K

% Bootstrap frames
bootstrap_frames = params.bootstrap_frames;

% KITTI
if ds == 0
    img0 = imread([params.path_data '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([params.path_data '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);

% MALAGA
elseif ds == 1
    img0 = rgb2gray(imread([params.path_data ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([params.path_data ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));

% PARKING
elseif ds == 2 
    img0 =  im2uint8(rgb2gray(imread([params.path_data ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))])));
    img1 =  im2uint8(rgb2gray(imread([params.path_data ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))])));

% ROSIE
elseif ds == 3
    img0 = imread([params.path_data ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]);
    img1 = imread([params.path_data ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]);
    
else
    assert(false);
end
    
% If empty, set default rectangular region dimensions for detecting the
% Harris corners
if isempty(params.ROI)
    params.ROI = [1, 1, size(img0,2), size(img0,1)];
end

% Elements to initialize VO pipeline: 
% T_WC_init: 4x4, homog. transform. Matrix from camera 1 frame to WCS
% init_kp: Mx2, keypoints which were triangulated
% init_landmarks: Mx3, landmarks in WCS (triangulated keypoints) 
[T_WC_init, init_kp, init_landmarks] = initialization(img0, img1, params);

% If empty, set default rectangular region dimensions for detecting the
% Harris corners
if isempty(params.ROI)
    params.ROI = [1, 1, size(img0,2), size(img0,1)];
end
%% Continuous

% initial state
S_prev.P = init_kp;
S_prev.X = init_landmarks;
S_prev.C = [];
S_prev.F = [];
S_prev.T = [];
img_prev = img1;
T_WC_prev = T_WC_init;



% init data for plotting
last20FramesIndices = 1:20;
% initialize all of the last 20 data points that don't yet exist with zeros
numberOfTrackedLandmarks_all = [zeros(1, 18), size(S_prev.X, 1), ...
                                zeros(1, last_frame - bootstrap_frames(2))];

% x and z coordinates of camera for all frames
allWorldCoordinates = [zeros(2, 18), T_WC_init([1, 3], end), ...
                       zeros(2, last_frame - bootstrap_frames(2))];

% range of frames
range = bootstrap_frames(2)+1:last_frame;                   
                   
% loop through all frames of respective dataset after initial frames
for i = range

    % KITTI
    if ds == 0
        % get current frame
        img_curr = imread([params.path_data '/00/image_0/' sprintf('%06d.png',i)]);
        
    % MALAGA    
    elseif ds == 1
        % get current frame
        img_curr = rgb2gray(imread([params.path_data ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
        
    % PARKING
    elseif ds == 2
        % get current frame
        img_curr = im2uint8(rgb2gray(imread([ params.path_data sprintf('/images/img_%05d.png',i) ])));
    
    %ROSIE
    elseif ds == 3
        % get current frame
        img_curr = imread([params.path_data sprintf('/images/img_%05d.png',i)]);
        
    else
        assert(false)
    end
    
    % process current frame
    [S_curr, T_WC_curr] = processFrame(img_curr, img_prev, S_prev, params, T_WC_prev);

    % visualize results for curent frame
    [last20FramesIndices, numberOfTrackedLandmarks_all, ...
        allWorldCoordinates] = ...
            plotting(S_curr, T_WC_curr, img_curr, ...
                numberOfTrackedLandmarks_all, allWorldCoordinates, ...
                last20FramesIndices);


   % delay to give plots time to refresh
   pause(0.01)
   
   % set previous image to current
   img_prev = img_curr;
   S_prev = S_curr;
   T_WC_prev = T_WC_curr;
   
end

% show image when finished
imshow('finish.jpg')

