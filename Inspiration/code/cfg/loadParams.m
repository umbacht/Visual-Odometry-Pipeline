function [params] = loadParams(dataset)
% load all parameters to a struct
%   INPUT:
%           dataset: string, name of the datast e.g. 'parking'
%   OUTPUT:
%           param: struct, parameters
%
% INSTRUCTION: If you need a parameter add it here. Either under General or
% if related to a specific dataset directly under the respectice section

%% General params

    %Triangulate new landmarks
    params.nb_candidate_keypoints = 100; % number of candidate keypoints in S
    params.new_candidate_keypoints_dist_thresh = 3; % distance from old to new keypoint to be considered as "new"
    params.bearing_angle_threshold = 2/180*pi; % in radians

    % Feature detection
    params.ROI = [];
    params.feature_quality = 1e-4;
    params.filt_size = 5;
    % Feature matching
    params.unique = true;


%% DATASET KITTI

if strcmp(dataset, 'KITTI')

    params.path_data = 'data/kitti';

    % INITIALIZATION

    % General parameters
    params.use_matlab = 1; % O: use functions from exercise solutions, 1: use functions from matlab libraries
    params.init_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.init_viz = 0; % enables visualisation of initialization

    % Frame selection
    params.bootstrap_frames = [1 6]; % frames used for bootstraping, pair [0 4] causes problems for patch matching wihtout matlab since two possible solutions for rel. pose are possibl, fraction of valid points in front of camera = 0.5

    % Harris corner detector
    params.corner_patch_size = 9; % The patch size for harris corner detector
    params.harris_kappa = 0.08; % The kappa parameter in corner response function
    params.num_keypoints = 500; % The number of keypoints per frame
    params.nonmaximum_supression_radius = 8; % The surpression radius to prevent selecting neighbouring pixels as keypoints
    params.ROI = [];
    params.min_quality = 1e-4;

    % Keypoint descriptor
    params.descriptor_radius = 21; % The local square neighborhood used to describe a keypoint

    % Match keypoints
    params.matching_mode = 'klt'; % {'klt','patch_matching'}

    % Patch matching parameters
    params.match_lambda = 8; % Treshold for two descriptors to match

    % KLT parameters
    params.max_bilinear_error_klt = 0.8; % The forward-backward error treshold
    params.num_pyramid_levels = 4; % The number of pyramid level
    params.block_size = [21 21];  % Size of neighborhood around each point being tracked
    params.max_iteration = 40; % Maximum number of search iterations for each point

    % CONTINOUS OPERATION

    % General parameters
    params.cont_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.num_std = 1000;

    % KLT parameters
    params.cont_max_bilinear_error_klt = params.max_bilinear_error_klt; % The forward-backward error treshold
    params.cont_num_pyramid_levels = params.num_pyramid_levels; % The number of pyramid level
    params.cont_block_size = params.block_size;  % Size of neighborhood around each point being tracked
    params.cont_max_iteration = params.max_iteration; % Maximum number of search iterations for each point

    % Estimate Pose parameters
    params.cont_num_trials = 32000;
    params.cont_distance_treshold = 0.01;
    params.cont_confidence = 90;

    % Triangulate new landmarks
    params.new_candidate_keypoints_dist_thresh = 10; % distance from old to new keypoint to be considered as "new"
    params.bearing_angle_threshold = 3/180*pi; % in radians
    params.strong_percentage = 0.1;

end

%% DATASET MALAGA

if strcmp(dataset, 'Malaga')

    params.path_data = 'data/malaga';

    % INITIALIZATION

    % General parameters
    params.use_matlab = 1; % O: use functions from exercise solutions, 1: use functions from matlab libraries
    params.init_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.init_viz = 0; % enables visualisation of initialization

    % Frame selection
    params.bootstrap_frames = [1 9]; % frames used for bootstraping, pair [0 4] causes problems for patch matching wihtout matlab since two possible solutions for rel. pose are possibl, fraction of valid points in front of camera = 0.5

    % Harris corner detector
    params.corner_patch_size = 9; % The patch size for harris corner detector
    params.harris_kappa = 0.08; % The kappa parameter in corner response function
    params.num_keypoints = 500; % The number of keypoints per frame
    params.nonmaximum_supression_radius = 50; % The surpression radius to prevent selecting neighbouring pixels as keypoints
    params.ROI = [];
    params.min_quality = 1e-4;

    % Keypoint descriptor
    params.descriptor_radius = 9; % The local square neighborhood used to describe a keypoint

    % Match keypoints
    params.matching_mode = 'klt'; % {'klt','patch_matching'}

    % Patch matching parameters
    params.match_lambda = 200; % Treshold for two descriptors to match

    % KLT parameters
    params.max_bilinear_error_klt = 0.8; % The forward-backward error treshold
    params.num_pyramid_levels = 4; % The number of pyramid level
    params.block_size = [21 21];  % Size of neighborhood around each point being tracked
    params.max_iteration = 40; % Maximum number of search iterations for each point

    % CONTINOUS OPERATION

    % General parameters
    params.cont_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.num_std = 1000; % delete landmarks that are more far away than so many standard deviation

    % KLT parameters
    params.cont_max_bilinear_error_klt = params.max_bilinear_error_klt; % The forward-backward error treshold
    params.cont_num_pyramid_levels = params.num_pyramid_levels; % The number of pyramid level
    params.cont_block_size = params.block_size;  % Size of neighborhood around each point being tracked
    params.cont_max_iteration = params.max_iteration; % Maximum number of search iterations for each point

    % Estimate Pose parameters
    params.cont_num_trials = 50000;
    params.cont_distance_treshold = 0.003;
    params.cont_confidence = 90;

    % Triangulate new landmarks
    params.new_candidate_keypoints_dist_thresh = 15; % distance from old to new keypoint to be considered as "new"
    params.bearing_angle_threshold = 0.5/180*pi; % in radians
    params.strong_percentage = 0.5;

end

%% DATASET PARKING

if strcmp(dataset, 'parking')

    % INITIALIZATION

    params.path_data = 'data/parking';

    % General parameters
    params.use_matlab = 1; % O: use functions from exercise solutions, 1: use functions from matlab libraries
    params.init_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.init_viz = 1; % enables visualisation of initialization

    % Frame selection
    params.bootstrap_frames = [0 5]; % frames used for bootstraping, pair [0 4] causes problems for patch matching wihtout matlab since two possible solutions for rel. pose are possibl, fraction of valid points in front of camera = 0.5

    % Harris corner detector parking
    params.corner_patch_size = 9; % The patch size for harris corner detector
    params.harris_kappa = 0.08; % The kappa parameter in corner response function
    params.num_keypoints = 6000; % The number of keypoints per frame
    params.nonmaximum_supression_radius = 20; % The surpression radius to prevent selecting neighbouring pixels as keypoints
    params.ROI = [];
    params.min_quality = 1e-4;

    % Keypoint descriptor parking
    params.descriptor_radius = 21; % The local square neighborhood used to describe a keypoint

    % Match keypoints

    params.matching_mode = 'klt'; % {'klt','patch_matching'}

    % Patch matching parameters parking

    params.match_lambda = 8; % Treshold for two descriptors to match

    % KLT parameters
    params.max_bilinear_error_klt = 0.8; % The forward-backward error treshold
    params.num_pyramid_levels = 6; % The number of pyramid level
    params.block_size = [21 21];  % Size of neighborhood around each point being tracked
    params.max_iteration = 40; % Maximum number of search iterations for each point

    % CONTINOUS OPERATION

    % General parameters parking
    params.cont_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.num_std = 1000; % delete landmarks that are more far away than so many standard deviation

    % KLT parameters parking parking
    params.cont_max_bilinear_error_klt = params.max_bilinear_error_klt; % The forward-backward error treshold
    params.cont_num_pyramid_levels = params.num_pyramid_levels; % The number of pyramid level
    params.cont_block_size = params.block_size;  % Size of neighborhood around each point being tracked
    params.cont_max_iteration = params.max_iteration; % Maximum number of search iterations for each point

    % Estimate Pose parameters parking
    params.cont_num_trials = 32000;
    params.cont_confidence = 97;
    params.cont_distance_treshold = 0.01;



    % Triangulate new landmarks parking parking
    params.new_candidate_keypoints_dist_thresh = 15; % distance from old to new keypoint to be considered as "new"
    params.bearing_angle_threshold = 1.5/180*pi; % in radians
    params.strong_percentage = 0.1;
    params.nb_candidate_keypoints = 300; % number of candidate keypoints in S


end


%% DATASET ROSIE

if strcmp(dataset, 'rosie')

    params.path_data = 'data/rosie';

    % INITIALIZATION

    % General parameters
    params.use_matlab = 1; % O: use functions from exercise solutions, 1: use functions from matlab libraries
    params.init_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.init_viz = 1; % enables visualisation of initialization

    % Frame selection
    params.bootstrap_frames = [0 9]; % frames used for bootstraping,

    % Harris corner detector
    params.corner_patch_size = 9; % The patch size for harris corner detector
    params.harris_kappa = 0.08; % The kappa parameter in corner response function
    params.num_keypoints = 225; % The number of keypoints per frame
    params.nonmaximum_supression_radius = 8; % The surpression radius to prevent selecting neighbouring pixels as keypoints
    params.ROI = []; % Region of interest in the image
    params.min_quality = 1e-4;

    % Keypoint descriptor
    params.descriptor_radius = 21; % The local square neighborhood used to describe a keypoint

    % Match keypoints
    params.matching_mode = 'klt'; % {'klt','patch_matching'}

    % Patch matching parameters
    params.match_lambda = 8; % Treshold for two descriptors to match

    % KLT parameters
    params.max_bilinear_error_klt = 0.8; % The forward-backward error treshold
    params.num_pyramid_levels = 4; % The number of pyramid level
    params.block_size = [21 21];  % Size of neighborhood around each point being tracked
    params.max_iteration = 40; % Maximum number of search iterations for each point

    % CONTINOUS OPERATION

    % General parameters
    params.cont_debug = 0; % 0: no debug output, 1: debug outputs such as plots
    params.num_std = 500; % delete landmarks that are more far away than so many standard deviation

    % KLT parameters
    params.cont_max_bilinear_error_klt = params.max_bilinear_error_klt; % The forward-backward error treshold
    params.cont_num_pyramid_levels = params.num_pyramid_levels; % The number of pyramid level
    params.cont_block_size = params.block_size;  % Size of neighborhood around each point being tracked
    params.cont_max_iteration = params.max_iteration; % Maximum number of search iterations for each point

    % Estimate Pose parameters
    params.cont_num_trials = 25000;
    params.cont_distance_treshold = 0.01;
    params.cont_confidence = 90;

    % Triangulate new landmarks
    params.new_candidate_keypoints_dist_thresh = 15; % distance from old to new keypoint to be considered as "new"
    params.bearing_angle_threshold = 0.75/180*pi; % in radians
    params.strong_percentage = 0.1;

end

end
