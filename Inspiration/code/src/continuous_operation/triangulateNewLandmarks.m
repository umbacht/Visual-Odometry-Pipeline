function S =  triangulateNewLandmarks(I_curr, I_prev, S, T_WC_curr, params)

% Inputs:   I_curr:   current image
%
%           I_prev:   previous image
%
%           S:        state, consisting of
%
%                                 P:    2D keypoints that are tracked
%                                       across frames
%                                       (Nx2)
%
%                                 X:    corresponding landmarks to key-
%                                       points in world frame.
%                                        (Nx3)
%
%                                 C:    image coordinates of candidate key-
%                                       points. Might become real keypoints 
%                                       later.
%                                       (Nx2)
%
%                                 F:    image coordinates of candidate key-
%                                       oints when they first appeared.
%                                       (Nx2)
%
%                                 T:    camera poses each corresponding to
%                                       one candidate keypoint.
%                                       This is the camera pose we were in
%                                       when the corresponding candidate
%                                       keypoint first appeared.
%                                       (4x4xN)
%
%
%              T_WC_curr:  current camera pose
%                           (4x4)
%
%
%              params:      params struct
%

%% Track previous candidate keypoints in current frame via KLT

if size(S.C, 1) > 0 % only needs to be done if there exist candidate keypoints at all

    % track candidate points
    point_tracker = vision.PointTracker('MaxBidirectionalError', params.max_bilinear_error_klt, ...
                                       'NumPyramidLevels', params.num_pyramid_levels, ...
                                       'BlockSize', params.block_size, ...
                                       'MaxIterations', params.max_iteration);

    initialize(point_tracker, S.C, I_prev); % initialize pointTracker with previous image

    % find candidate points on current image
    [C_curr, is_valid] = point_tracker(I_curr);

    % update current candidates as tracked candidate keypoints that are still valid
    S.C = C_curr(is_valid, :);
    S.F = S.F(is_valid, :);
    S.T = S.T(:, :, is_valid);
    
    release(point_tracker); % do we need this?
end

%% Check if current candidate keypoints satisfy the conditions to be added as real keypoints
K = params.camera.IntrinsicMatrix'; % K is in lecture convention
T_WC_curr = [T_WC_curr(1:3,1:3).', T_WC_curr(1:3,4);T_WC_curr(4,:)];
T_CW_curr = inv(T_WC_curr); % transformation from world to camera frame
new_keypoint_ids = [];
for i = 1:size(S.C, 1) % iterate through candidate points

    [R_C, T_C] = cameraPoseToExtrinsics(T_WC_curr(1:3,1:3), T_WC_curr(1:3,4));
    M_C = cameraMatrix(params.camera, R_C, T_C);
    
    [R_F, T_F] = cameraPoseToExtrinsics(S.T(1:3,1:3, i), S.T(1:3, 4, i));
    M_F = cameraMatrix(params.camera, R_F, T_F);
    
    candidate_landmark = triangulate(S.C(i, :), S.F(i, :), M_C, M_F);

    % compute bearing angle alpha
    v_cp = T_WC_curr(1:3, 4) - candidate_landmark'; % vector pointing from current camera location to candidate landmark
    v_fp = S.T(1:3, 4, i) - candidate_landmark'; % vector pointing from first camera location with that candidate keypoint to candidate landmark

    % Check if bearing angle is sufficiently large
    alpha = acos(v_cp'*v_fp/(norm(v_cp)*norm(v_fp)));
    if(alpha > params.bearing_angle_threshold)

        new_keypoint_ids = [new_keypoint_ids, i];
        
        % add candidate landmark to landmarks
        S.X = [S.X;
                candidate_landmark];

        % add candidate keypoint to keypoints
        S.P = [S.P;
                S.C(i, :)];
    end
end

% delete new keypoints from candidates list
S.C(new_keypoint_ids, :) = [];
S.F(new_keypoint_ids, :) = [];
S.T(:, :, new_keypoint_ids) = [];

%% Find new candidate keypoints in current frame
keypoints = detectHarrisFeatures(I_curr, ...
                                'MinQuality', params.min_quality, ...
                                'ROI', params.ROI, ...
                                'FilterSize', params.corner_patch_size);

% remove corners that are candidate keypoints or proper keypoints already
is_new = logical(ones(keypoints.length(), 1)); % logical index array to specify which keypoints are new
existing_keypoints = [S.P; S.C]; 
for i = 1:keypoints.length() % iterate through fresh detected corners

    % compute distance of new candidate keypoint to all previous proper keypoints
    distances = [sqrt(sum((keypoints.Location(i, :) - existing_keypoints).^2, 2))];
                  
    % if the distance to at least one of the old candidate points is too
    % small (meaning it exists already), it is discarded.
    if(min(distances) < params.new_candidate_keypoints_dist_thresh)
       is_new(i) = 0;
    end
end

% collect new candidate keypoints that have enough distance to old candidate keypoints
nb_new_candidate_keypoints = min([sum(is_new), params.num_keypoints - length(S.C) - length(S.P)]);
if nb_new_candidate_keypoints > 0
    keypoints_new = keypoints(is_new);
    strongest_keypoints = keypoints_new.selectStrongest(max(round(params.strong_percentage*nb_new_candidate_keypoints), 1));
    non_strong_keypoints = keypoints_new(keypoints_new.Metric < min(strongest_keypoints.Metric));
    uniform_keypoints = non_strong_keypoints.selectUniform(max(nb_new_candidate_keypoints - length(strongest_keypoints), 1), size(I_curr));
    C_new = [strongest_keypoints.Location; uniform_keypoints.Location];
else
    C_new = [];
end

% Add new candidate keypoints to S

S.C = [S.C;
       C_new];

S.F = [S.F;
       C_new];

S.T = cat(3, S.T, repmat(T_WC_curr, 1, 1, size(C_new, 1)));


end
