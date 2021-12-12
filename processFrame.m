function [S_crt, T_WC_crt] = processFrame(image_crt, image_prv, S_prv, parameters)
    % 1. Associate keypoints in the crt frame to prv landmarks
    % 2. Estimate current camera pose
    % 3. Triangulate new landmarks not previously found

%   Input: 
%     Image_current
%     Image_previous
%     S_previous: State Struct with .P, .X, .C, .F, .T of previous image
%     parameters: Struct with Kameraintrinsics in .K
% 
%   Output:
%     S_current: State Struct with .P, .X, .C, .F, .T of current image
%     T_WC_current: Pose of current camera

%   Nomenclature:
    % current: crt
    % previous: prv

%   Todo:
    % PointTracker set values
    % estimateFundamentalMatrix set value
    % Check transformation CW or WC

%   Paremeterlist:
    % Kameraintrinsics K


    %% Match Keypoints from previous and current image with KLT
    landmarks_prv = S_prv.X;
    keypoints_prv = S_prv.P;
    K = parameters.K;

    % KLT with Vision Toolbox PointTracker
    pointTracker = vision.PointTracker('SET SOME STUFF');
    initialize(pointTracker, keypoints_prv, image_prv);

    [tracked_points,point_validity] = pointTracker(image_crt);
    
    % Get valid tracked points in crt image
    tracked_keypoints_crt = tracked_points(point_validity,:);
    tracked_landmarks_crt = landmarks_prv(point_validity,:);

    % Get valid tracked points in prv image
    tracked_keypoints_prv = keypoints_prv(point_validity,:);
    release(pointTracker);
    

    %% Estimate Pose from matching keypoints

    [F_hat, inliersIndex] = estimateFundamentalMatrix(tracked_keypoints_prv, tracked_keypoints, 'Method', ...
        'RANSAC', 'NumTrials', 2000, 'DistanceThreshold', 1e-4);

    % Get E = [R|t] from inliers used to estimate F (RANSAC subset)
    [R,t] = relativeCameraPose(F_hat, K, tracked_keypoints_prv(inliersIndex,:), tracked_landmarks_crt(inliersIndex,:));
    
    % Build new state S_crt and T_crt
    T_WC_crt = [R, t];
    T_WC_crt = [T_WC_crt; 0 0 0 1];
    T_WC_crt = S_prv.T * T_WC_crt;
    S_crt.P = tracked_keypoints_crt(inliersIndex,:);
    S_crt.X = tracked_landmarks_crt(inliersIndex,:);
    S_crt.C = S_prv.C;
    S_crt.F = S_prv.F;
    S_crt.T = S_prv.T;
    


end