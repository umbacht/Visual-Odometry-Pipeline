function [S_crt, T_WC_crt] = continuous_operation(image_crt, image_prv, S_prv, T_WC_prv, parameter)
    % 1. Associate keypoints in the crt frame to prv landmarks
    % 2. Estimate current camera pose
    % 3. Triangulate new landmarks not previously found

%{
S.P = [X,Y] Nx2
S.X = [X,Y,Z] Nx3
S.C = [X,Y] Mx2
S.F = [X,Y] Mx2
S.T 

Plotting: plot(y,x)
%}

%   Input: 
%     Image_current
%     Image_previous
%     S_previous: State Struct with .P, .X, .C, .F, .T of previous image
%     parameters: Struct with Kameraintrinsics in .K

%   Output:
%     S_current: State Struct with .P, .X, .C, .F, .T of current image
%     T_WC_current: Pose of current camera

%   Nomenclature:
%     current: crt
%     previous: prv

%   Todo:
%     PointTracker set values (improve because we only get little matches)
%     estimateFundamentalMatrix set value
%     Check transformation CW or WC
%     Check if E is unique
%     HOMOGENEOUS OR NOT STORING IN .X AND .P

%   Paremeterlist:
%     Kameraintrinsics K
    
%% Match Keypoints from previous and current image with KLT
    landmarks_prv = S_prv.X;
    keypoints_prv = S_prv.P;

    % KLT with Vision Toolbox PointTracker
    pointTracker = vision.PointTracker('MaxBidirectionalError', parameter.MaxBidirectionalError_cont, ...
                                   'NumPyramidLevels', parameter.NumPyramidLevels_cont, ...
                                   'BlockSize', parameter.BlockSize_cont, ...
                                   'MaxIterations', parameter.MaxIterations_cont);
                               
    initialize(pointTracker, keypoints_prv, image_prv);

    [tracked_points,point_validity] = pointTracker(image_crt);
    
    % Get valid tracked points in crt image
    tracked_keypoints_crt = tracked_points(point_validity,:);
    tracked_landmarks_crt = landmarks_prv(point_validity,:);

    % Get valid tracked points in prv image
    tracked_keypoints_prv = keypoints_prv(point_validity,:);
    release(pointTracker);
    
%     disp('Continuous');
%     disp('KP_prv: ' + string(length(keypoints_prv)));
%     disp(sum(point_validity));
%     disp(length(point_validity)-sum(point_validity));
%     a = 7;
        
    %% Estimate Pose from matching keypoints

    [tracked_keypoints_prv_norm, T1] = normalise2dpts([tracked_keypoints_prv'; ones(1,length(tracked_keypoints_prv))]);
    [tracked_keypoints_crt_norm, T2] = normalise2dpts([tracked_keypoints_crt'; ones(1,length(tracked_keypoints_crt))]);

    [F_hat, inliersIndex] = estimateFundamentalMatrix(tracked_keypoints_prv_norm(1:2,:)', tracked_keypoints_crt_norm(1:2,:)', 'Method','RANSAC', 'NumTrials', 2000, 'DistanceThreshold', 1e-4);
    F_hat = T2.'* F_hat * T1;

    % Get E = [R|t] from inliers used to estimate F (RANSAC subset)
    [R,t] = relativeCameraPose(F_hat, cameraParameters('IntrinsicMatrix',parameter.K'), tracked_keypoints_prv(inliersIndex,:), tracked_keypoints_crt(inliersIndex,:));
    % returns the orientation and location of camera 2 relative to camera 1

    % Decompose and disambiguate transformation, and triangulate landmarks.
    if size(R,3)>1
        Roots = R(:,:,1:2);
        u3 = t(1,:)';
        p1 = [tracked_keypoints_prv(inliersIndex,:)'; ones(1,length(tracked_keypoints_prv(inliersIndex,:)))];
        p2 = [tracked_keypoints_crt(inliersIndex,:)'; ones(1,length(tracked_keypoints_crt(inliersIndex,:)))];
        [R,t] = disambiguateRelativePose(Roots,u3,p1,p2,parameter.K,parameter.K);
        t = t';
    end

    % Build new state S_crt and T_crt
    T_imgprv_imgcrt = [R', -R'*t'];
    T_imgprv_imgcrt = [T_imgprv_imgcrt; 0 0 0 1];
    T_WC_crt = T_WC_prv * T_imgprv_imgcrt;
    S_crt.P = tracked_keypoints_crt(inliersIndex,:);
    S_crt.X = tracked_landmarks_crt(inliersIndex,:);

    S_crt.C = S_prv.C;
    S_crt.F = S_prv.F;
    S_crt.T = S_prv.T;

    % Plots for debugging:
    plotting = false;
    if plotting
        showMatchedFeatures(image_prv, image_crt, tracked_keypoints_prv(inliersIndex,:), S_crt.P)
    end

    %% New Landmarks triangulation 
    if size(S_crt.C, 1) > 0 
        S_crt = triangulate_new_landmarks(image_crt, image_prv, S_crt, T_WC_crt, parameter);
    end
    
    S_crt = find_new_candidate_kp(image_crt, S_crt, T_WC_crt, parameter);
   
    %% Delete Old Candidate KeyPoints
%     num_std = 1000;
%     X_c = (inv(T_WC_crt) * [S_crt.X, ones(length(S_crt.P), 1)]')';
%     Z_c = X_c(:, 3);
%     mean_z = mean(Z_c);
%     std_z = std(Z_c);
%     
%     % Delete landmarks behind vehicle
%     S_crt.X = S_crt.X(Z_c > 1 & Z_c < (mean_z + num_std*std_z), 1:3);
%     S_crt.P = S_crt.P(Z_c > 1 & Z_c < (mean_z + num_std*std_z), :);
    
end