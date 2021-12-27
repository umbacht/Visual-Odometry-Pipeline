function [S_crt, T_WC_crt] = continuous_operation(image_crt, image_prv, S_prv, parameter)
    % 1. Associate keypoints in the crt frame to prv landmarks
    % 2. Estimate current camera pose
    % 3. Triangulate new landmarks not previously found

%{
S.P = [X,Y] 2xN
S.X = [X,Y,Z] 3xN
S.C = [X,Y] 2xM
S.F = [X,Y] 2xM
S.T = [R|t] 4x4

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
    landmarks_prv = S_prv.X(1:3,:)';
    keypoints_prv = flipud(S_prv.P(1:2,:));
    keypoints_prv = keypoints_prv';

    % KLT with Vision Toolbox PointTracker
    pointTracker = vision.PointTracker('MaxBidirectionalError', 0.8, ...
                                   'NumPyramidLevels', 6, ...
                                   'BlockSize', [21 21], ...
                                   'MaxIterations', 40);
    initialize(pointTracker, keypoints_prv, image_prv);

    [tracked_points,point_validity] = pointTracker(image_crt);
    
    % Get valid tracked points in crt image
    tracked_keypoints_crt = tracked_points(point_validity,:);
    tracked_landmarks_crt = landmarks_prv(point_validity,:);

    % Get valid tracked points in prv image
    tracked_keypoints_prv = keypoints_prv(point_validity,:);
    release(pointTracker);
    
    %% Estimate Pose from matching keypoints

    [tracked_keypoints_prv_norm, T1] = normalise2dpts([tracked_keypoints_prv'; ones(1,length(tracked_keypoints_prv))]);
    [tracked_keypoints_crt_norm, T2] = normalise2dpts([tracked_keypoints_crt'; ones(1,length(tracked_keypoints_crt))]);

    [F_hat, inliersIndex] = estimateFundamentalMatrix(tracked_keypoints_prv_norm(1:2,:)', tracked_keypoints_crt_norm(1:2,:)', 'Method','RANSAC', 'NumTrials', 2000, 'DistanceThreshold', 1e-4);
    F_hat = T2.'*F_hat*T1;

    % Get E = [R|t] from inliers used to estimate F (RANSAC subset)
    [R,t] = relativeCameraPose(F_hat, cameraParameters('IntrinsicMatrix',parameter.K'), tracked_keypoints_prv(inliersIndex,:), tracked_keypoints_crt(inliersIndex,:));
    % returns the orientation and location of camera 2 relative to camera 1

    % Decompose and disambiguate transformation, and triangulate landmarks.
    if size(R,3)>1
        Roots = R(:,:,1:2);
        u3 = t(1,:)';
        p1 = [tracked_keypoints_prv(inliersIndex,:)'; ones(1,length(tracked_keypoints_prv(inliersIndex,:)))];
        p2 = [tracked_keypoints_crt(inliersIndex,:)'; ones(1,length(tracked_keypoints_crt(inliersIndex,:)))];
        [R,t] = disambiguateRelativePose(Roots,u3,p1,p2,K,K);
    end

    % Build new state S_crt and T_crt
    T_imgprv_imgcrt = [R, t'];
    T_imgprv_imgcrt = [T_imgprv_imgcrt; 0 0 0 1];
    T_WC_crt = S_prv.T * T_imgprv_imgcrt;
    S_crt.P = tracked_keypoints_crt(inliersIndex,:);
    S_crt.X = tracked_landmarks_crt(inliersIndex,:);

    S_crt.C = S_prv.C;
    S_crt.F = S_prv.F;
    S_crt.T = S_prv.T;

    % Plots for debugging:
    plotting = false;
    if plotting      
        showMatchedFeatures(image_prv, image_crt, tracked_keypoints_prv, S_crt.P)
%         imshow(image_crt);
%         hold on
%         %Plot matched keypoints from current and previous image:
%         plot(S_crt.P(:,2), S_crt.P(:,1), 'gx', 'Linewidth', 2);
%         tracked_keypoints_prv = tracked_keypoints_prv(inliersIndex,:);
%         plot(tracked_keypoints_prv(:,1),tracked_keypoints_prv(:,2),'bx','Linewidth',2)
%     
%         x_from = tracked_keypoints_prv(:,1)';
%         x_to = S_crt.P(:,1)';
%         y_from = tracked_keypoints_prv(:,2)';
%         y_to = S_crt.P(:,2)';
%         plot([y_from; y_to], [x_from; x_to], 'g-', 'Linewidth', 3);
% 
%         %plot lossed keypoints
%         not_matched_keypoints_prv = keypoints_prv(~point_validity,:);
%         plot(not_matched_keypoints_prv(:,2),not_matched_keypoints_prv(:,1),'rx','LineWidth',2);
%         hold off
    end

    %% Triangulation of new landmarks

    S_crt = triangulate_new_landmarks(image_crt, image_prv, S_crt, parameter);
    




    
end