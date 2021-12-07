function [S_curr, T_W_C_1] = ...
    estimatePose(I_1, I_0, S_prev, params, T_W_C_0)

    % This function associates keypoints to existing landmarks
    % and estimates the current pose
    
    % Inputs:   
    %           I_1         :       image of current frame
    %           I_0         :       image of previous frame
    %           kp_0        :       keypoints of previous frame
    %           landmarks_0 :       landmarks of previous frame
    %           params      :       Paramteres
    %           T_W_C_0     :       estiamted pose of previous frame
    % Outputs:   
    %           kp_1        :       keypoints of current frame
    %           landmarks_1 :       landmark of current frame
    %           T_W_C_1     :       estiamted pose of current frame


% 
%% Track Keypoints

kp_0 = S_prev.P;
landmarks_0 = S_prev.X;

pointTracker = vision.PointTracker('MaxBidirectionalError', params.cont_max_bilinear_error_klt, ...
                                   'NumPyramidLevels', params.cont_num_pyramid_levels, ...
                                   'BlockSize', params.cont_block_size, ...
                                   'MaxIterations', params.cont_max_iteration);
                               
initialize(pointTracker, kp_0, I_0);     

[kp_1_valid, isValid] = pointTracker(I_1);
matched_points_1 = kp_1_valid(isValid,:);             
landmarks_1_1 = landmarks_0(isValid,:); 
matched_points_0 = kp_0(isValid,:);

release(pointTracker)



% Global Method 
% % [R, t, isInlier] = ...
% %     estimateWorldCameraPose(matched_points_1, landmarks_1_1, params.camera, ...
% %                                 'MaxNumTrials', params.cont_num_trials, ...
% %                                 'Confidence', params.cont_confidence, ...
% %                                 'MaxReprojectionError', params.cont_max_reprojection_error);

% exercise 7 method
% [R, t, isInlier, max_num_inliers_history, ~] = ...
%         ransacLocalization(matched_points_1.', landmarks_1_1.', params.camera_K);



[F, isInlier] = estimateFundamentalMatrix(matched_points_0, ... 
    matched_points_1,'Method','RANSAC',...
   'NumTrials',params.cont_num_trials,'DistanceThreshold',params.cont_distance_treshold,'Confidence', params.cont_confidence);

inliers_1 = matched_points_1(isInlier,:);
inliers_0 = matched_points_0(isInlier,:);

[rel_orient, rel_loc, valid_points_fraction]= relativeCameraPose(F, params.camera, inliers_0, inliers_1);

if size(rel_loc, 1) > 2
    disp("Multiple solutions found in etimatePose from relativeCameraPose Method");
    
end

T_W_C_rel = [rel_orient(:, :, 1)', rel_loc(1, :).';0 0 0 1];                      


landmarks_1 = landmarks_1_1(isInlier,:);

S_curr.P = inliers_1;
S_curr.X = landmarks_1;
S_curr.F = S_prev.F;
S_curr.C = S_prev.C;
S_curr.T = S_prev.T;



%T_W_C_1 = [R, t.';0 0 0 1];

T_W_C_1 = T_W_C_0 * T_W_C_rel;
if size(rel_loc, 1) > 2
    T_W_C_1 = T_W_C_0;
end


% Set y translation to zero, vehicle is in a plane
%T_W_C_1(2,4) = 0;


end