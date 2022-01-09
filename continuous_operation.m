function [S_crt, T_WC_crt] = continuous_operation(image_crt, image_prv, S_prv, T_WC_prv, parameter)
% Continious VO Operation consisting of:
% - Tracking keypoints in new image
% - Pose Estimation of new image
% - Triangulate new landmarks
% - Find new candidates
%
% Input: 
% - Image_current
% - Image_previous
% - S_previous: State Struct with .P, .X, .C, .F, .T of previous image
% - parameters: Struct with all parameters
%
% Output:
% - S_current: State Struct with .P, .X, .C, .F, .T of current image
% - T_WC_current: Pose of current camera
%
% Nomenclature:
% - current: crt
% - previous: prv
    
%% Match Keypoints from previous and current image with KLT
landmarks_prv = S_prv.X;
keypoints_prv = S_prv.P;

% KLT PointTracker
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

%%
% intrinsics = cameraIntrinsics([parameter.K(1,1),parameter.K(2,2)],parameter.K(1:2,3)',[1226,370]);
% 
% [R_C_W,t_C_W,inliersIndex] = estimateWorldCameraPose(...
%      tracked_keypoints_crt,tracked_landmarks_crt,intrinsics, 'MaxNumTrials',2000, 'Confidence', 90, 'MaxReprojectionError', 1);

% tracked_landmarks_crt = tracked_landmarks_crt(tracked_landmarks_crt(:,3)>0, :);
% tracked_keypoints_crt = tracked_keypoints_crt(tracked_landmarks_crt(:,3)>0, :);

[R_C_W, t_C_W, inliersIndex, max_num_inliers_history, num_iteration_history] = ...
    ransacLocalization(fliplr(tracked_keypoints_crt)', tracked_landmarks_crt', parameter.K);
    
%% Estimate Fundamental Matrix from matching keypoints using RANSAC

% Normalise points
% [tracked_keypoints_prv_norm, T1] = normalise2dpts([tracked_keypoints_prv'; ...
%     ones(1,length(tracked_keypoints_prv))]);
% [tracked_keypoints_crt_norm, T2] = normalise2dpts([tracked_keypoints_crt'; ...
%     ones(1,length(tracked_keypoints_crt))]);

% [F_hat, inliersIndex] = estimateFundamentalMatrix(tracked_keypoints_prv_norm(1:2,:)', ...
%     tracked_keypoints_crt_norm(1:2,:)', 'Method',parameter.method, ...
%     'NumTrials', parameter.NumTrials, ...
%     'DistanceThreshold', parameter.DistanceThreshold);
% F_hat = T2.'* F_hat * T1; 
% E = parameter.K'*F_hat*parameter.K;
% [Rots,u3] = decomposeEssentialMatrix(E);
% 
% % Decompose and disambiguate transformation, and triangulate landmarks.
% p1 = [tracked_keypoints_prv(inliersIndex,:)'; ones(1,length(tracked_keypoints_prv(inliersIndex,:)))];
% p2 = [tracked_keypoints_crt(inliersIndex,:)'; ones(1,length(tracked_keypoints_crt(inliersIndex,:)))];
% [R,t] = disambiguateRelativePose(Rots,u3,p1,p2,parameter.K,parameter.K);

%% Build current state
% Transformation from previous pose to current pose
% T_imgprv_imgcrt = [R', -R'*t];
% T_imgprv_imgcrt = [T_imgprv_imgcrt; 0 0 0 1];
% Camera pose in world coordinates
% T_WC_crt = T_WC_prv * T_imgprv_imgcrt;

T_WC_crt = [R_C_W', -R_C_W'*t_C_W ; 0 0 0 1];
% T_WC_crt = [R_C_W, t_C_W' ; 0 0 0 1];
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

%% New Landmarks triangulation and find new candidates
if size(S_crt.C, 1) > 0 
    S_crt = triangulate_new_landmarks(image_crt, image_prv, S_crt, T_WC_crt, parameter);
end

S_crt = find_new_candidate_kp(image_crt, S_crt, T_WC_crt, parameter);

%% Delete KeyPoints and Landmarks which are behind the Camera or very far away
landmarks_cameraframe = T_WC_crt \ ([S_crt.X, ones(length(S_crt.X),1)]');
z_coordinate = landmarks_cameraframe(3, :);

S_crt.X = S_crt.X(z_coordinate>0 & z_coordinate<parameter.max_distance, 1:3);
S_crt.P = S_crt.P(z_coordinate>0 & z_coordinate<parameter.max_distance, :);

end