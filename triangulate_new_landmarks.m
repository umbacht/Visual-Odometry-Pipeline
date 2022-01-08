function S_crt = triangulate_new_landmarks(image_crt, image_prv, S_crt, T_WC_crt, parameter)
% Triangulate the 3D coordinates of new landmarks
%
% Input:
% - Previous Image
% - Current Image
% - Previous State
% - Current Camera Pose in World coordinates
% - Paramter
% Output:
% - Current State

%% Track previous candidates in current image
point_tracker = vision.PointTracker('MaxBidirectionalError', parameter.MaxBidirectionalError_triang, ...
                               'NumPyramidLevels', parameter.NumPyramidLevels_triang, ...
                               'BlockSize', parameter.BlockSize_triang, ...
                               'MaxIterations', parameter.MaxIterations_triang);

initialize(point_tracker, S_crt.C, image_prv);
[C_crt, point_validity] = point_tracker(image_crt);
S_crt.C = C_crt(point_validity, :);

S_crt.F = S_crt.F(point_validity, :);
S_crt.T = S_crt.T(:, :, point_validity);

release(point_tracker);

new_index_kp = [];

%% Triangulate valid candidates
for i=1:length(S_crt.C)
    
    camera_ex_C = [T_WC_crt(1:3,1:3)' -T_WC_crt(1:3,1:3)'*T_WC_crt(1:3,4)];
    M_C = parameter.K * camera_ex_C;

    camera_ex_F = [S_crt.T(1:3,1:3,i)' -S_crt.T(1:3,1:3,i)'*S_crt.T(1:3,4,i)];
    M_F = parameter.K * camera_ex_F;

    X = linearTriangulation([S_crt.F(i,:)'; 1], [S_crt.C(i,:)'; 1], M_F, M_C);

    if X(3) < 0
        warning('Point behind camera');
    end
    
    % Calculate Bearing angle:
    a = T_WC_crt(1:3,4)-X(1:3);
    b = S_crt.T(1:3,4,i)-X(1:3);

    angle = acos(a'*b/(norm(a)*norm(b)));
    
    % Only create new landmarks and candidates if bearing angle large
    % enough
    if (angle > parameter.angle_threshold)
        S_crt.X = [S_crt.X; X(1:3)'];
        S_crt.P = [S_crt.P; S_crt.C(i, :)];
        new_index_kp = [new_index_kp, i];
    end
end

% Deleting new added KeyPoints from candidate list
S_crt.C(new_index_kp, :) = [];
S_crt.F(new_index_kp, :) = [];
S_crt.T(:, :, new_index_kp) = [];
end 