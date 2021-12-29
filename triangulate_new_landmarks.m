function S_crt = triangulate_new_landmarks(image_crt, image_prv, S_crt, T_WC_crt, parameter)


%% Outline:
%{
1. -Find harris candidates
2. -Pointtracker
3. -Append S.T & S.F
4. Check if valid keypoints
5. if valid: triangulate new landmarks
6. Delete old/obsolete landmarks
%}

%% Previois cancidate KeyPoints Triangulation & Tracking
% track previous candidates in the current image
point_tracker = vision.PointTracker('MaxBidirectionalError', parameter.MaxBidirectionalError_triang, ...
                               'NumPyramidLevels', parameter.NumPyramidLevels_triang, ...
                               'BlockSize', parameter.BlockSize_triang, ...
                               'MaxIterations', parameter.MaxIterations_triang);

initialize(point_tracker, S_crt.C, image_prv);
% find candidate points
[C_crt, point_validity] = point_tracker(image_crt);
S_crt.C = C_crt(point_validity, :);

S_crt.F = S_crt.F(point_validity, :);
S_crt.T = S_crt.T(:, :, point_validity);

release(point_tracker);

% disp('Triang');
% disp(sum(point_validity));
% disp(length(point_validity)-sum(point_validity));
% a = 6;

new_index_kp = [];
% Triangulate candidates
for i=1:length(S_crt.C)
    
    camera_ex_C = [T_WC_crt(1:3,1:3) T_WC_crt(1:3,4)];
    M_C = parameter.K * camera_ex_C;

    camera_ex_F = [S_crt.T(1:3,1:3,i) S_crt.T(1:3,4,i)];
    M_F = parameter.K * camera_ex_F;

    X = linearTriangulation([S_crt.C(i,:)'; 1], [S_crt.F(i,:)'; 1] , M_C, M_F);
    
    % Bearing angle:
    a = T_WC_crt(1:3,4)-X(1:3);
    b = S_crt.T(1:3,4,i)-X(1:3);

    angle = acos(a'*b/(norm(a)*norm(b)));
    if (angle > parameter.angle_threshold)
        S_crt.X = [S_crt.X; X(1:3)'];
        S_crt.P = [S_crt.P; S_crt.C(i, :)];
        % 
        new_index_kp = [new_index_kp, i];
    end
end

% Deleting new KeyPoints from candidate list
S_crt.C(new_index_kp, :) = [];
S_crt.F(new_index_kp, :) = [];
S_crt.T(:, :, new_index_kp) = [];
end 