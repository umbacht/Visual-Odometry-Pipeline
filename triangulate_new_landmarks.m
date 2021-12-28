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

%% Parameters from exercise 3.
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

num_keypoints = 200;

threshold = 50;

%% Triangulation and tracking of previous candidate keypoints
if size(S_crt.C, 1) > 0 
    % track previous candidates in current image
    point_tracker = vision.PointTracker('MaxBidirectionalError', 0.8, ...
                                   'NumPyramidLevels', 6, ...
                                   'BlockSize', [21 21], ...
                                   'MaxIterations', 40);
    initialize(point_tracker, S_crt.C, image_prv);
    % find candidate points
    [C_crt, is_valid] = point_tracker(image_crt);

    S_crt.C = C_crt(is_valid, :);
    S_crt.F = S_crt.F(is_valid, :);
    S_crt.T = S_crt.T(:, :, is_valid);
    release(point_tracker);


    new_keypoint_ids = [];

    % Triangulate candidates
    for i=1:length(S_crt.C)
        
        camera_ex_C = [T_WC_crt(1:3,1:3).' T_WC_crt(1:3,4)];
        M_C = parameter.K * camera_ex_C;

        camera_ex_F = [S_crt.T(1:3,1:3, i) S_crt.T(1:3,4,i)];
        M_F = parameter.K * camera_ex_F;

        X = linearTriangulation([S_crt.C(i,:)'; 1], [S_crt.F(i,:)'; 1] , M_C, M_F);

        
        % Bearing angle:
        a = X(1:3) - T_WC_crt(1:3,4);
        b = X(1:3) - S_crt.T(1:3,4,i);

        angle = acos(a'*b/(norm(a)*norm(b)));
        if (angle >2/180*pi)
            S_crt.X = [S_crt.X; X(1:3)'];
            S_crt.P = [S_crt.P; S_crt.C(i, :)];

            % delete candidate
            new_keypoint_ids = [new_keypoint_ids, i];

        end


    % delete new keypoints from candidates list
%     S_crt.C(new_keypoint_ids, :) = [];
%     S_crt.F(new_keypoint_ids, :) = [];
%     S_crt.T(:, :, new_keypoint_ids) = [];

    end    
    
end
    
database_keypoints = [S_crt.P; S_crt.C]; 

%% Find new candidates (harris features) from current image:
harris_features = harris(image_crt, harris_patch_size, harris_kappa);
keypoints = selectKeypoints(harris_features, num_keypoints, nonmaximum_supression_radius)'; %non-max supr. returns [X,Y]

keypoints = flipud(keypoints')';

is_new_keypoints = ones(length(keypoints), 1);

for i = 1:length(keypoints)
    distances = [sqrt(sum((keypoints(i, :) - database_keypoints).^2, 2))];
    sorted_dists = sort(distances);
    sorted_dists = sorted_dists(sorted_dists~=0);
    min_non_zero_dist = sorted_dists(1);
    
    if min_non_zero_dist <= threshold % if ssd closer than threshold we discard keypoint
        is_new_keypoints(i) = 0;
    end 
end

   
% Taking only the keypoints which fullfill a certain condition
C_new = keypoints(logical(is_new_keypoints),:);
S_crt.C = [S_crt.C; C_new];

S_crt.T = cat(3, S_crt.T, repmat(T_WC_crt, 1, 1, size(C_new, 1)));
S_crt.F = [S_crt.F; C_new];

end 