function S_crt = find_new_candidate_kp(image_crt, image_prv, S_crt, T_WC_crt, parameter)

% Function Description

    %% Find new candidates (harris features) from current image:
    database_keypoints = [S_crt.P; S_crt.C]; 

    harris_features = harris(image_crt, harris_patch_size, harris_kappa);
    assert(min(size(harris_features) == size(harris_features)));

    % Selecting KeyPoints 
    keypoints = selectKeypoints(harris_features, parameter.num_keypoints, parameter.nonmaximum_supression_radius)'; 
    keypoints = flipud(keypoints')';
    new_kps = ones(length(keypoints), 1);

    for i = 1:length(keypoints)
        distances = [sqrt(sum((keypoints(i, :) - database_keypoints).^2, 2))];

        sorted_dists = sort(distances);
        min_non_zero_dist = sorted_dists(1);    

        % if ssd closer than threshold we discard keypoint
        if min_non_zero_dist <= parameter.threshold 
            new_kps(i) = 0;
        end   
    end

    C_new = keypoints(logical(new_kps),:);

    % Taking only the keypoints which fullfill a certain condition
    S_crt.C = [S_crt.C; C_new];
    S_crt.F = [S_crt.F; C_new];

    S_crt.T = cat(3, S_crt.T, repmat(T_WC_crt, 1, 1, size(C_new, 1)));
end
    