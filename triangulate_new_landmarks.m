function S = triangulate_new_landmarks(image_crt, image_prv, S_crt, parameter)

% Parameters from exercise 3.
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

num_keypoints = 200;

database_keypoints = [S_crt.P'; S_crt.C']; 

%% Find harris features from current image:
harris_features = harris(image_crt, harris_patch_size, harris_kappa);
keypoints = selectKeypoints(harris_features, num_keypoints, nonmaximum_supression_radius); %non-max supr.
% descriptors = describeKeypoints(image_crt, keypoints, descriptor_radius); %descriptors

is_new_keypoint = logical(ones(length(keypoints), 1));

for i=1:length(keypoints)
    distances = [sqrt(sum((keypoints(i, :) - existing_keypoints).^2, 2))];

    sorted_dists = sort(distances);
    sorted_dists = sorted_dists(sorted_dists~=0);
    min_non_zero_dist = sorted_dists(1);

    if distances <= 4 * min_non_zero_dist
        is_new_keypoints(i) = 0;
    end
end

sorted_dists = sort(dists);
sorted_dists = sorted_dists(sorted_dists~=0);
min_non_zero_dist = sorted_dists(1);

matches(dists >= 4 * min_non_zero_dist) = 0;



end 