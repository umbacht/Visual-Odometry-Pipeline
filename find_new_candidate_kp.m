function S_crt = find_new_candidate_kp(image_crt, S_crt, T_WC_crt, parameter)

% Function Description

%% Find new candidates (harris features) from current image:
database_keypoints = [S_crt.P; S_crt.C]; 

% Remove candidates in distortion frame
if isfield(parameter, 'distortion_frame')
	image_croped = imcrop(image_crt, parameter.distortion_frame);
	harris_features = harris(image_croped, parameter.harris_patch_size, parameter.harris_kappa);
    harris_features = padarray(harris_features, [parameter.distortion_frame(2),parameter.distortion_frame(1)],0);
else
	harris_features = harris(image_crt, parameter.harris_patch_size, parameter.harris_kappa);
end

assert(min(size(harris_features) == size(harris_features)));

% Selecting KeyPoints 
keypoints = selectKeypoints(harris_features, parameter.num_keypoints, parameter.nonmaximum_supression_radius)'; 
keypoints = flipud(keypoints')';
new_kps = ones(length(keypoints), 1);



% Avoid new keypoints that are too close to already existing keypoints:
min_distances = pdist2(database_keypoints, keypoints,'squaredeuclidean','Smallest',1)';
new_kps(min_distances <= parameter.threshold^2) = 0;
    
C_new = keypoints(logical(new_kps),:);

% Add new candidates to list
S_crt.C = [S_crt.C; C_new];
S_crt.F = [S_crt.F; C_new];

S_crt.T = cat(3, S_crt.T, repmat(T_WC_crt, 1, 1, size(C_new, 1)));

end
    