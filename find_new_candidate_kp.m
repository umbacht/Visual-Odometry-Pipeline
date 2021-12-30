function S_crt = find_new_candidate_kp(image_crt, S_crt, T_WC_crt, parameter)

% Function Description

    %% Find new candidates (harris features) from current image:
    database_keypoints = [S_crt.P; S_crt.C]; 

    harris_features = harris(image_crt, parameter.harris_patch_size, parameter.harris_kappa);
    assert(min(size(harris_features) == size(harris_features)));

    % Selecting KeyPoints 
    keypoints = selectKeypoints(harris_features, parameter.num_keypoints, parameter.nonmaximum_supression_radius)'; 
    
    
    keypoints = flipud(keypoints')';
    new_kps = ones(length(keypoints), 1);
    
    %avoid new keypoints that are too close to already existing keypoints:
    min_distances = pdist2(database_keypoints, keypoints,'squaredeuclidean','Smallest',1)';
    new_kps(min_distances <= parameter.threshold^2) = 0;
    
%     for i = 1:length(keypoints) %%% THis for loop can be replaced by pdist2()
%         distances = [sqrt(sum((keypoints(i, :) - database_keypoints).^2, 2))];
% 
%         sorted_dists = sort(distances);
%         min_non_zero_dist = sorted_dists(1);    
% 
%         % if ssd closer than threshold we discard keypoint
%         if min_non_zero_dist <= parameter.threshold 
%             new_kps(i) = 0;
%         end   
%     end
    
    C_new = keypoints(logical(new_kps),:);

    % Taking only the keypoints which fullfill a certain condition
    S_crt.C = [S_crt.C; C_new];
    S_crt.F = [S_crt.F; C_new];

    S_crt.T = cat(3, S_crt.T, repmat(T_WC_crt, 1, 1, size(C_new, 1)));
    
%     figure(123)
%     subplot(2,1,1);
%     imshow(image_crt);
%     hold on;
%     plot(C_new(:,1), C_new(:,2), 'bx', 'Linewidth', 2);
%     subplot(2,1,2);
%     imshow(image_crt);
%     hold on;
%     plot(S_crt.C(:,1), S_crt.C(:,2), 'rx', 'Linewidth', 2);
%     hold off;
%     a = 4;
    
    
end
    