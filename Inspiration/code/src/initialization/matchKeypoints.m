function [matchedPoints0,matchedPoints1] = matchKeypoints(img0, img1, kp0_valid, kp1_valid, descr0, descr1, params)
% Matches the keypoints found in img0 with img0 by either patch matching or
% KLT
%
%   INPUTS:
%           img0        : matrix NxM, first image
%           img1        : matrix NxM, second image
%           kp0_valid   : matrix Px2, keypoints for which a descriptor was
%                                     found
%           kp1_valid   : matrix Px2, keypoints for which a descriptor was
%                                     found
%           descr0      : matrix LxL, descriptor for kp0 in img0  
%           descr1      : matrix LxL, descriptor for kp1 in img1
%
%   OUTPUTS:
%            matchedPoints1 : matrix Qx2, keypoints in img1 which were 
%                             matched witch keypoints in img0
%            matchedPoints0 : matrix Qx2, keypoints in img0 which were 
%                             matched witch keypoints in img1

%% Match keypoints

if strcmp(params.matching_mode, 'klt') 
    
    pointTracker = vision.PointTracker('MaxBidirectionalError', params.max_bilinear_error_klt, ...
                                   'NumPyramidLevels', params.num_pyramid_levels, ...
                                   'BlockSize', params.block_size, ...
                                   'MaxIterations', params.max_iteration);

    if params.use_matlab == 1
        
        initialize(pointTracker,kp0_valid.Location,img0);
        [kp1_valid_points, is_valid]=pointTracker(img1);
        % Save the points in a cornerPoints object
        kp1_valid = cornerPoints(kp1_valid_points);
        % Filter out invalid points
        matchedPoints0 = kp0_valid(is_valid, :);
        matchedPoints1 = kp1_valid(is_valid, :);

    end

    if params.use_matlab == 0 % separate case necessary because of different objects and size of vectors
        
        kp0_valid = fliplr(kp0_valid'); % make it Mx2 and [x y]
                
        initialize(pointTracker, kp0_valid, img0);
        [kp1_valid_points, is_valid]=pointTracker(img1);
        kp1_valid = kp1_valid_points;
        % Filter out invalid points
        matchedPoints0 = kp0_valid(is_valid, :);
        matchedPoints1 = kp1_valid(is_valid, :);
        
    end
    
    % DEBUG: Display the detected points
    if params.init_debug == 1
        figure; 
        showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
        title('[matchKeypoints] matched keypoints with klt');
    end
    % DEBUG

end

if strcmp(params.matching_mode, 'patch_matching')
    
    if params.use_matlab == 0
        % Param
        match_lambda = params.match_lambda;
        % Match descriptors, 
        % entry of matches are 1 if resppective kp got mached and 0 if not
        matches = matchDescriptors(descr1, descr0, match_lambda);
        [~,matchedPoints1_idx,matchedPoints0_idx]=find(matches);
        % select keypoints which were matched
        matchedPoints0 = kp0_valid(:,matchedPoints0_idx);
        matchedPoints1 = kp1_valid(:,matchedPoints1_idx);
        % make vector Mx2 and x as first col and y as second col
        matchedPoints0 = fliplr(matchedPoints0');
        matchedPoints1 = fliplr(matchedPoints1');
       

        % DEBUG: Show Matches
        if params.init_debug == 1
            figure; 
            showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
            title('[matchKeypoints] Matched keypoints no matlab');   
        end
        % DEBUG
    
    elseif params.use_matlab == 1
        % matches descriptors, unique: true no multiple matches,
        % metrics: SSD (default)
        matches = matchFeatures(descr0, descr1,'Unique', true);
        matchedPoints0 = kp0_valid(matches(:,1),:);
        matchedPoints1 = kp1_valid(matches(:,2),:);
        
        % DEBUG: Show matched keypoints
        if params.init_debug == 1
            figure; 
            showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
            title('[matchKeypoints] Matched keypoints with matlab');
        end
        % DEBUG
    end

end
end

