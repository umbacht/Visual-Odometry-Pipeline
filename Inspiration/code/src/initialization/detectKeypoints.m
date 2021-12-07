function [kp0, kp1] = detectKeypoints(img0, img1, params)
% Detects harris features in the images
%
% INPUTS:
%             img0    : matrix NxM, first image 
%             img1    : matrix NxM, second image
%             params  : the parameters
% 
% OUTPUTS:
%             kp0   : matrix Kx2, 2D location of keypoints in img0
%             kp1   : matrix Kx2, 2D location of keypoints in img1
%
%% Detect harris features

% With exercise solutions
if params.use_matlab == 0
    
    % Compute harris scores for each pixel 
    % Scores
    harris_scores0 = harris(img1, params.corner_patch_size, params.harris_kappa);
    harris_scores1 = harris(img0, params.corner_patch_size, params.harris_kappa);
    
    % Select keypoints
    % Scores
    kp0 = selectKeypoints(harris_scores0, ...
         params.num_keypoints, params.nonmaximum_supression_radius);
    kp1 = selectKeypoints(harris_scores1, ...
         params.num_keypoints, params.nonmaximum_supression_radius);
     
    % DEBUG: Show keypoints in img0
    if params.init_debug == 1
        figure;
        imshow(img0);
        hold on;
        plot(kp0(2, :), kp0(1, :), 'gx', 'Linewidth', 1);
        title('[detectKeypoints] Harris features of img0, no matlab');
        hold off;
    end
    % DEBUG

end

% With matlab functions
if params.use_matlab == 1
    
    kp0 = detectHarrisFeatures(img0, 'MinQuality', params.feature_quality, ...
                                     'ROI', params.ROI, ...
                                     'FilterSize', params.filt_size);
    kp1 = detectHarrisFeatures(img1, 'MinQuality', params.feature_quality, ...
                                     'ROI', params.ROI, ...
                                     'FilterSize', params.filt_size);

    % Select strongest features
    kp0 = kp0.selectUniform(200, size(img0));
    kp1 = kp1.selectUniform(200, size(img1));

%             % Compute harris scores for each pixel 
%             % Scores
%             harris_scores0 = harris(img1, params.corner_patch_size, params.harris_kappa);
%             harris_scores1 = harris(img0, params.corner_patch_size, params.harris_kappa);
% 
%             % Select keypoints
%             % Scores
%             kp0 = selectKeypoints(harris_scores0, ...
%                  params.num_keypoints, params.nonmaximum_supression_radius);
%             kp1 = selectKeypoints(harris_scores1, ...
%                  params.num_keypoints, params.nonmaximum_supression_radius);
%             kp0 = cornerPoints(kp0');
%             kp1 = cornerPoints(kp1');
 
     
    % DEBUG: Show keypoints in img0
    if params.init_debug == 1
        figure;
        imshow(img0); hold on;
        plot(kp0);
        title('[detectKeypoints] Harris features of img0, with matlab');
        plot( kp0.Location(1,2), kp0.Location(1,1), 'rx');
        hold off;
 
    end
    % DEBUG
   
end

end

