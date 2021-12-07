function [rel_orient, rel_loc, inliers_idx] = relativePose2(img0, img1, matchedPoints0, matchedPoints1, params)
% Estiamtes the transformation from camera0 to camera1
% 
%   INPUTS:
%           img0            : matrix NxM, first image
%           img1            : matrix NxM, second image
%           matchedPoints1  : matrix Qx2, keypoints in img1 which were 
%                             matched witch keypoints in img0
%           matchedPoints0  : matrix Qx2, keypoints in img0 which were 
%                             matched witch keypoints in img1
%           params          : the parameters
%
%   OUTPUTS:
%           pose_c1_c0      : marix 4x4, pose of camera 1 w.r.t. camera 0
%           inliers_idx     : vector Kx1, index of inliers RANSAC for estimation of
%                             fundamental matrix F
%


%% Estimate relative Pose

% Estimate fundamental matrix
[F, inliers_idx] = estimateFundamentalMatrix(matchedPoints0, ... 
    matchedPoints1,'Method','RANSAC',...
   'NumTrials',32000,'DistanceThreshold',1e-2,'Confidence', 81);

% extract inlier points from matched points
inliers0 = matchedPoints0(inliers_idx, :);
inliers1 = matchedPoints1(inliers_idx, :);

% DEBUG: show matched keypoints before and after outlier removal
if params.init_debug == 1
    figure;
    showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1, ...
        'montage','PlotOptions',{'ro','go','y--'});
    title('[relativePose2] All point matches');

    figure;
    showMatchedFeatures(img0, img1, inliers0, inliers1, 'montage', ...
        'PlotOptions', {'ro','go','c--'});
    title('[relativePose2] Point matches after outliers were removed');
end
% DEBUG

% Estimate relative camera pose, pose of camera relative to its previous pose
[rel_orient, rel_loc, valid_points_fraction]= relativeCameraPose(F, params.camera, inliers0, inliers1);

if valid_points_fraction<0.7
    warning('[relativePose2]: valid inlier points that project in front of both cameras is low: %f', valid_points_fraction);
end

end

