function [T_c0_c1, init_kp, init_landmarks] = initialization(img0, img1, params)
% The initialization function takes two images which are then used to
% compute the initial keypoints (on the image plane )with their 
% corresponding landmarks (in the WCS). Also the homogenous transformation
% matrix which transforms points on the img0 into the WCS is computed.
%
%   INPUT:
%       img0            : matrix NxM, first image of data set
%       img1            : matrix NxM, next image (not subsequent to img0)
%       params          : parameter
%
%   OUTPUT:
%       M_w_c1          : matrix 4x4, homogenous transformation matrix from camera
%                                     frame 1 to WCS
%       init_kp         : matrix Kx2, initial keypoints in the image plane on img 1                   
%       init_landmarks  : matrix Kx2, initial landmarks which where triangulated from
%                                     the init_kp

%% Initialization

% detect Keypoints with harris detector
[kp0, kp1] = detectKeypoints(img0, img1, params);

% get descriptor of keypoints
[descr0, kp0_valid, descr1, kp1_valid] = descrKeypoints(img0, img1, kp0, kp1, params);

% match keypoints
[matchedPoints0, matchedPoints1] = matchKeypoints(img0, img1, kp0_valid, kp1_valid, descr0, descr1, params);

% compute relative pose
[rel_orient, rel_loc, inliers_idx] = relativePose2(img0, img1, matchedPoints0, matchedPoints1, params);

% triangulate landmarks with matchedPoints and homog. transform. matrix
% from c1 to WCS
[T_c0_c1, init_landmarks] = triangLandmarks(matchedPoints0(inliers_idx,:), matchedPoints1(inliers_idx,:), rel_orient, rel_loc, params);

% Make transformation matrix homogenous
T_c0_c1 = [T_c0_c1; 0 0 0 1];

% onyl use valid landmarks i.e. z > 0
init_landmarks = init_landmarks (init_landmarks(:,3)>=0, :);

% initial keypoints in frame img1
init_kp = matchedPoints1(init_landmarks(:,3)>=0, :);
% covnert to cornerPoints object to array
init_kp = init_kp.Location;


if params.init_viz == 1

    % Plot matched features
    figure;
    subplot(3,2,1);
    showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
    title('Point matches');
    
    subplot(3,2,2);
    showMatchedFeatures(img0, img1, matchedPoints0(inliers_idx), matchedPoints1(inliers_idx));
    title('Point matches after outliers removed');
   
    % plot point cloud and cameras
    subplot(3,2,3:6)
    hold on
    grid on
    plotCamera('Location',[0 0 0], 'Orientation',eye(3), 'Label','Camera0', 'AxesVisible',true, 'Size',2, 'Color',[0,0,0]);
    plotCamera('Location',T_c0_c1(1:3, 4), 'Orientation',T_c0_c1(1:3, 1:3), 'Label','Camera1', 'Size',2);
    plot([1 2 3], [1 2 3]);
    scatter3(init_landmarks(:,1), init_landmarks(:,2), init_landmarks(:,3), 'filled' )
    %xlim([-5 5]); ylim([-3 3]); zlim([-1 60]);
    xlabel('x'); ylabel('y'); zlabel('z')
    view(0, 0); % view directly from top
    title('Initial 3D Scene, from top view');
    axis equal;
    
end

end
