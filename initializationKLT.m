function [P1,X1,T1] = initializationKLT(init_frames, parameter)
% Establish initial pose of the camera from initial frames using Harris and
% KLT PointTracker. First image becomes world coordinate frame
%
% Input: 
% - Initial frames, 
% - Parameters
% Output: 
% - Initial 3D Landmarks 4xN (Homogenous Coordinates [X,Y,Z,1]
% - Corresponding 2D Keypoints 3xN (Homogeneous Coordinates [X,Y,1])
 
%% Find Harris keypoints in first image
image1 = init_frames{1};

% Ignoring outer boundary from undistortion for own dataset
if isfield(parameter, 'distortion_frame')
	image_croped = imcrop(image1, parameter.distortion_frame);
	harris1 = harris(image_croped, parameter.harris_patch_size, ...
        parameter.harris_kappa);
    harris1 = padarray(harris1, [parameter.distortion_frame(2), ...
        parameter.distortion_frame(1)],0);
else
	harris1 = harris(image1, parameter.harris_patch_size, parameter.harris_kappa);
end

keypoints1 = selectKeypoints(harris1, parameter.num_keypoints, ...
    parameter.nonmaximum_supression_radius);

%% KLT PointTracker
% Tracking Harris corners through images

pointTracker = vision.PointTracker('MaxIterations', ...
    parameter.MaxIterations_init, 'BlockSize', parameter.BlockSize_init, ...
    'MaxBidirectionalError', parameter.MaxBidirectionalError_init);
    
frame1 = image1;
kpts1 = flipud(keypoints1)';
for i = 2:(size(init_frames,1)-1)
    frame2 = init_frames{i};

    initialize(pointTracker, kpts1, frame1);
    [tracked_points,point_validity] = pointTracker(frame2);
    nnz(point_validity)
    % Get valid tracked points in frame i
    tracked_keypoints2 = tracked_points(point_validity,:);

    % Get valid tracked points in initial frame
    if i==2
        tracked_keypoints1 = kpts1(point_validity,:);
    else
        tracked_keypoints1 = tracked_keypoints1(point_validity,:);
    end
    frame1 = frame2;
    kpts1 = tracked_keypoints2;

    release(pointTracker);
end
    
%% Create homogenous coordinates:
p1 = [tracked_keypoints1';ones(1,size(tracked_keypoints1,1))];
p2 = [tracked_keypoints2';ones(1,size(tracked_keypoints2,1))];

%% Estimate Fundamental matrix and derrive Essential Matrix (RANSAC)
% Normalize points
[p1_tilda,T1] = normalise2dpts(p1);
[p2_tilda,T2] = normalise2dpts(p2);

% 
[F, inliersIndex] = estimateFundamentalMatrix(p1_tilda(1:2,:)', p2_tilda(1:2,:)', ...
     'Method',parameter.method, 'NumTrials',parameter.NumTrials, 'DistanceThreshold',parameter.DistanceThreshold);

inlier_ratio =  nnz(inliersIndex)/size(inliersIndex,1);

F = (T2.') * F * T1;

E = parameter.K'*F*parameter.K;
%% Discard outliers:
p1 = p1(:, inliersIndex);
p2 = p2(:, inliersIndex);

%% Decompose and disambiguate transformation, and triangulate landmarks.
[Rots,u3] = decomposeEssentialMatrix(E);

[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,parameter.K,parameter.K);
M1 = parameter.K * eye(3,4);
M2 = parameter.K * [R_C2_W, T_C2_W];
X = linearTriangulation(p1,p2,M1,M2);

%% Initial State
P1 = p2;
X1 = X;
T1 = [R_C2_W', -R_C2_W'*T_C2_W];


%% Plotting for debugging
%{
figure(69)
imshow(image2);
hold on

%Plot matched keypoints from current and previous image:
plot(P1(:,1), P1(:,2), 'gx', 'Linewidth', 2);
tracked_keypoints1 = tracked_keypoints1(inliersIndex,:);
plot(tracked_keypoints1(:,1),tracked_keypoints1(:,2),'bx','Linewidth',2)

showMatchedFeatures(image1, image2, ...
    tracked_keypoints1(inliersIndex,:), tracked_keypoints2(inliersIndex,:));


figure(6);
% tiledlayout(1,2,'TileSpacing','compact')
tiledlayout(2,1)

nexttile;
hold on;
imshow(image1);
plot(tracked_points(:,1), tracked_points(:,2), 'rx', 'Linewidth', 2);

x_from = tracked_keypoints2(:, 1)';
x_to = tracked_keypoints1(:, 1)';
y_from = tracked_keypoints2(:, 2)';
y_to = tracked_keypoints1(:, 2)';
plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
title('First image showing red tracked points of second image');
nexttile;
hold on;
imshow(image2);
plot(tracked_keypoints2((1-inliersIndex)>0, 1), ...
    tracked_keypoints2((1-inliersIndex)>0, 2), 'rx', 'Linewidth', 2);
plot(tracked_keypoints2((inliersIndex)>0, 1), ...
    tracked_keypoints2((inliersIndex)>0, 2), 'gx', 'Linewidth', 2);
x_from = tracked_keypoints2(inliersIndex>0, 1)';
x_to = tracked_keypoints1(inliersIndex>0, 1)';
y_from = tracked_keypoints2(inliersIndex>0, 2)';
y_to = tracked_keypoints1(inliersIndex>0, 2)';
plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
title('Second image showing red (RANSAC) outliers and green inliers');
%}
