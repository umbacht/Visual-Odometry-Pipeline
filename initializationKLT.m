function [P1,X1,T1] = initializationKLT(image1, image2, parameter)

% Parameters from main.
K = parameter.K;
harris_patch_size = parameter.harris_patch_size;
harris_kappa = parameter.harris_kappa;
nonmaximum_supression_radius = parameter.nonmaximum_supression_radius;
num_keypoints = parameter.num_keypoints;

%% Find Harris keypoints for first image
harris1 = harris(image1, harris_patch_size, harris_kappa);

% figure(111)
% imagesc(harris1);
% daspect([1 1 1]);


keypoints1 = selectKeypoints(harris1, num_keypoints, nonmaximum_supression_radius);

% figure(222);
% imshow(image1);
% hold on;
% plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);

%% KLT
% KLT with Vision Toolbox PointTracker
    pointTracker = vision.PointTracker('MaxIterations', 30, 'BlockSize', [31,31]);
    initialize(pointTracker, flipud(keypoints1)', image1);

    [tracked_points,point_validity] = pointTracker(image2);
    
% Get valid tracked points in crt image
    tracked_keypoints2 = tracked_points(point_validity,:);

% Get valid tracked points in prv image
    tracked_keypoints1 = flipud(keypoints1(:,point_validity))';
    release(pointTracker);

%% Create homogenous coordinates:
p1 = [tracked_keypoints1';ones(1,size(tracked_keypoints1,1))];
p2 = [tracked_keypoints2';ones(1,size(tracked_keypoints2,1))];
%% Estimate Essential matrix. 
%first normalize vectors 
[p1_tilda,T1] = normalise2dpts(p1);
[p2_tilda,T2] = normalise2dpts(p2);

[F, inliersIndex] = estimateFundamentalMatrix(p1_tilda(1:2,:)', p2_tilda(1:2,:)', ...
     'Method','RANSAC', 'NumTrials',2000, 'DistanceThreshold',1e-4);

inlier_ratio =  nnz(inliersIndex)/size(inliersIndex,1);

F = (T2.') * F * T1;

E = K'*F*K;
%% use only inliers, and create homogenous coordinates:
p1 = p1(:, inliersIndex);
p2 = p2(:, inliersIndex);

%% Decompose and disambiguate transformation, and triangulate landmarks.
[Rots,u3] = decomposeEssentialMatrix(E);

[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
X = linearTriangulation(p1,p2,M1,M2);

%% Initialize state for continuous VO pipeline 
P1 = p2;
X1 = X;
T1 = [R_C2_W', -R_C2_W'*T_C2_W];


%% Plotting for debugging
% figure(69)
% imshow(image2);
% hold on

% %Plot matched keypoints from current and previous image:
% plot(P1(:,1), P1(:,2), 'gx', 'Linewidth', 2);
% tracked_keypoints1 = tracked_keypoints1(inliersIndex,:);
% plot(tracked_keypoints1(:,1),tracked_keypoints1(:,2),'bx','Linewidth',2)
% 
% showMatchedFeatures(image1, image2, ...
%     tracked_keypoints1(inliersIndex,:), tracked_keypoints2(inliersIndex,:));


% figure(6);
% % tiledlayout(1,2,'TileSpacing','compact')
% tiledlayout(2,1)

% nexttile;
% hold on;
% imshow(image1);
% plot(tracked_points(:,1), tracked_points(:,2), 'rx', 'Linewidth', 2);
% 
% x_from = tracked_keypoints2(:, 1)';
% x_to = tracked_keypoints1(:, 1)';
% y_from = tracked_keypoints2(:, 2)';
% y_to = tracked_keypoints1(:, 2)';
% plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
% title('First image showing red tracked points of second image');
% nexttile;
% hold on;
% imshow(image2);
% plot(tracked_keypoints2((1-inliersIndex)>0, 1), ...
%     tracked_keypoints2((1-inliersIndex)>0, 2), 'rx', 'Linewidth', 2);
% plot(tracked_keypoints2((inliersIndex)>0, 1), ...
%     tracked_keypoints2((inliersIndex)>0, 2), 'gx', 'Linewidth', 2);
% x_from = tracked_keypoints2(inliersIndex>0, 1)';
% x_to = tracked_keypoints1(inliersIndex>0, 1)';
% y_from = tracked_keypoints2(inliersIndex>0, 2)';
% y_to = tracked_keypoints1(inliersIndex>0, 2)';
% plot([x_from; x_to], [y_from; y_to], 'g-', 'Linewidth', 3);
% title('Second image showing red (RANSAC) outliers and green inliers');

