function [P1,X1, C1, F1, T1] = initialization(image1, image2, K)

% Parameters from exercise 3.
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

num_keypoints = 200;

%% Find Harris keypoint patch descriptors for each image
harris1 = harris(image1, harris_patch_size, harris_kappa);
harris2 = harris(image2, harris_patch_size, harris_kappa);

figure(111)
subplot(1, 2, 1);
imagesc(harris1);
daspect([1 1 1]);
subplot(1, 2, 2);
imagesc(harris2);
daspect([1 1 1]);

keypoints1 = selectKeypoints(harris1, num_keypoints, nonmaximum_supression_radius);
keypoints2 = selectKeypoints(harris2, num_keypoints, nonmaximum_supression_radius);

figure(222);
subplot(1, 2, 1);
imshow(image1);
hold on;
plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
subplot(1, 2, 2);
imshow(image2);
hold on;
plot(keypoints2(2, :), keypoints2(1, :), 'rx', 'Linewidth', 2);

descriptors1 = describeKeypoints(image1, keypoints1, descriptor_radius);
descriptors2 = describeKeypoints(image2, keypoints2, descriptor_radius);

figure(333);
for i = 1:16
    subplot(4, 4, i);
    patch_size = 2 * descriptor_radius + 1;
    imagesc(uint8(reshape(descriptors2(:,i), [patch_size patch_size])));
    axis equal;
    axis off;
end

%% KLT
% KLT with Vision Toolbox PointTracker
    pointTracker = vision.PointTracker();
    initialize(pointTracker, keypoints1', image1);

    [tracked_points,point_validity] = pointTracker(image2);
    
% Get valid tracked points in crt image
    tracked_keypoints2 = tracked_points(point_validity,:);
%     tracked_landmarks2 = landmarks_prv(point_validity,:);

% Get valid tracked points in prv image
    tracked_keypoints1 = keypoints1(:,point_validity)';
    release(pointTracker);

%% Create homogenous coordinates:
p1 = [tracked_keypoints1';ones(1,size(tracked_keypoints1,1))];
p2 = [tracked_keypoints2';ones(1,size(tracked_keypoints2,1))];
%% Estimate Essential matrix. 
%(do we need to normalize vectors?) (we can change some
%other parameters)
[p1_tilda,T1] = normalise2dpts(p1);
[p2_tilda,T2] = normalise2dpts(p2);

[F, inliersIndex] = estimateFundamentalMatrix(p1_tilda(1:2,:)', p2_tilda(1:2,:)', ...
     'Method','RANSAC', 'NumTrials',2000, 'DistanceThreshold',1e-4);

inlier_ratio =  nnz(inliersIndex)/size(inliersIndex,1)
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
unmatched_keypoints2 = keypoints2(:, point_validity < 1);
C1 = unmatched_keypoints2;
F1 = unmatched_keypoints2;
T1 = [R_C2_W', -R_C2_W'*T_C2_W];


%%


figure(5);
subplot(1, 2, 1);
hold on;
imshow(image1);
plot(keypoints2(2, :), keypoints2(1, :), 'rx', 'Linewidth', 2);
plotMatches(point_validity, keypoints2, keypoints1);

subplot(1, 2, 2);
hold on;
imshow(image2);
plot(matched_keypoints2(2, (1-inliersIndex)>0), ...
    matched_keypoints2(1, (1-inliersIndex)>0), 'rx', 'Linewidth', 2);
plot(matched_keypoints2(2, (inliersIndex)>0), ...
    matched_keypoints2(1, (inliersIndex)>0), 'gx', 'Linewidth', 2);
plotMatches(keypoints1_idx(inliersIndex>0), ...
    matched_keypoints2(:, inliersIndex>0), ...
    keypoints1);
% plot(p2(2,:), p2(1,:), 'rx', 'Linewidth', 2);
% plotMatches(1:size(p2,2),p2(1:2,:),p1(1:2,:));
