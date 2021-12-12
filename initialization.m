function [P1,X1, C1, F1, T1] = initialization(image1, image2, K)

% Parameters from exercise 3.
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 5;

num_keypoints = 1000;

%% Find Harris keypoint patch descriptors for each image
harris1 = harris(image1, harris_patch_size, harris_kappa);
harris2 = harris(image2, harris_patch_size, harris_kappa);
keypoints1 = selectKeypoints(harris1, num_keypoints, nonmaximum_supression_radius);
keypoints2 = selectKeypoints(harris2, num_keypoints, nonmaximum_supression_radius);
descriptors1 = describeKeypoints(image1, keypoints1, descriptor_radius);
descriptors2 = describeKeypoints(image2, keypoints2, descriptor_radius);

%% Match keypoints between images and remove unmatched keypoints
all_matches = matchDescriptors(descriptors1, descriptors2, match_lambda);
%matches(i) gives the descriptor index of descriptors1 which matches to descriptors2(:, i).
%value is zero if descriptor, descriptors2(:, i), has no match.
matched_keypoints2 = keypoints2(:, all_matches > 0);
keypoints1_idx = all_matches(all_matches > 0);
matched_keypoints1 = keypoints1(:, keypoints1_idx);

%% Create homogenous coordinates:
p1 = [matched_keypoints1;ones(1,size(matched_keypoints1,2))];
p2 = [matched_keypoints2;ones(1,size(matched_keypoints2,2))];

%% Estimate Essential matrix. 
%(do we need to normalize vectors?) (we can change some
%other parameters)
[p1_tilda,T1] = normalise2dpts(p1);
[p2_tilda,T2] = normalise2dpts(p2);

[F, inliersIndex] = estimateFundamentalMatrix(p1_tilda(1:2,:)', p2_tilda(1:2,:)', ...
     'Method','RANSAC', 'NumTrials',2000, 'DistanceThreshold',1e-4);
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
unmatched_keypoints2 = keypoints2(:, all_matches < 1);
C1 = unmatched_keypoints2;
F1 = unmatched_keypoints2;
T1 = inv([R_C2_W, T_C2_W; zeros(1,3),1]);


%%


figure(5);
subplot(2, 1, 1);
hold on;
imshow(image1);
plot(keypoints2(2, :), keypoints2(1, :), 'rx', 'Linewidth', 2);
plotMatches(all_matches, keypoints2, keypoints1);

subplot(2, 1, 2);
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
