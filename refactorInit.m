function [S1.P,S1.X,S1.C,S1.F,S1.T] = initialization(image1, image2, parameters.K)

% Parameters from exercise 3.
parameters.harris_patch_size = 9;
parameters.harris_kappa = 0.08;
parameters.nonmaximum_supression_radius = 8;
parameters.descriptor_radius = 9;
parameters.match_lambda = 4;

parameters.num_keypoints = 200;

%% Find Harris keypoint patch descriptors for each image
harris1 = harris(image1, parameters.harris_patch_size, parameters.harris_kappa);
harris2 = harris(image2, parameters.harris_patch_size, parameters.harris_kappa);

figure(111)
subplot(1, 2, 1);
imagesc(harris1);
daspect([1 1 1]);
subplot(1, 2, 2);
imagesc(harris2);
daspect([1 1 1]);

keypoints1 = selectKeypoints(harris1, parameters.num_keypoints, parameters.nonmaximum_supression_radius);
keypoints2 = selectKeypoints(harris2, parameters.num_keypoints, parameters.nonmaximum_supression_radius);

figure(222);
subplot(1, 2, 1);
imshow(image1);
hold on;
plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
subplot(1, 2, 2);
imshow(image2);
hold on;
plot(keypoints2(2, :), keypoints2(1, :), 'rx', 'Linewidth', 2);

descriptors1 = describeKeypoints(image1, keypoints1, parameters.descriptor_radius);
descriptors2 = describeKeypoints(image2, keypoints2, parameters.descriptor_radius);

figure(333);
for i = 1:16
    subplot(4, 4, i);
    patch_size = 2 * parameters.descriptor_radius + 1;
    imagesc(uint8(reshape(descriptors2(:,i), [patch_size patch_size])));
    axis equal;
    axis off;
end

%% Match keypoints between images and remove unmatched keypoints
all_matches = matchDescriptors(descriptors2, descriptors1, parameters.match_lambda);
%matches(i) gives the descriptor index of descriptors1 which matches to descriptors2(:, i).
%value is zero if descriptor, descriptors2(:, i), has no match.
figure(444);
imshow(image2);
hold on;
plot(keypoints2(2, :), keypoints2(1, :), 'rx', 'Linewidth', 2);
plotMatches(all_matches, keypoints2, keypoints1);

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
E = parameters.K'*F*parameters.K;

%% use only inliers, and create homogenous coordinates:
p1 = p1(:, inliersIndex);
p2 = p2(:, inliersIndex);

%% Decompose and disambiguate transformation, and triangulate landmarks.
[Rots,u3] = decomposeEssentialMatrix(E);

[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,parameters.K,parameters.K);
M1 = parameters.K * eye(3,4);
M2 = parameters.K * [R_C2_W, T_C2_W];
X = linearTriangulation(p1,p2,M1,M2);

%% Initialize state for continuous VO pipeline 
S1.P = p2;
S1.X = X;
unmatched_keypoints2 = keypoints2(:, all_matches < 1);
S1.C = unmatched_keypoints2;
S1.F = unmatched_keypoints2;
S1.T = [R_C2_W', -R_C2_W'*T_C2_W];


%%


figure(5);
subplot(1, 2, 1);
hold on;
imshow(image1);
plot(keypoints2(2, :), keypoints2(1, :), 'rx', 'Linewidth', 2);
plotMatches(all_matches, keypoints2, keypoints1);

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
