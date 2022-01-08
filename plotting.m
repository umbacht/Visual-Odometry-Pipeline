function [numMatched3dPoints, xzCoordinates, last20Frameidx] = ...
    plotting(S_crt, T_WC_crt, image_crt, numMatched3dPoints, xzCoordinates, last20Frameidx)
%% Creating Plots of VO

crtIdx = last20Frameidx(end);
T_CW_crt = [T_WC_crt(1:3,1:3)',  -T_WC_crt(1:3,1:3)'*T_WC_crt(1:3,4);
                0,0,0,1];
xzCoordinates(:,crtIdx) = T_WC_crt([1,3],end);
numMatched3dPoints(crtIdx) = size(S_crt.X,1);

% initialize plot and make full screen
figure(2)
set(gcf, 'Units','normalized','Position',[0 0 1 1])

% plot current image with landmarks
subplot(4,5,[1,2,6,7]) % NEED CHANGE LATER, the position of the plot 
imshow(image_crt)
hold on
plot(S_crt.C(:, 1), S_crt.C(:, 2), 'rx', 'MarkerSize',3); % plot candidate keypoints C
plot(S_crt.P(:, 1), S_crt.P(:, 2), 'gx', 'MarkerSize',3); % plot keypoints P
hold off
title('Current Image')

% plot number of matched 3d points over last 20 frames
subplot(4,5,[11,16])
plot(-19:0, numMatched3dPoints(last20Frameidx))
title('# tracked landmarks over last 20 frames')

% plot the full trajectory
subplot(4,5,[12,17])
plot(xzCoordinates(1,19:crtIdx),xzCoordinates(2,19:crtIdx)) %%%Edited
axis equal
title('Full Trajectory')

% plot the trajectory of last 20 frames and landmarks view from above
r1 = subplot(4,5,[3,4,5,8,9,10,13,14,15,18,19,20]);
cla(r1);
hold on
plot(xzCoordinates(1,last20Frameidx),xzCoordinates(2,last20Frameidx),'-x','MarkerSize',3)

plot(S_crt.X(:,1),S_crt.X(:,3),'k.')
hold off
axis equal
title('Trajectory of last 20 frames and landmarks')

% update indices
last20Frameidx = last20Frameidx + 1;

end