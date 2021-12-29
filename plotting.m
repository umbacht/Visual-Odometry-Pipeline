function [numMatched3dPoints, xzCoordinates, last20Frameidx] = ...
    plotting(S_crt, T_WC_crt, image_crt, numMatched3dPoints, xzCoordinates, last20Frameidx)

crtIdx = last20Frameidx(end);
T_CW_crt = [T_WC_crt(1:3,1:3)',  -T_WC_crt(1:3,1:3)'*T_WC_crt(1:3,4);
                0,0,0,1];
xzCoordinatesCW(:,crtIdx) = T_CW_crt([1,3],end);
% xzCoordinatesWC(:,crtIdx) = T_WC_crt([1,3],end);
numMatched3dPoints(crtIdx) = size(S_crt.X,1);

figure(2)

% plot current image with landmarks
subplot(4,4,[1,2,5,6]) % NEED CHANGE LATER, the position of the plot 
imshow(image_crt)
hold on
plot(S_crt.C(:, 1), S_crt.C(:, 2), 'rx'); % plot candidate keypoints C
plot(S_crt.P(:, 1), S_crt.P(:, 2), 'gx'); % plot keypoints P
hold off
title('Current Image')

% plot number of matched 3d points over last 20 frames
subplot(4,4,[9,13])
plot(-19:0, numMatched3dPoints(last20Frameidx))
title('The number of matched 3d points over last 20 frames')

% plot the full trajectory
subplot(4,4,[10,14])
plot(xzCoordinatesCW(1,19:crtIdx),xzCoordinatesCW(2,19:crtIdx))
axis equal
title('The Full Trajectory')

% plot the trajectory of last 20 frames and landmarks view from above
r1 = subplot(4,4,[3,4,7,8,11,12,15,16]);
cla(r1);
hold on
plot(xzCoordinates(1,last20Frameidx),xzCoordinates(2,last20Frameidx),'rx','linewidth',3)

plot(S_crt.X(:,1),S_crt.X(:,3),'gx')
hold off
axis equal
title('the trajectory of last 20 frames and landmarks view from above')


% set(gcf, 'GraphicsSmoothing', 'on')

% update indices
last20Frameidx = last20Frameidx + 1;

end