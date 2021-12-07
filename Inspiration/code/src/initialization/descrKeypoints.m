function [descr0, kp0_valid, descr1, kp1_valid] = descrKeypoints(img0, img1, kp0, kp1, params)
% The function derives the descriptors from pixels surrounding the
% keypoints
%
%   INPUT:
%           img0    : matrix NxM, first image
%           img1    : matrix NxM, second image
%           kp0     : matrix Kx2, 2D location of keypoints in img0
%           kp1     : matrix Kx2, 2D location of keypoints in img1
%           params  : the parameters
%
%   OUTPUT:
%           descr0      : matrix LxL, descriptor for kp0 in img0  
%           descr1      : matrix LxL, descriptor for kp1 in img1
%           kp0_valid   : matrix Px2, keypoints for which a descriptor was
%                                     found
%           kp1_valid   : matrix Px2, keypoints for which a descriptor was
%                                     found

%% Describe harris features

if params.use_matlab == 0
    
    % Decribe keypoints
    % Param
    descriptor_radius = params.descriptor_radius;
    % Description
    descr0 = describeKeypoints(img0, kp0, descriptor_radius); 
    descr1 = describeKeypoints(img1, kp1, descriptor_radius);
    kp0_valid = kp0;
    kp1_valid = kp1;
    
end

if params.use_matlab == 1
    
    % decribe keypoints 
    [descr0, kp0_valid] = extractFeatures(img0, kp0, ...
        'Method', 'Block', 'BlockSize', params.descriptor_radius);
    [descr1, kp1_valid] = extractFeatures(img1, kp1, ...
        'Method', 'Block', 'BlockSize', params.descriptor_radius);
    
    % DEBUG: Plot valid keypoints
    if params.init_debug == 1
        figure;
        imshow(img0);
        hold on;
        plot(kp0_valid);
        title('[descrKeypoints] valid keypoints kp0_{valid} in img0');
        hold off;
    end
    % DEBUG
    
end

end

