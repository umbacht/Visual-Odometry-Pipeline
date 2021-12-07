function [T_c0_c1, landmarks] = triangLandmarks(matchedPoints0, matchedPoints1, rel_orient, rel_loc, params)
% Triangulates the matched keypoints into 3D landmarks represented in WCS.
% Therefore, the camera projection matrix camMatrix is computed.
%
%    INPUTS:
%               matchedPoints0 : matrix Qx2, keypoints in img0 which were 
%                                matched witch keypoints in img1
%               matchedPoints1 : matrix Qx2, keypoints in img1 which were 
%                                matched witch keypoints in img0
%               pose_c1_c0     : pose of camera 1 w.r.t. camera 0
%               params         : The parameters
%
%    OUTPUTS:
%               M_w_c1         : matrix 4x4, transformation matrix from 
%                                camera 1 to WCS i.e. camera 0
%               landmarks      : matrix Nx3, matched keypoints
%                                triangulated to WCS

%% Compute the extrinsics i.e. R_c0_c1 = R_w_c1, t_c0_c1 = t_w_c1

% frame of cam0 is the origin of WCS
[R_c1_c0, t_c1] = cameraPoseToExtrinsics(rel_orient, rel_loc);
t_c1 = t_c1'; % for convention

% Homogenous transformation matrix from camera1 origin to WCS-origin
% M_w_c1 = [R_w_c1, t_w_c1'; 0 0 0 1];

% Compute the camera matrix = K[R|t]
camMatrix0 = cameraMatrix(params.camera, eye(3), zeros(3,1)); 
camMatrix1 = cameraMatrix(params.camera, R_c1_c0, t_c1);

% T_c0_c1 = [R_c0_c1 | t_c0]
R_c0_c1 = inv(R_c1_c0);
T_c0_c1 = [ R_c0_c1, -R_c0_c1*t_c1 ];

% triangulate Landmarks
landmarks = triangulate(matchedPoints0, matchedPoints1, camMatrix0, camMatrix1);

end

