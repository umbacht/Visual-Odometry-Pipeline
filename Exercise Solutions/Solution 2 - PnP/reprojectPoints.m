function p_reproj = reprojectPoints(P, M_tilde, K)
% Reproject 3D points given a projection matrix
%
% P: [nx3] coordinates of the 3d points in the world frame
% M_tilde: [3x4] projection matrix
% K: [3x3] camera matrix
%
% p_reproj: [nx2] coordinates of the reprojected 2d points

p_homo = (K*M_tilde*[P';ones(1,length(P))])';
p_homo(:,1) = p_homo(:,1) ./ p_homo(:,3);
p_homo(:,2) = p_homo(:,2) ./ p_homo(:,3);

p_reproj = p_homo(:,1:2);

end

