function F = fundamentalEightPoint_normalized(p1, p2)
% estimateEssentialMatrix_normalized: estimates the essential matrix
% given matching point coordinates, and the camera calibration K
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix
%

% Normalize each set of points so that the origin
% is at centroid and mean distance from origin is sqrt(2).
[p1_tilda,T1] = normalise2dpts(p1);
[p2_tilda,T2] = normalise2dpts(p2);

% Linear solution
F = fundamentalEightPoint(p1_tilda,p2_tilda);

% Undo the normalization
F = (T2.') * F * T1;

end

