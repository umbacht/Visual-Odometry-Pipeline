function W = getSimWarp(dx, dy, alpha_deg, lambda)
% alpha given in degrees, as indicated

alpha_rad = alpha_deg * pi / 180;
c = cos(alpha_rad);
s = sin(alpha_rad);
R = [c -s; s c];
W = lambda * [R [dx; dy]];

end

