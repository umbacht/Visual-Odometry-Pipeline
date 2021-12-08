function derotated_patch = derotatePatch(img, loc, patch_size, ori)

% The patch can overlap at most sqrt(2)/2 * patch_size over the image edge.
% To prevent this, pad the image with zeros
padding = ceil(sqrt(2) * patch_size / 2);
derotated_patch = zeros(patch_size,patch_size);
padded_img = padarray(img, [padding, padding]);

% compute derotated patch  
for px=1:patch_size
    for py=1:patch_size
      x_origin = px - 1 - patch_size/2;
      y_origin = py - 1 - patch_size/2;

      % rotate patch by angle ori
      x_rotated = cos(pi*ori/180) * x_origin - sin(pi*ori/180) * y_origin;
      y_rotated = sin(pi*ori/180) * x_origin + cos(pi*ori/180) * y_origin;

      % move coordinates to patch
      x_patch_rotated = loc(2) + x_rotated;
      y_patch_rotated = loc(1) - y_rotated;

      % sample image (using nearest neighbor sampling as opposed to more
      % accuracte bilinear sampling)
      y_img_padded = ceil(y_patch_rotated + padding);
      x_img_padded = ceil(x_patch_rotated + padding);
      derotated_patch(py, px) = padded_img(y_img_padded, x_img_padded);
    end
end
      
end

