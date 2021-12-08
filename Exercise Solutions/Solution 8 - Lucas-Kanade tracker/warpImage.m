function I = warpImage(I_R, W)

I = zeros(size(I_R));

% Somehow pre-calculating this resulted in major speedup.
max_coords = fliplr(size(I_R));

% Careful with indexing!
for x = 1:size(I_R, 2)
    for y = 1:size(I_R, 1)
        warped = (W * [x y 1]')';
        if all(warped < max_coords & warped > [1 1])
            I(y, x) = I_R(int32(warped(2)), int32(warped(1)));
        end
    end
end

end

