function blurred_images = computeBlurredImages(image_pyramid, num_scales, sigma)
    num_octaves = numel(image_pyramid);
    blurred_images = cell(1, num_octaves);
    for oct_idx = 1:num_octaves
        imgs_per_oct = num_scales + 3;
        octave_stack = zeros([size(image_pyramid{oct_idx}) imgs_per_oct]);
        for stack_idx = 1:imgs_per_oct
            s = stack_idx - 2;
            % Hence s in {-1, ..., num_scales + 1}
            octave_stack(:, :, stack_idx) = imgaussfilt(image_pyramid{oct_idx}, sigma * 2^(s / num_scales));
        end
       blurred_images{oct_idx} = octave_stack;
    end
end
