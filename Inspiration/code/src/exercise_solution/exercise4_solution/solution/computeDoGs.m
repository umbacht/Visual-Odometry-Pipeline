function DoGs = computeDoGs(blurred_images)
    num_octaves = numel(blurred_images);
    DoGs = cell(1, num_octaves);
    for oct_idx = 1:num_octaves
        DoG =  zeros(size(blurred_images{oct_idx})-[0 0 1]);
        num_dogs_per_octave = size(DoG, 3);
        for dog_idx = 1:num_dogs_per_octave
           DoG(:, :, dog_idx) = abs(...
               blurred_images{oct_idx}(:, :, dog_idx + 1) - ...
               blurred_images{oct_idx}(:, :, dog_idx));
        end
        DoGs{oct_idx} = DoG;
    end
end