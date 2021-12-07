function kpt_locations = extractKeypoints(DoGs, contrast_threshold)
    num_octaves = numel(DoGs);
    kpt_locations = cell(1, num_octaves);
    for oct_idx = 1:num_octaves
       DoG = DoGs{oct_idx};
       % This is some neat trick to avoid for loops.
       DoG_max = imdilate(DoG, true(3, 3, 3));
       % Equivalent to this is:
       % DoG_max = movmax(movmax(movmax(DoG, 3, 1), 3, 2), 3, 3);
       is_kpt = (DoG == DoG_max) & (DoG >= contrast_threshold);
       % We do not consider the extrema at the boundaries of the DoGs.
       is_kpt(:, :, 1) = false;
       is_kpt(:, :, end) = false;
       [x, y, s] = ind2sub(size(is_kpt), find(is_kpt));
       kpt_locations{oct_idx} = horzcat(x, y, s);
    end
end