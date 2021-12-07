function image_pyramid = computeImagePyramid(image, num_octaves)
    image_pyramid = cell(1, num_octaves);
    image_pyramid{1} = image;
    for idx = 2:num_octaves
       image_pyramid{idx} = imresize(image_pyramid{idx - 1}, 0.5);
    end
end