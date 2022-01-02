clear all;
close all;

% Set directory
walking_path = '../datasets/walking_horz_video';
addpath(walking_path);
addpath(genpath('Exercise Solutions'));


% K Parameter
K = load([walking_path '/K.txt']);

last_frame = 2116;

for i = 1:last_frame
    image = imread([walking_path '/images/' sprintf('Image_%d.jpg',i)]);
    imshow(image);
    pause(0.01)
end






