% Initialize workspace: add folders and subfolders to path
thisfile = which(mfilename);
mainfolder = fileparts(thisfile);
cd(mainfolder);
addpath(genpath(mainfolder));
clear thisfile mainfolder