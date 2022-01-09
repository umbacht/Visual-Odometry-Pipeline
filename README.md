# Visual-Odometry-Pipeline
Mini-project for the course: *Vision Algorithms for Mobile Robotics*

Authors: Wenqing Chang @wchang1011, Jeroen Riessbacher @Tarzanio, Luca Di Pierno @ApocalyptoNicci, Thomas Umbach @umbacht

## Instructions to run the code:

### Directory setup:
1. All .m files in one file named "code" including the file "Exercise Solutions" containing the exercise solution scripts.
2. All datasets in file named "datasets" on same level as "code" file. 
   Relative paths for datasets:
   - "../datasets/kitti/"
   - "../datasets/malaga-urban-dataset-extract-07/"
   - "../datasets/parking/"
   - "../datasets/walking/"


### Running the code

- In main.m file: Set parameter ds to chose which dataset 
	- ds = 0: KITTI
	- ds = 1: Malaga
	- ds = 2: Parking
	- ds = 3: Self recorded walking
	
- Run main.m

## Videos of datasets:
- [KITTI](https://youtu.be/5besGTteSaU)
- [Malaga]( https://youtu.be/tz77heKUBBE)
- [Parking](https://youtu.be/SbBriyLgZg0)
- [Walking (own)](Part 1: https://youtu.be/IumEqt9J7Uo, ...
                  Part 2: https://youtu.be/kZDZKcieDFQ)

### Specifications of machine on which screencasts were captured:
- OS: MacBook Pro (13-inch, 2018, Four Thunderbolt 3 Ports)
- Max. number of threads: 1
- CPU Freq.: 2.7 GHz Quad-Core Intel Core i7
- RAM: 16 GB 2133 MHz LPDDR3

## Needed Toolboxes:
- MATLAB_R2020b
- Computer Vision Toolbox
- Statistics and Machine Learning Toolbox