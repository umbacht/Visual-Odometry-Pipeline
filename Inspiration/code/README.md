# Vision Algorithms for Mobile Robotics: Mini Project

*Authors: Elias Asikainen, Robin Frauenfelder, Pascal Lieberherr, Silvan Löw*

In this Reposotory are all files needed to reproduce the results of our visulal odometry pipeline.

## Dependencies
Following programs / tollboxes are needed:
- `MATLAB 2020b`
- `Computer Vision Toolbox`
- `Image Processing Toolbox`
- `Statistics and Machine Learning Toolbox`

## Running the code

1. Clone this Reposotory:
    ```bash
    git clone https://gitlab.ethz.ch/visionmobilerobots/project
    ```
2. Run the initialisation file:
    ```bash 
    init_workspace.m
    ```
3. Set the wanted dataset parameter (`ds`) in the `main.m` file on line 11 (`0`: KITTI, `1`: Malaga, `2`: parking, `3`: rosie):

4. Run the main file;
    ```bash 
    main.m
    ```

## Videos

All videos are uploaded to YouTube here.
[KITTI](https://www.youtube.com/watch?v=yq8YdIr0Iig&feature=youtu.be), [Malaga](https://www.youtube.com/watch?v=hNJwbfJbpI4&feature=youtu.be), [Parking](https://www.youtube.com/watch?v=ZszFLWTXbAg&feature=youtu.be), [Rosie](https://www.youtube.com/watch?v=-2u6_rH_cnc&feature=youtu.be)

