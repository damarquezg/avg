### Monocular visual odometry algorithm.
```
## Algorithm
Uses Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker.
The scale informaion is extracted from the KITTI dataset ground truth files.
More info: http://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

## Requirements
OpenCV 3.0
$ sudo apt-get install ros-indigo-opencv3

## How to compile?
You can compile the code as follows:
(in the mono_cv directory, type the following)

$ mkdir build
$ cd build
$ cmake ..
$ make

## How to run? 
After compilation, in the mono_cv directory, type the following:

$ cd build
$ ./monovo

## Before you run
You need to have the sequences from KITTI's Visual Odometry Dataset

To run this algorithm on your own data, you must modify the intrinsic calibration parameters and the scale 
information in the code.

For this exemple you have in the data directory a dataset (kitti00)
To unzip the data, type the following (in the data directory):

$ unzip kitti00.zip
```

### Lab
```
1) Read : 
http://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
`http://www.example.com`
[link to Google!](http://google.com)

2) Examine the data: 
    2.1) what kind of data are in the dataset?
    2.2) what can you conclude?
    
3) Examine and run the code:
    3.1) From code, write a detailed pseudo-code (algorithm),
    3.2) what can you conclude?
 
4) Where is the camera position information?
    4.1) what can you conclude?
    
5) Write a simple function to:
    5.1) take information about the camera position 
    5.2) write the camera position "trajectory" in a txt file
    
6) Using the camera position file (5.2), plot the data (you can use matlab or python)
    6.1) what can you conclude?
```
