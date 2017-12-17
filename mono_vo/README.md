Monocular visual odometry algorithm.

##Algorithm
Uses Nister's Five Point Algorithm for Essential Matrix estimation, and FAST features, with a KLT tracker.
The scale informaion is extracted from the KITTI dataset ground truth files.
More info: http://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

##Requirements
OpenCV 3.0
$ sudo apt-get install ros-indigo-opencv3

##How to compile?
You can compile the code as follows:
(in the mono_cv directory, type the following)

$ mkdir build
$ cd build
$ cmake ..
$ make

##How to run? 
After compilation, in the mono_cv directory, type the following:

$ cd build
$ ./monovo

##Before you run
You need to have the sequences from KITTI's Visual Odometry Dataset

To run this algorithm on your own data, you must modify the intrinsic calibration parameters and the scale 
information in the code.

For this exemple you have in the data directory a dataset (kitti00)
To unzip the data, type the following (in the data directory):

$ unzip kitti00.zip

### Lab
```
1) play the bagfile: demo_single.bag (step 4)

2) Examine the data: 
    2.1) what kind of data are in the bag file?
    2.2) display the data
    2.3) what can you conclude?
    
3) launch avg_lab.launch (step 3)

4) Examine the topics:
    4.1) list the topics
    4.2) display messages published to topics
    4.3) display the type information of topics
    4.4) what can you conclude?
    
5) Where is the camera position information?
    5.1) what can you conclude?
    
6) Write a simple subscriber and publisher to:
    6.1) take information about the camera position 
    6.2) publish the information as a "Pose" message
    6.2) write the camera position "trajectory" in a txt file
    
7) Using the camera position file (6.2), plot the data (you can use matlab or python)
    7.1) what can you conclude?
```