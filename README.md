# Vision Programming Challenge

## Problem Statement
This task attempts to gain a better understanding of your knowledge of computer vision as well as your programming capability. Edge detection is a preliminary but important step in computer vision. The goal of this challenge is to detect the edges of a checkboard and highlight them with a green line superimposed on the image.

Different sample images are provided to test your code with. The ideal solution will be modular, object-oriented, and able to deal with different image sizes and different numbers of squares and their respective sizes. It should also be robust to noise and rotation.

The different filters used should be implemented in a modular and parameterizable form. Popular image processing libraries like OpenCV, which are open source, can be used. It is ideally expected that the candidate solves the problem using the C++ language. 
## Tasks and results
### 1. Basic 
For C++, the EdgeDetector.hpp and EdgeDetector.cpp files should be filled in. If any libraries are required, add them to the CMakeLists.txt. For Python, the edge_detector.py file should be filled in. Instructions must also be provided to download and install any required Python packages.

It should be possible to provide different images as input to your code and show the output as the image with the edges detected as green lines. Also, provide a Readme.md file detailing instructions on installation, implementation steps, concepts used, and possible improvements. 

### 2. Vision_ROS: 
Provide ROS .srv and .msg files required to create a ROS service for edge detection. Give example usage of this service with a client to detect edges for image files in a directory.

Additionally, detect edges for the images subscribed from an image topic that is available when the given ROS bag (.bag) file is played. The input image and detected edges should be visualised on RViz.

Convert the detected edge pixels from pixel coordinates (u, v) to 3D data (x, y, z) using the depth images and camera parameters also available in the .bag file. Publish the 3D data to a ROS topic (suggestion: of type sensor_msgs/PointCloud), with a topic name edge_points.

### 3. Robot_ROS: 
Extend the code further by visualizing the 3D edge points for each frame as RViz markers together with the visualization of a robot. You can use the robot URDF model provided to visualize the robot and multiple frames of reference. Please provide a video of the markers and the robot on Rviz for a duration of one loop of the given .bag file as part of the submission.

### 4. Advanced: 
Do all the above tasks.

