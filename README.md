# AA241x Vision Example Code #

This ROS package is a skeleton for the teams of Spring 2019's AA241x course.  The purpose of this package is the control of the PiCam v2 camera that is connected to the Raspberry Pi 3B+ and the detection of the AprilTag information.  This package has been created separately from the rest of the mission / command and control nodes as the dependencies for the nodes related to the camera and imagine are quite different than the dependencies for the other nodes required for the execution of the AA241x mission.  Furthermore, the separation allows teams to test the mission / command and control nodes on the desktops running Gazebo (i.e. not the Raspberry Pi 3B+) without any compilation problems that would occur due to the fact that the nodes in this package are very specific to the Raspberry Pi 3B+ hardware (i.e. the PiCam v2).

This skeleton package currently contains the following nodes:

 - [`vision_node`](#vision-node): a skeleton to help you get started with a node to be able to retrieve and process images from the PiCam v2 for the purpose of detecting AprilTags.  At the moment it contains an example that reads from the camera and runs a basic level of detection for the 16h5 family of AprilTags.

 - [`display_node`](#display-node): a helper node to allow the visualization of the image frames retrieved from the PiCam v2.  *Note: this node should NOT be run while in flight as it does use a lot of resources and will not be useful in flight; this should only be used for debugging.*

## Quick Start ##

The follow sections will help you get started with this ROS package.

### Getting the Code ###

It is recommended that you first [fork](https://help.github.com/en/articles/fork-a-repo) the repository to allow you to make changes and build off the code as desired in your own fork of the code.

Once forked, you will be able to clone your repository on to your team's Raspberry Pi 3B+.  This is a ROS package that has been designed with the `catkin_ws` that is used with ROS Melodic, so make sure to clone this repository into the `catkin_ws/src` directory, where ROS packages live.

For example:

```sh
cd ~/catkin_ws/src
git clone https://github.com/<your-github-handle>/aa241x_vision.git
```

Once you have it cloned, you can build the code using `catkin_make`:

```sh
cd ~/catkin_ws/
catkin_make
```

### Running the Code ###

Each of the nodes of this package can be run using the basic [`rosrun`](http://wiki.ros.org/rosbash#rosrun) framework or can be run using the example launch file using `roslaunch`.  For a tutorial on `roslaunch` check out either [this resource](http://www.clearpathrobotics.com/assets/guides/ros/Launch%20Files.html) or [this resource](http://wiki.ros.org/roslaunch#Tutorials).  The example launch file (`vision.launch`) will start both the `vision_node` and the `display_node` to demonstrate the debugging of viewing the camera images with frame images of size `640x480`.

To run using the launch file:

```sh
cd ~/catkin_ws/
source devel/setup.bash
roslaunch aa241x_vision vision.launch
```

**Note:** The launch file contains 2 arguments that can be set: `frame_width` and `frame_height` that control the size of the image frame to read from the camera and display.  Those parameters can be specified as launch file parameters using the following syntax:

```sh
roslaunch aa241x_vision vision.launch frame_width:=640 frame_height:=480
```

**NOTE:** nodes can have parameters that define or alter some of the functionality when the node is run, which is being leveraged in the vision and display nodes for adjusting the size of the image being shown.  For more details on node parameters, check out [this ROS help page on Nodes](http://wiki.ros.org/Nodes).

### Dependencies ###

This code has the following dependencies:

 - [RaspiCam C++](https://github.com/cedricve/raspicam) - the library that enables interfacing with the PiCam v2 using C++. **Note:** the library uses the OpenCV [VideoCapture](https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html) class and enables setting [many different parameters](https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d) that control the behavior of the camera.  Most notably used in this example are:
     + `cv::CAP_PROP_FORMAT` - defines the matrix type to read the image to, currently set to `CV_8UC1` which is a single channel 8-bit matrix, resulting in a grayscale image.  For a color image, you would use `CV_8UC3` (3 channel 8-bit image).
     + `cv::CAP_PROP_FRAME_WIDTH` - defines the width of the image to read from the camera (max value is `1280`).
     + `cv::CAP_PROP_FRAME_HEIGHT` - defines the width of the image to read from the camera (max value is `960`).

 - [APrilTag Library](https://github.com/AprilRobotics/apriltag) - this is the original AprilTag library in C put out by the April Robotics Lab.  **This is not the only library out there and may not be the best performance library.**


## Nodes ##

Here is a more detailed description of the different nodes.

### Vision Node ###

The vision node does the heavy lifting of reading an image from the camera and running the image through the AprilTag detector.  Depending on the input flags, those images, annotated with the AprilTag information will be published so the (Display Node)[#display-node] can display the images.

Parameters for the node:
 - `frame_width`: integer number of pixels for the width of the image frame
 - `frame_height`: integer number of pixels of the height of the image frame
 - `display_video`: a boolean flag to determine whether or not to publish video frames for the display node to display

**NOTE:** If the `display_video` parameter is `false`, images will never display, regardless of whether or not the display node is running!

### Display Node ###

Displays the images using an OpenCV image frame.

Parameters for the node:
 - `frame_width`: integer number of pixels for the width of the image frame
 - `frame_height`: integer number of pixels of the height of the image frame


## Camera Calibration ##

When it comes to doing pose estimation of the tag, there are two key pieces of information that are needed: the dimension of the tag, and the **camera intrinsics** (effectively a set of parameters that define model the camera).

We will be giving you the information on the tag size and tag family that will be used for the landing platform.

The **camera intrinsics** is information that can be determined through camera calibration.  Thankfully, OpenCV provides a helpful [tutorial and example script for calibrating a camera](https://docs.opencv.org/3.2.0/d4/d94/tutorial_camera_calibration.html).  The tutorial also provides some good information on the meaning of the different parameters for the camera and how to go through a camera calibration.  To help run the [OpenCV example code](https://github.com/opencv/opencv/tree/master/samples/cpp/tutorial_code/calib3d/camera_calibration) reference in that tutorial, we have made a [repository to compile the camera calibration example to get you started](https://github.com/aa241x/camera-calib).