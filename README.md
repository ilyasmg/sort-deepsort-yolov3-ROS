# YOLO + SORT/DEEP SORT for ROS (not maintained anymore)
Tracker ROS node (sort and deep sort) using darknet_ros (YOLOv3).
Detected bounding boxes from YOLO are used by the sort tracker.  
This has been done using ROS Kinetic, python2 and OpenCV2. If you need a modern ros wrapper for SORT, you should use newer versions of ROS, OpenCV and python.

## Installation
In order to install darknet_ros, clone the latest version using SSH (see [how to set up an SSH key](https://confluence.atlassian.com/bitbucket/set-up-an-ssh-key-728138079.html)) into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
    cd ../
    catkin build darknet_ros
In order to install sort_track, clone this repository in your catkin workspace and compile the package using ROS

    cd src
    git clone https://github.com/ilyas95/sort-deepsort-yolov3-ROS
    catkin build sort_track
   
## How to use  
Remember to source your workspace in every new terminal window by using

    source ~/catkin_workspace/devel/setup.bash
Run darknet_ros detector (For my project I used mainly YOLOv3-tiny but it should work for the other YOLOs)
    
    roslaunch darknet_ros yolo_v3-tiny.launch
If you use the computer camera, you may need to install usb_cam ROS package and run this before running darknet_ros node:

    roslaunch usb_cam usb_cam-test.launch
If you still have Waiting for image. message, try to see in the file ros.yaml what is the camera topic
Then you can choose between:
- SORT  


To run:

    roslaunch sort_track sort.launch
    
    
- DEEP SORT


Before running go to catkin_workspace/src/sort_track/src and open track_deep.py
In line 90, modify model_filename to your directory
To run:

    roslaunch sort_track sort_deep.launch
IMPORTANT!! Before running the tracker nodes, you have to make the python scripts executable.
For SORT, go to catkin_workspace/src/sort-deepsort-yolov3-ROS/sort_track/src and make track.py executable.
For DEEP SORT, go to the same folder and make track_deep.py executable.

## Disclaimer

This project is using code from:

[abewley/sort](https://github.com/abewley/sort): Simple, online, and realtime tracking of multiple objects in a video sequence.

[nwojke/deep_sort](https://github.com/nwojke/deep_sort): Simple Online Realtime Tracking with a Deep Association Metric.

[leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros): YOLO ROS: Real-Time Object Detection for ROS
