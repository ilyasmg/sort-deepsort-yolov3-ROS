# YOLO + SORT/DEEP SORT for ROS
Tracker ROS node (sort and deep sort) using darknet_ros (YOLOv3).
Detected bounding boxes from YOLO are used by the sort tracker.

## Installation
In order to install darknet_ros, clone the latest version using SSH (see [how to set up an SSH key](https://confluence.atlassian.com/bitbucket/set-up-an-ssh-key-728138079.html)) into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
    cd ../
    catkin build darknet_ros
In order to install sort_track, clone this repository in your catkin workspace and compile the package using ROS

    cd src
    git clone https://github.com/ilyas95/sort-deepsort-yolov3-ROS
    catkin build sort_tracker

## Disclaimer

This project is using code from:

[abewley/sort](https://github.com/abewley/sort): Simple, online, and realtime tracking of multiple objects in a video sequence.

[nwojke/deep_sort](https://github.com/nwojke/deep_sort): Simple Online Realtime Tracking with a Deep Association Metric.

[leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros): YOLO ROS: Real-Time Object Detection for ROS
