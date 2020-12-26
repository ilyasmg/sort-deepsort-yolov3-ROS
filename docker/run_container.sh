#!/bin/bash

xhost +

docker run --runtime=nvidia --rm -it -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all --privileged -v $1:/ros_ws/ noetic_ws:latest 