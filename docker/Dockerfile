FROM ros:noetic

SHELL ["/bin/sh", "-c"]

RUN apt-get update && apt-get install -y \
  cmake \
  libx11-dev \
  python3-catkin-tools \
  python3-pip \
  ros-noetic-catkin \
  ros-noetic-vision-opencv \
  ros-noetic-image-transport

RUN pip3 install osrf-pycommon

