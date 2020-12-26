FROM ros:kinetic

SHELL ["/bin/sh", "-c"]

RUN apt-get update && apt-get install -y \
  cmake \
  libx11-dev \
  python-catkin-tools \
  ros-kinetic-catkin \
  ros-kinetic-opencv3 \
  ros-kinetic-cv-bridge \
  ros-kinetic-image-transport \
  ros-kinetic-usb-cam \
  python-pip

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
