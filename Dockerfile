
########################
# Base for ROS install - helps with development

ARG ROS_DISTRO=noetic
# FROM ros:$ROS_DISTRO-ros-base
FROM husarion/panther:$ROS_DISTRO as rmit_panther_rosbase

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository and initialise ROS workspace
RUN apt update && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    apt install -y vim ros-$ROS_DISTRO-joy && \
    apt install -y ros-noetic-pcl-ros

# Get and unzip the opencv repository.
# https://docs.opencv.org/4.7.0/d7/d9f/tutorial_linux_install.html
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.7.0.zip
RUN unzip opencv.zip
RUN mkdir -p build && cd build

# Configure.
RUN cmake ../opencv-4.7.0

# Build.
RUN cmake --build

########################
# Build software for RMIT Panther
ARG ROS_DISTRO=noetic
FROM rmit_panther_rosbase

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws
COPY ./noetic_workspace/src/first_pkg ./src/first_pkg
COPY ./noetic_workspace/src/multiple-object-tracking-lidar-master ./src/multiple-object-tracking-lidar-master

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make

# This should not be required - replaced with above
# RUN apt update && \
#     source /opt/ros/$ROS_DISTRO/setup.bash && \
#     rosdep update --rosdistro $ROS_DISTRO && \
#     rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
#     catkin_make && \
#     apt autoremove -y && \
#     apt clean && \
#     rm -rf /var/lib/apt/lists/*


COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

