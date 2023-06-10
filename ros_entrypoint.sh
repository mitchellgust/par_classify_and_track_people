#!/bin/bash

# This file is required by the Dockerfile

set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros_ws/devel/setup.bash"

exec "$@"
