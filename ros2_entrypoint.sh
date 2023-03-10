#!/bin/bash
set -e

export GAZEBO_PLUGIN_PATH=${installPrefix}/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${installPrefix}/share/gazebo-11/models:${GAZEBO_MODEL_PATH}:/root/dd_ws/ardak/src/ardak/description/

# setup ros2 environment
source "/opt/ros/humble/setup.bash"
source "/root/dd_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS='-j4'
alias build_amd="colcon build --install-base=install_amd64 --build-base=build_amd64"
alias build_arm="colcon build --install-base=install_arm64 --build-base=build_arm64 --packages-skip=realsense_gazebo_plugin"

if [ "$(dpkg --print-architecture)" == "arm64" ]; then
    if [ -f "/root/dd_ws/ardak/install_arm64/setup.bash" ]; then
        source "/root/dd_ws/ardak/install_arm64/setup.bash"
    fi
fi

if [ "$(dpkg --print-architecture)" == "amd64" ]; then
    if [ -f "/root/dd_ws/ardak/install_amd64/setup.bash" ]; then
        source "/root/dd_ws/ardak/install_amd64/setup.bash"
    fi
fi

exec "$@"
