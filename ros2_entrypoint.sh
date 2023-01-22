#!/bin/bash
set -e

export GAZEBO_PLUGIN_PATH=${installPrefix}/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${installPrefix}/share/gazebo-11/models:${GAZEBO_MODEL_PATH}:/root/dd_ws/ardak/src/ardak/description/

# setup ros2 environment
source "/opt/ros/humble/setup.bash"
source "/root/dd_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS='-j4'

if [ -f "/root/dd_ws/ardak/install/setup.bash" ]; then
    source "/root/dd_ws/ardak/install/setup.bash"
fi

exec "$@"
