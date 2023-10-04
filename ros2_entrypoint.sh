#!/bin/bash

export GAZEBO_PLUGIN_PATH=${installPrefix}/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${installPrefix}/share/gazebo-11/models:${GAZEBO_MODEL_PATH}:/root/dd_ws/ardak/src/ardak/description/

# setup ros2 environment
source "/opt/ros/humble/setup.bash"

if [ -f "/root/dd_ws/ardak/install/setup.bash" ]; then
        source "/root/dd_ws/ardak/install/setup.bash"
fi

#if [ -f "/root/dd_ws/install/setup.bash" ]; then
#        source "/root/dd_ws/install/setup.bash"
#fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS='-j4'
alias build_amd="colcon build --install-base=install_amd64 --build-base=build_amd64"
alias build_arm="colcon build --install-base=install_arm64 --build-base=build_arm64 --packages-skip=realsense_gazebo_plugin"

alias xvfb="export DISPLAY=:1 && Xvfb $DISPLAY -screen 0 1024x768x16 &"
alias gzdeploy="npm run --prefix /root/dd_ws/gzweb deploy ---"
alias gzweb="npm run --prefix /root/dd_ws/gzweb/ start -p 10622 &"

exec "$@"
