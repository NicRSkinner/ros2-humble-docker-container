#XAUTH=/home/nskinner/.Xauthority

docker run -it --net=host --privileged -v C:/Users/nicho/Documents/ardak/:/root/dd_ws/ardak -v C:/Users/nicho/Documents/ros-inc/:/opt/ros/humble/cp dockros bash

#    -v /home/nskinner/.gazebo:/root/.gazebo \