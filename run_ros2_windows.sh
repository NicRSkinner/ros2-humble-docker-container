#XAUTH=/home/nskinner/.Xauthority

 docker run -it --privileged \
		    -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v /mnt/wslg:/mnt/wslg \
                -e DISPLAY \
                -e WAYLAND_DISPLAY \
                -e XDG_RUNTIME_DIR \
                -e PULSE_SERVER \
                dockros bash

#docker run -it --net=host --privileged -v C:/Users/nicho/Documents/ardak/:/root/dd_ws/ardak -v C:/Users/nicho/Documents/ros-inc/:/opt/ros/humble/cp dockros bash

#    -v /home/nskinner/.gazebo:/root/.gazebo \