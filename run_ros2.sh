XAUTH=/home/$USER/.Xauthority

docker run -it \
    --net=host \
    --privileged \
    -p $1:8800 \
    -e XDG_RUNTIME_DIR=/tmp \
    -e DISPLAY=$DISPLAY \
    -e GDK_BACKEND=wayland \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY  \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --gpus all \
    --ipc=host \
    -v /run/udev:/run/udev:ro \
    -v /dev:/dev \
    -v /home/$USER/.gazebo:/root/.gazebo \
    -v /home/$USER/Documents/ardak/:/root/dd_ws/ardak \
    -v /home/$USER/Documents/ros-inc/:/opt/ros/foxy/cp \
    -v /home/$USER/Documents/cp:/root/dd_ws/cp \
    dockros  \
    bash
